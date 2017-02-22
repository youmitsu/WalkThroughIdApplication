#include "walk_through_id.h"

#include <thread>
#include <chrono>

#include <omp.h>

#include "stdlib.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "fftw3.h"

#define _USE_MATH_DEFINES
#include <math.h>
#define POS_RATE 1000
#define FREQ_SIZE 12
#define WALK_JOINT_COUNT 3
#define WALK_VEC_SIZE FREQ_SIZE*WALK_JOINT_COUNT
#define WALK_JUDGE_THRESH -1.0

#define TRAIN_MEMBER 3
#define TRAIN_DATA_SIZE 20
#define DATA_SIZE TRAIN_MEMBER*TRAIN_DATA_SIZE 

#pragma comment(lib, "libfftw3-3.lib")
#pragma comment(lib, "libfftw3f-3.lib")
#pragma comment(lib, "libfftw3l-3.lib")

// Constructor
WalkThroughId::WalkThroughId()
{
    // Initialize
    initialize();
}

// Destructor
WalkThroughId::~WalkThroughId()
{
    // Finalize
    finalize();
}

void WalkThroughId::initialize()
{
	frame_count = 0;
	import_walk_judge_temp();
	create_classifier();
    vector<vector<float>> features;
    dynamic_features = features;
    for(int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
        vector<float> v;
        dynamic_features.push_back(v);
    }
}

void WalkThroughId::finalize(){

}

//分類器構築
void WalkThroughId::create_classifier(){
	float labels_data[DATA_SIZE];
	for (int i = 0; i < TRAIN_MEMBER; i++){
		for (int j = 0; j < TRAIN_DATA_SIZE; j++){
			labels_data[i*TRAIN_DATA_SIZE + j] = (float)i;
		}
	}
	Mat labels(DATA_SIZE, 1, CV_32FC1, labels_data);

	float data[DATA_SIZE][DYNAMIC_FEATURE_COUNT];  //訓練データ保持の配列(あとでMatにぶちこむ)

	const string input_dynamic_filename = "_output_features.dat";
	const string person_names[TRAIN_MEMBER] = { "mitsuhori", "koyama", "hiroki" };

	int d = 0;
	for (int i = 0; i < TRAIN_MEMBER; i++){
		string filename = person_names[i] + input_dynamic_filename;
		ifstream input_file;
		input_file.open(filename);
		if (input_file.fail()){
			cout << "ファイルが見つかりません." << endl;
			cin.get();
		}
		string str;
		int f = 0;  //特徴量番号
		while (getline(input_file, str)){
			string tmp;
			istringstream stream(str);
			int c = 0;
			while (getline(stream, tmp, ' ')){
				float val = stof(tmp);
				data[d][f] = val;
				f++;
			}
			//データ1セット読み終わったらリセット
			if (f == FEATURE_SIZE){
				f = 0;
				d++;
			}
		}
	}
	Mat training_data(DATA_SIZE, FEATURE_SIZE, CV_32F, data);

	/****************SVMパラメータの設定****************/
	CvSVMParams params;
	params.svm_type = SVM::C_SVC;
	params.kernel_type = SVM::RBF;
	params.C = 1.0;
	params.degree = 1.2;
	params.gamma = 0.0005;
	params.coef0 = 1.0;

	svm.train(training_data, labels, Mat(), Mat(), params);
}

void WalkThroughId::execute(array<Joint, JointType::JointType_Count>& joints, bool isValidData){
	shift_count();
	if (isValidData){
		insert_features(joints);
		if (walking_judge()){
			if (calc_spectrums()){
				
			}
		}
		else{
			cout << "false" << endl;
		}
	}
	else{
		insert_empty_features();
	}
	frame_count++;
}

bool WalkThroughId::calc_spectrums(){
	int d = 0;
	spectrums_input = Mat(1, FEATURE_SIZE, CV_32F);
	for (int j = 0; j < DYNAMIC_FEATURE_COUNT; j++){
		fftw_complex *in = NULL;
		fftw_complex *out = NULL;
		fftw_plan p = NULL;
		int size = FREQ_SIZE;
		size_t mem_size = sizeof(fftw_complex)* size;
		in = (fftw_complex*)fftw_malloc(mem_size);
		out = (fftw_complex*)fftw_malloc(mem_size);

		if (!in || !out){
			fprintf(stderr, "failed to allocate %d[byte] memory(-.-)\n", (int)mem_size);
			return false;
		}

		p = fftw_plan_dft_1d(size, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
		for (int i = start_frame; i <= end_frame; i++){
			in[i][0] = dynamic_features[j][i];
			in[i][1] = 0;
		}

		fftw_execute(p);

		// output is DC exchanged and scaled.
		double scale = 1. / size;
		double re, im, mag, ang;
		for (int i = 0; i < size; i++){
			re = out[i][0] * scale;
			im = out[i][1] * scale;
			mag = sqrt(re*re + im*im);
			ang = atan2(im, re);
			if (i == 1 || i == 2 | i == 3){
				spectrums_input.at<float>(0, d) = mag;
				d++;
			}
		}

		if (p) fftw_destroy_plan(p);
		if (in) fftw_free(in);
		if (out) fftw_free(out);
	}
}

void WalkThroughId::execute_classification(){
	int result = (int)svm.predict(spectrums_input);
}

void WalkThroughId::shift_count(){
	if (frame_count == FREQ_SIZE-1){
		start_frame = 0;
		end_frame = FREQ_SIZE-1;
	}
	else if (frame_count >= FREQ_SIZE-1){
		start_frame++;
		end_frame++;
	}
}

bool WalkThroughId::walking_judge(){

	if (frame_count < FREQ_SIZE){
		return false;
	}

	float sum = 0;
	float val;
	for (int i = 0; i < WALK_JOINT_COUNT; i++){
		int joint_type = walking_judge_features[i];
		for (int j = start_frame; j <= end_frame; j++){
			val = dynamic_features[joint_type][j];
			walking_judge_input.push_back(val);
			sum += val;
		}
	}
	float input_mean = sum / WALK_VEC_SIZE;

	double sxx = 0.0;
	double syy = 0.0;
	double sxy = 0.0;
	for (int i = 0; i < WALK_VEC_SIZE; i++){
		sxx += (walking_judge_input[i] - input_mean)*(walking_judge_input[i] - input_mean);
		syy += (walking_judge_temp[i] - temp_mean)*(walking_judge_temp[i] - temp_mean);
		sxy += (walking_judge_input[i] - input_mean)*(walking_judge_temp[i] - temp_mean);
	}
	double R = sxy / sqrt(sxx*syy);
	cout << R << endl;

	walking_judge_input.clear();
	if (R > WALK_JUDGE_THRESH){
		return true;
	}
	else{
		return false;
	}
}

void WalkThroughId::import_walk_judge_temp(){
	ifstream input_datafile;
	input_datafile.open("template.dat");
	if (input_datafile.fail()){
		cout << "ファイルが見つかりません" << endl;
		cin.get();
	}
	string str;
	int d = 0;
	float sum = 0;
	float val;
	while (getline(input_datafile, str)){
		val = stof(str);
	    walking_judge_temp.push_back(val);
		sum += val;
		d++;
	}
	temp_mean = sum / WALK_VEC_SIZE;
}

void WalkThroughId::insert_features(array<Joint, JointType::JointType_Count>& joints){
	insert_dynamic_features(joints);
	//insert_static_features(joints);
}

void WalkThroughId::insert_dynamic_features(array<Joint, JointType::JointType_Count>& joints){
	for (int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
		vector<int> use_joints = dynamic_feature_use_joints[i];
		float angle;
		if (use_joints.size() == 3){
			Joint p = joints[use_joints[0]];
			Point3f p1(p.Position.X, p.Position.Y, p.Position.Z);
			p = joints[use_joints[1]];
			Point3f p2(p.Position.X, p.Position.Y, p.Position.Z);
			p = joints[use_joints[2]];
			Point3f p3(p.Position.X, p.Position.Y, p.Position.Z);
			angle = evaluate_angle(p1, p2, p3);
		}
		else if (use_joints.size() == 4){
			Joint p = joints[use_joints[0]];
			Point3f p1(p.Position.X, p.Position.Y, p.Position.Z);
			p = joints[use_joints[1]];
			Point3f p2(p.Position.X, p.Position.Y, p.Position.Z);
			p = joints[use_joints[2]];
			Point3f p3(p.Position.X, p.Position.Y, p.Position.Z);
			p = joints[use_joints[3]];
			Point3f p4(p.Position.X, p.Position.Y, p.Position.Z);
			angle = evaluate_seperated_angle(p1, p2, p3, p4);
		}
		dynamic_features[i].push_back(angle);
	}
}

void WalkThroughId::insert_static_features(array<Joint, JointType::JointType_Count>& joints){

}

void WalkThroughId::insert_empty_features(){
	for (int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
		dynamic_features[i].push_back(0.0);
	}
}

float WalkThroughId::calc_tall(){
	return 0;
}

float WalkThroughId::calc_shoulderLen(){
	return 0;
}

float WalkThroughId::calc_armLeftLen(){
	return 0;
}

float WalkThroughId::calc_armRightLen(){
	return 0;
}

float WalkThroughId::calc_bodyLen(){
	return 0;
}

float WalkThroughId::calc_length(){
	return 0;
}

float WalkThroughId::evaluate_angle(Point3f c, Point3f a, Point3f b){
	int ax_cx = (c.x - a.x)*POS_RATE;
	int ay_cy = (c.y - a.y)*POS_RATE;
	int az_cz = (c.z - a.z)*POS_RATE;
	int bx_cx = (c.x - b.x)*POS_RATE;
	int by_cy = (c.y - b.y)*POS_RATE;
	int bz_cz = (c.z - b.z)*POS_RATE;
	float cos = ((ax_cx*bx_cx) + (ay_cy*by_cy) + (az_cz*bz_cz)) / ((sqrt((ax_cx*ax_cx) + (ay_cy*ay_cy) + (az_cz*az_cz))*sqrt((bx_cx*bx_cx) + (by_cy*by_cy) + (bz_cz*bz_cz))));
	float angle = acosf(cos);
	if (cos > -1.0 && cos < 0.0){
		angle = M_PI - angle;
	}
	return angle;
}

float WalkThroughId::evaluate_seperated_angle(Point3f pA, Point3f pB, Point3f pC, Point3f pD){
	int ax_bx = (pA.x - pB.x)*POS_RATE;
	int ay_by = (pA.y - pB.y)*POS_RATE;
	int az_bz = (pA.z - pB.z)*POS_RATE;
	int cx_dx = (pC.x - pD.x)*POS_RATE;
	int cy_dy = (pC.y - pD.y)*POS_RATE;
	int cz_dz = (pC.z - pD.z)*POS_RATE;
	float cos = ((ax_bx*cx_dx) + (ay_by*cy_dy) + (az_bz*cz_dz)) / ((sqrt((ax_bx*ax_bx) + (ay_by*ay_by) + (az_bz*az_bz))*sqrt((cx_dx*cx_dx) + (cy_dy*cy_dy) + (cz_dz*cz_dz))));
	float angle = acosf(cos);
	if (cos > -1.0 && cos < 0.0){
		angle = -angle;
	}
	return angle;
}

float WalkThroughId::evaluate_dist(Point3f a, Point3f b){
	float d = sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
	return d;
}