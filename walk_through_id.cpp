#include "walk_through_id.h"

#include <thread>
#include <chrono>

#include <omp.h>

#include "stdlib.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>
#define POS_RATE 1000
#define FREQ_SIZE 12
#define WALK_JOINT_COUNT 3
#define WALK_VEC_SIZE FREQ_SIZE*WALK_JOINT_COUNT
#define WALK_JUDGE_THRESH 0.6

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
    vector<vector<float>> features;
    dynamic_features = features;
    for(int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
        vector<float> v;
        dynamic_features.push_back(v);
    }
}

void WalkThroughId::finalize(){

}

void WalkThroughId::execute(array<Joint, JointType::JointType_Count>& joints, bool isValidData){
	shift_count();
	if (isValidData){
		insert_features(joints);
		if (walking_judge()){
			cout << "true" << endl;
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
		cout << "ƒtƒ@ƒCƒ‹‚ªŒ©‚Â‚©‚è‚Ü‚¹‚ñ" << endl;
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