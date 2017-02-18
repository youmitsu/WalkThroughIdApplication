#include <opencv2/opencv.hpp>

#include <vector>
#include <array>
#include <Kinect.h>

using namespace std;
using namespace cv;

#define ID_COUNT 9
#define DYNAMIC_FEATURE_COUNT 6
#define STATIC_FEATURE_COUNT 5

enum member{
	MITSUHORI = 1,
	HIROKI = 2,
	KOYAMA = 3,
	KAJUMA = 4,
	RUKA = 5,
	SAITOU = 6,
	SASAKI = 7,
	HAYASHI = 8,
	HANAIZUMI = 9
};

#define WHO KOYAMA
#define PROC_ID 2016010604
#define DEVISE 1

enum StaticFeatureType{
	Feature_ShoulderLength = 0,
	Feature_Tall = 1,
	Feature_ArmLeftLen = 2,
	Feature_ArmRightLen = 3,
	Feature_BodyLen = 4
};

enum DynamicFeatureType{
	Feature_Neck = 0,
	Feature_LeftShoulder = 1,
	Feature_RightShoulder = 2,
	Feature_LeftElbow = 3,
	Feature_RightElbow = 4,
	Feature_Hip = 5,
	Feature_LeftKnee = 6,
	Feature_RightKnee = 7,
};

const vector<vector<int>> static_feature_use_joints = {
	{ JointType_SpineShoulder, JointType_ShoulderRight, JointType_ShoulderLeft },
	{},
	{ JointType_ElbowLeft, JointType_ShoulderLeft, JointType_WristLeft },
	{ JointType_ElbowRight, JointType_ShoulderRight, JointType_WristRight },
	{ JointType_SpineMid, JointType_SpineShoulder, JointType_SpineBase }
};

const vector<vector<int>> dynamic_feature_use_joints = {
	{ JointType_Neck, JointType_Head, JointType_SpineShoulder},
	{ JointType_ElbowLeft, JointType_ShoulderLeft, JointType_WristLeft },
	{ JointType_ElbowRight, JointType_ShoulderRight, JointType_WristRight },
	{ JointType_KneeLeft, JointType_HipLeft, JointType_KneeRight, JointType_HipRight },
	{ JointType_KneeLeft, JointType_HipLeft, JointType_AnkleLeft },
	{ JointType_KneeRight, JointType_HipRight, JointType_AnkleRight }
};

class WalkThroughId
{
private:
	vector<vector<float>> dynamic_features;

public:
	//Constructor
	WalkThroughId();

	//Destructor
	~WalkThroughId();

	void initialize();

	void finalize();

	void execute(array<Joint, JointType::JointType_Count> joints, bool isValidData);

	void acquire_positions(array<Joint, JointType::JointType_Count> joints);

	float calc_features();

	void insert_features(array<Joint, JointType::JointType_Count> joints);

	void insert_dynamic_features(array<Joint, JointType::JointType_Count> joints);

	void insert_static_features(array<Joint, JointType::JointType_Count> joints);

	void insert_empty_features();

	float calc_tall();

	float calc_shoulderLen();

	float calc_armLeftLen();

	float calc_armRightLen();

	float calc_bodyLen();

	float calc_length();

	float evaluate_angle(Point3f c, Point3f a, Point3f b);

	float evaluate_seperated_angle(Point3f pA, Point3f pB, Point3f pC, Point3f pD);

	float evaluate_dist(Point3f a, Point3f b);
};