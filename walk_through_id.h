#include <opencv2/opencv.hpp>

#include <vector>
#include <array>
#include <Kinect.h>

using namespace std;
using namespace cv;

#define ID_COUNT 9
#define DYNAMIC_FEATURE_COUNT 6
#define STATIC_FEATURE_COUNT 5
#define FEATURE_SIZE DYNAMIC_FEATURE_COUNT*STATIC_FEATURE_COUNT

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
	Feature_LeftElbow = 1,
	Feature_RightElbow = 2,
	Feature_Hip = 3,
	Feature_LeftKnee = 4,
	Feature_RightKnee = 5,
};

const vector<vector<int>> static_feature_use_joints = {
	{ JointType_SpineShoulder, JointType_ShoulderRight, JointType_ShoulderLeft },
	{},
	{ JointType_ElbowLeft, JointType_ShoulderLeft, JointType_WristLeft },
	{ JointType_ElbowRight, JointType_ShoulderRight, JointType_WristRight },
	{ JointType_SpineMid, JointType_SpineShoulder, JointType_SpineBase }
};

const vector<vector<int>> dynamic_feature_use_joints = {
	{ JointType_Neck, JointType_Head, JointType_SpineShoulder },
	{ JointType_ElbowLeft, JointType_ShoulderLeft, JointType_WristLeft },
	{ JointType_ElbowRight, JointType_ShoulderRight, JointType_WristRight },
	{ JointType_KneeLeft, JointType_HipLeft, JointType_KneeRight, JointType_HipRight },
	{ JointType_KneeLeft, JointType_HipLeft, JointType_AnkleLeft },
	{ JointType_KneeRight, JointType_HipRight, JointType_AnkleRight }
};

const int walking_judge_features[] = {
	Feature_Hip,
	Feature_LeftKnee,
	Feature_RightKnee
};

class WalkThroughId
{
private:
  vector<vector<float>> dynamic_features;
  int frame_count;
  int start_frame;
  int end_frame;

  vector<float> walking_judge_input;
  vector<float> walking_judge_temp;
  float temp_mean;

  Mat spectrums_input;

  CvSVM svm;
public:
  //Constructor
  WalkThroughId();

  //Destructor
  ~WalkThroughId();

  void initialize();

  void finalize();

  void execute(array<Joint, JointType::JointType_Count>& joints, bool isValidData);

  void create_classifier();

  void execute_classification();

  bool calc_spectrums();
  
  void shift_count();

  bool walking_judge();

  void import_walk_judge_temp();

  void insert_features(array<Joint, JointType::JointType_Count>& joints);

  void insert_dynamic_features(array<Joint, JointType::JointType_Count>& joints);

  void insert_static_features(array<Joint, JointType::JointType_Count>& joints);

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
