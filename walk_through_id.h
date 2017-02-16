#include <opencv2/opencv.hpp>

#include <vector>
#include <array>
#include <Kinect.h>

using namespace std;
using namespace cv;

#define ID_COUNT 9
#define DYNAMIC_FEATURE_COUNT 6

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

class WalkThroughId
{
private:
  vector<vector<Point3f>> dynamic_features;

public:
  //Constructor
  WalkThroughId();

  //Destructor
  ~WalkThroughId();

  void initialize();

  void finalize();

  void acquire_positions(array<Joint, JointType::JointType_Count> joints);

  float calc_features();

  void insert_features();

};
