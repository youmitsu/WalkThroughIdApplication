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
    vector<vector<float>> features;
    for(int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
        vector<float> v;
		features.push_back(v);
    }
	dynamic_features = features;
}

void WalkThroughId::finalize(){

}

void WalkThroughId::execute(array<Joint, JointType::JointType_Count> joints, bool isValidData){
  if(isValidData){
    insert_features(joints);
  }else{
    insert_empty_features();
  }
}

void WalkThroughId::insert_features(array<Joint, JointType::JointType_Count> joints){
  insert_dynamic_features(joints);
 // insert_static_features(joints);
}

void WalkThroughId::insert_dynamic_features(array<Joint, JointType::JointType_Count> joints){
  for(int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
    vector<int> use_joints = dynamic_feature_use_joints[i];
	float angle;
    if(use_joints.size() == 3){
      Joint p = joints[use_joints[0]];
	  Point3f p1(p.Position.X, p.Position.Y, p.Position.Z);
      p = joints[use_joints[1]];
	  Point3f p2(p.Position.X, p.Position.Y, p.Position.Z);
      p = joints[use_joints[2]];
	  Point3f p3(p.Position.X, p.Position.Y, p.Position.Z);
	  angle = evaluate_angle(p1, p2, p3);
    }else if(use_joints.size() == 4){
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

void WalkThroughId::insert_static_features(array<Joint, JointType::JointType_Count> joints){

}

void WalkThroughId::insert_empty_features(){

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
  int ax_cx = c.x - a.x;
	int ay_cy = c.y - a.y;
	int az_cz = c.z - a.z;
	int bx_cx = c.x - b.x;
	int by_cy = c.y - b.y;
	int bz_cz = c.z - b.z;
	float cos = ((ax_cx*bx_cx) + (ay_cy*by_cy) + (az_cz*bz_cz)) / ((sqrt((ax_cx*ax_cx) + (ay_cy*ay_cy) + (az_cz*az_cz))*sqrt((bx_cx*bx_cx) + (by_cy*by_cy) + (bz_cz*bz_cz))));
	float angle = acosf(cos);
	if (cos > -1.0 && cos < 0.0){
		angle = M_PI - angle;
	}
	return angle;
}

float WalkThroughId::evaluate_seperated_angle(Point3f pA, Point3f pB, Point3f pC, Point3f pD){
  int ax_bx = pA.x - pB.x;
	int ay_by = pA.y - pB.y;
	int az_bz = pA.z - pB.z;
	int cx_dx = pC.x - pD.x;
	int cy_dy = pC.y - pD.y;
	int cz_dz = pC.z - pD.z;
	float cos = ((ax_bx*cx_dx) + (ay_by*cy_dy) + (az_bz*cz_dz)) / ((sqrt((ax_bx*ax_bx) + (ay_by*ay_by) + (az_bz*az_bz))*sqrt((cx_dx*cx_dx) + (cy_dy*cy_dy) + (cz_dz*cz_dz))));
	float angle = acosf(cos);
	if (cos > -1.0 && cos < 0.0){
		angle =  - angle;
	}
	return angle;
}

float WalkThroughId::evaluate_dist(Point3f a, Point3f b){
  float d = sqrtf((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
  return d;
}
