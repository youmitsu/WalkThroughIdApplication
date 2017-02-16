#include "walk_through_id.h"

#include <thread>
#include <chrono>

#include <omp.h>

#include "stdlib.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

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
    vector<vector<Point3f>> features;
    dynamic_features = features;
    for(int i = 0; i < DYNAMIC_FEATURE_COUNT; i++){
        vector<Point3f> v;
        dynamic_features.push_back(v);
    }
}

void WalkThroughId::finalize(){

}

void WalkThroughId::acquire_positions(array<Joint, JointType::JointType_Count> joints){

}

void WalkThroughId::insert_features(){

}

float WalkThroughId::calc_features(){
	return 0;
}
