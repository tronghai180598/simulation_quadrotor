#pragma once 
#include "Quadrotor.h"
#include <string>
#include <math.h>
// float F;
// float V;
float my_rand(float min, float max){ return ( rand() * (max - min)) / RAND_MAX + min; }
class IMU{
private: 
    Quadrotor *model_;
public: 
    IMU (Quadrotor& model): model_(&model){};
    float Fi_last = 0.0f;  
    float Vi_last = 0.0f;
  //  void readimu(float& Fi_out, float& Vi_out){
        // Fi_last = model_ -> getFi();
        // Vi_last = model_ -> getVi();
        // Fi_out = Fi_last + my_rand(-0, 0);
        // Vi_out = Vi_last + my_rand(-0, 0);
    // }
};


