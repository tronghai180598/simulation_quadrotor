#include "chiakhoa.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

//(float mTm, float mTv, float mTf, float Kf, float Kp, float Kd) 
int main() {
  Quadrotor quad(0.001f);
  quad.state = std::vector<float>(12, 0.0f); // Initialize state vector with 12 zeros
  quad.Forces = std::vector<float>(4, 0.0f); // Initialize Forces vector with 4 zeros
  
  // Hovering thrust plus extra 20% to make it rise
  float hovering_thrust = 9.81f * quad.m;
  std::vector<float> control_inputs = {hovering_thrust * 1.2f, 0.0f, 0.0f, 0.0f};
    
  for (int i = 0; i < 1000; ++i) {
      quad.state = quad.step(0.001f, control_inputs);
      // Print the z position (state[11]) and z velocity (state[8])
      if (i % 100 == 0) {
          printf("Time: %.3f s, Z Position: %.6f m, Z Velocity: %.6f m/s, Forces[0]: %.6f N\n", 
                 quad.time_elapse, quad.state[11], quad.state[8], quad.Forces[0 ]);
      }
  }
  return 0;
   
}
