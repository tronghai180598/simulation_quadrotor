#include "controller.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>

//(float mTm, float mTv, float mTf, float Kf, float Kp, float Kd) 
int main() {
  float dt = 0.001f;
  Quadrotor quadrotor;
  Controller controller(dt);
  std::vector<float> control_inputs = std::vector<float>(4, 0.0f);
  
  // Initialize controller and quadrotor state
  controller.state = std::vector<float>(12, 0.0f);
  quadrotor.state = std::vector<float>(12, 0.0f);
  quadrotor.Forces = std::vector<float>(4, 0.0f);
  
  float desired_x = 1.0, desired_y = 2.0, desired_z = 3.0;

  controller.x_des = desired_x;
  controller.y_des = desired_y;
  controller.z_des = desired_z;
  char sstr[32] = "asd.log";
  FILE * fw = fopen(sstr, "w+");
   int ticks = int(10.0 / dt);//  ticks = int(5.0 / dt);
  
  for (int i = 0; i < ticks; ++i) {
    controller.update(); 
    control_inputs = {controller.F_total, controller.tau_phi, controller.tau_theta, controller.tau_psi};
    std::vector<float> state = quadrotor.step(dt, control_inputs);
    controller.state = state;
    fprintf( fw, "%f %f %f %f\n", dt*i, state[9], state[10], state[11] );
  }
  fclose(fw);
  FILE *gp = popen("gnuplot -persist","w"); // gp - дескриптор канала
   if (gp == NULL)
    {
        printf("Error opening pipe to GNU plot.\n");
        return 0;
    }
    if(1){
        fprintf(gp, "\
      datafile = \"asd.log\"\n\
      set terminal png font arial 20 size 1600,800\n\
      set output \"aT1.png\"\n\
      set grid x y\n\
      plot \
      datafile using 1:2 title \"phi\" w l lw 2 lc rgb \"#e08080\",\
      datafile using 1:3 title \"theta\" w l lw 2 lc rgb \"#e08080\",\
      datafile using 1:4 title \"psi\" w l lw 2 lc rgb \"#808080\"\n\
    \
      set terminal svg size 630,300 font 'arial,12'\n\
      set output \"aT1.svg\"\n\
      set grid x y\n\
      plot \
      datafile using 1:2 title \"phi\" w l lw 2 lc rgb \"#e08080\",\
      datafile using 1:3 title \"theta\" w l lw 2 lc rgb \"#e08080\",\
      datafile using 1:4 title \"psi\" w l lw 2 lc rgb \"#808080\"\n\
\
"); //
    }
    pclose(gp);
  return 0;
   
}
