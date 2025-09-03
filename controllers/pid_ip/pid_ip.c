// include h files to run simulation
#include <math.h>
#include <stdlib.h>
#include <stdio.h> 
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <time.h>

// macros
#define MAX_FORCE 40
#define MAX_POSITION 50
//#define CENTERING_ANGLE 0.7
//#define CENTERING_FORCE 4
#define TIME_STEP 32

static WbDeviceTag horizontal_motor, horizontal_position_sensor, hip;
// sensor variables
double prev_angle_position, prev_horizontal_position;
double angle_position, horizontal_position;
// controller varibles
double angle_ref, horizontal_pos_ref;
double prev_angle_error, angle_error;
double prev_horizontal_pos_error, horizontal_pos_error;
double angle_differential, angle_integral_sum;
double horizontal_differential, horizontal_integral_sum;
// controller output, which is the system input
double power;

// timer for disturbances and setpoints variations
double time1, timedif;

static void initialize() {
  wb_robot_init();
  // device instances
  horizontal_motor = wb_robot_get_device("horizontal_motor");
  horizontal_position_sensor = wb_robot_get_device("horizontal position sensor");
  hip = wb_robot_get_device("hip");
  // enable sensors
  wb_position_sensor_enable(horizontal_position_sensor, TIME_STEP);
  wb_position_sensor_enable(hip, TIME_STEP);
  
  // set points
  angle_ref = 0.0;
  horizontal_pos_ref = 0.1;
  // initial positions
  prev_angle_position = wb_position_sensor_get_value(hip);
  prev_horizontal_position = wb_position_sensor_get_value(
                              horizontal_position_sensor
                              );
  // initial errors
  // prev_angle_error = angle_ref - prev_angle_position;
  // prev_horizontal_pos_error = horizontal_pos_ref - prev_horizontal_position;
  
  // initial pid terms
  angle_differential = 0.0;
  angle_integral_sum = 0.0;
  horizontal_differential = 0.0;
  horizontal_integral_sum = 0.0;
  
  wb_motor_set_force(horizontal_motor, 0);
}

static void run() {
  // getting sensor values
  angle_position = wb_position_sensor_get_value(hip);
  horizontal_position = wb_position_sensor_get_value(
                      horizontal_position_sensor);
  printf("%f,", horizontal_pos_ref);
  printf("%f,", horizontal_position);
  printf("%f,", angle_position);
  // compute current errors
  angle_error = angle_ref - angle_position;
  horizontal_pos_error = horizontal_pos_ref - horizontal_position;
  //printf("position error \n");
  //printf("%f \n", horizontal_pos_error);
  //printf("angle error \n");
  //printf("%f \n", angle_error);

  // update pid terms
  angle_integral_sum = angle_error + angle_integral_sum;
  horizontal_integral_sum = horizontal_pos_error + horizontal_integral_sum;
  angle_differential = angle_error - prev_angle_error;
  horizontal_differential = horizontal_pos_error - prev_horizontal_pos_error;
  
  // pid controllers
  double pid_angle =
    - 70.0*angle_error
    - 1*angle_integral_sum
    - 250*angle_differential;
  //double pid_horizontal_pos =
  //  - 9*horizontal_pos_error
  //  - 0.04*horizontal_integral_sum
   // - 110*horizontal_differential;
  double pid_horizontal_pos =
    - 3.160*horizontal_pos_error
    - 0.0051*horizontal_integral_sum
    - 70.0625*horizontal_differential;
  // output pid
  power = pid_angle + pid_horizontal_pos;
  
  // force saturation
  power = power < MAX_FORCE ? power : MAX_FORCE;
  power = power > -MAX_FORCE ? power : -MAX_FORCE;
  printf("%f \n", power);
  // set motor force
  wb_motor_set_force(horizontal_motor, power);

  // update previous errors
  prev_angle_error = angle_error;
  prev_horizontal_pos_error = horizontal_pos_error;
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  initialize();

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  // initial time
  time1 = (double) clock();
  time1 = time1 / CLOCKS_PER_SEC; // seconds
  while (wb_robot_step(TIME_STEP) != -1) {
    // changing setpoints
    timedif = (((double) clock()) / CLOCKS_PER_SEC) - time1;
    if (timedif > 10) {
      horizontal_pos_ref = -0.2;
    }
    if (timedif > 30) {
      horizontal_pos_ref = 0.0;
    }
    // algorithm
    run();
    //disturbance
    if ((timedif > 20) && (timedif < 20.5)) {
      wb_motor_set_force(horizontal_motor, power + 4);
    }
  };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
