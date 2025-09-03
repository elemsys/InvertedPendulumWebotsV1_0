// include h files to run simulation
#include <math.h>
#include <stdlib.h>
#include <stdio.h> 
#include <time.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

// macros
#define MAX_FORCE 40
#define MAX_POSITION 50
#define TIME_STEP 32

// Get the node by its DEF name (e.g. "MY_OBJECT") (supervisor)
WbNodeRef cart_node;
// devices
static WbDeviceTag horizontal_motor, horizontal_position_sensor, hip;
// sensor variables for space state vector
double deriv_gain = 1.00;
double angle, x;
double prev_angle, prev_x;
double dangle, dx;
double X[4];

// Reference vars and vector
double angle_ref, x_ref;
double dangle_ref, dx_ref; // derivative ss vector vars
double X_ref[4];

// Controller vars
// K of LQR
//double Kd[4] = {-0.7431, -2.1035, -54.0665, -9.4850}; // it 0 R = 1
//double Kd[4] = {-0.5884,   -1.8655,  -53.3378,   -9.3488};// prasad vals
//double Kd[4] = {-1.8402, -3.7709, -59.1707, -10.4107}; // prasad vals R = 0.1
//double Kd[4] = {-2.3204, -4.2116, -60.4336, -10.6718}; // it 0 R = 0.1
//double Kd[4] = {-6.4113, -7.6949, -70.0903, -12.4190};
double Kd[4] = {-8.3324, -9.1941, -74.7931, -13.1969}; // it5 functional
//double Kd[4] = {-10.0256, -10.4212 , -78.3010,  -13.8002};

// controller output, which is the system input u
double power;

// timer for disturbances and setpoints variations
double time1, timedif;

static double calc_power_lqr() {
  float power_lqr = 0.0;
  double error = 0.0;
  for(int i=0; i<4; i++)
  {
    error = X_ref[i] - X[i];
    power_lqr += error*Kd[i];
  }
  return power_lqr;
}

static void initialize() {
  wb_robot_init();
  // Get the node by its DEF name (e.g. "MY_OBJECT")
  cart_node = wb_supervisor_node_get_from_def("CART_SOLID");
  
  // device instances
  horizontal_motor = wb_robot_get_device("horizontal_motor");
  horizontal_position_sensor = wb_robot_get_device("horizontal position sensor");
  hip = wb_robot_get_device("hip");
  // enable sensors
  wb_position_sensor_enable(horizontal_position_sensor, TIME_STEP);
  wb_position_sensor_enable(hip, TIME_STEP);
  
  // set points
  angle_ref = 0, x_ref = 0.1;
  dangle_ref = 0, dx_ref = 0;
  X_ref[0] = x_ref, X_ref[1] = dx_ref;
  X_ref[2] = angle_ref, X_ref[3] = dangle_ref;
  
  // initial angle and position
  prev_angle = 0;
  prev_x = 0;
  // read sensors
  x =  wb_position_sensor_get_value(horizontal_position_sensor);
  dx = deriv_gain*(x - prev_x)/0.032; // Ts = 0.032 from TIME_STEP 32ms
  angle = wb_position_sensor_get_value(hip);
  dangle = deriv_gain*(angle - prev_angle)/0.032;
  
  // define ss vector
  X[0] = x, X[1] = dx;
  X[2] = angle, X[3] = dangle;
  
  // initial power
  power = 0;
  power = calc_power_lqr();
  
  wb_motor_set_force(horizontal_motor, 0);
}

static void run() {
  // read sensors
  x =  wb_position_sensor_get_value(horizontal_position_sensor);
  angle = wb_position_sensor_get_value(hip);
  printf("%f,", X_ref[0]);
  printf("%f,", x);
  printf("%f,", angle);
  
  dx = deriv_gain*(x - prev_x)/0.032; // Ts = 0.032 from TIME_STEP 32ms
  dangle = deriv_gain*(angle - prev_angle)/0.032;
  
  // update ss vector
  X[0] = x, X[1] = dx;
  X[2] = angle, X[3] = dangle;

  // compute motor power
  power = calc_power_lqr();
  
  // force saturation
  power = power < MAX_FORCE ? power : MAX_FORCE;
  power = power > -MAX_FORCE ? power : -MAX_FORCE;
  printf("%f\n", power);
  // set motor force
  wb_motor_set_force(horizontal_motor, power);
  
  // update previous states
  prev_x = x;
  prev_angle = angle;
  
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
  // bool force_applied = false; // check if force was applied

  while (wb_robot_step(TIME_STEP) != -1) {
    // changing setpoints
    timedif = (((double) clock()) / CLOCKS_PER_SEC) - time1;
    if (timedif > 10) {
      X_ref[0] = -0.2;
      //const double force[3] = {10.0, 0.0, 0.0};  // x-direction force
      //wb_supervisor_node_add_force(cart_node, force, false);  // false = world coords
      
      // Define the impulse vector
      //const double force[3] = {5.0, 0.0, 0.0};
      
      // Define the offset from the object's center of mass (e.g., at the origin)
      //const double offset[3] = {0.0, 0.0, 0.0};
      
      // Apply the force for a single time step
      // wb_supervisor_node_add_force_with_offset(cart_node, force, offset, false); // false = world coords
      // force_applied = true;
    }
    // Remove the force in the next step to simulate a quick impulse
    /*
    if (force_applied) {
       const double zero_force[3] = {-5.0, 0.0, 0.0};
       wb_supervisor_node_add_force_with_offset(cart_node, zero_force, zero_force, false);
    }
    */
    if (timedif > 30) {
      X_ref[0] = 0.0;
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