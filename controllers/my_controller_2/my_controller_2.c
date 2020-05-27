/*
 * File:          my_controller_2.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
 
#include <webots/robot.h>
#include <webots/motor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 100
#define PI 3.14

const int JOINTS = 3;

/* Measurements */
const double MODULE_SIZE = 0.2;
const double GAP_SIZE = 0.01;
double WHOLE_LENGTH;

/**
 * http://academic.uprm.edu/pcaceres/Undergrad/Snake/id3.htm
 *
 * t - time in seconds.
 * i - i-th joint.
 * n - number of segments.
 * f - frequenty, in hertz
 * a - winding angle, in radians.
 * b - number of periods in a length.
*/
double angular_velocity(double t, double i, double n, double f, double a, double b) {
  double temp = sin(b / (2 * n));
  if (temp < 0) temp = - temp;
  
  double alpha = a * temp;
  
  double beta = b / n;
  double result = alpha * sin(2 * PI * f + (i - i) * beta + t) * (180 / PI);
  return result;
}

double iterate_velocity(double t, int i) {
  double f = 1;
  double a = 1;
  double b = 1;
  double speed = angular_velocity(t, i, JOINTS+1, f, a, b);
  return speed;
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // Initialize and get all motors
  WbDeviceTag motor[JOINTS];
  motor[0] = wb_robot_get_device("rotational motor 1");
  motor[1] = wb_robot_get_device("rotational motor 2");
  motor[2] = wb_robot_get_device("rotational motor 3");

  // Calculate length of the robot
  WHOLE_LENGTH = MODULE_SIZE * (JOINTS + 1) + GAP_SIZE * JOINTS;
  printf("t\t\t0\t1\t2\n");
  double t = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    printf("%f\t", t);
    for (int i = 0; i < JOINTS; i++) {
      // calculate joint angular velocity and apply it to motor
      double speed = iterate_velocity(t, i);
      //wb_motor_set_velocity(motor[i], speed);
      wb_motor_set_velocity(motor[i], 10);
      printf("%f\t", speed);
    }
    printf("\n");
    
    // increment time
    t += ((double) TIME_STEP) / 1000;
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
