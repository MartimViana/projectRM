/*
 * File:          worm_1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/motor.h>

/* C libraries*/
#include <stdio.h>
#include <math.h>

#define TIME_STEP 1000

/**
 * General robot parameters
*/
const double JOINT_LENGTH = 0.01;
const double BODY_LENGTH = 0.1;
const double TOTAL_LENGTH = 0.21;

/**
 * Connections between cylinders
*/
const int CONNECTION_AMT = 2;
/**
 * Acceleration
*/
const double ACCELERATION = 0.000625;
const double SIMULATION_TIME = 10;
double time;
/**
 * Calculates angular velocity of serpentine moviment.
 * l - Whole length of the robot body.
 * kn - number of wave shapes.
 * s - Body length along body curve.
 * alpha - Initial winding angle of the curve.
 * i - Part of robot whose result belongs to.
*/
const double serpentinoid_curve(double l, int kn, double s, double alpha) {
  return (-2*kn*M_PI*alpha/l)*sin(2*kn*M_PI*s/l);
}


const double iterate_acc(double t, const double T, const double acc) {
  if (0 <= t && t < T/20) return acc;
  else if (T <= t && t < 9*T/10) return 0;
  else if (9*T/20 <= t && t < T) return -acc;
  return 0;
}
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

   /* Setup motor variable */
   WbDeviceTag motor[CONNECTION_AMT];
   motor[0] = wb_robot_get_device("rotational motor 1");
   motor[1] = wb_robot_get_device("rotational motor 2");

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors
     */

    /* Process sensor data here */
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
     for(int i = 0; i < CONNECTION_AMT; i++) {
       wb_motor_set_position(motor[i], INFINITY);
       
       // get current velocity
       double velocity = wb_motor_get_velocity(motor[i]);
       printf("v(%d): %f\n", i, velocity);
       
       // apply acceleration to velocity
       velocity += iterate_acc(time, SIMULATION_TIME, ACCELERATION);
       
       // set velocity in motor
       wb_motor_set_velocity(motor[i], velocity); 
     }
     time += 1;
     
     // loop moviment
     if (time >= SIMULATION_TIME) time = 0;
     
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
