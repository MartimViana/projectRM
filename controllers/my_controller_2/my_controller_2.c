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

#define TIME_STEP 32
#define PI 3.14159265359

// number of joinst that it has
const int JOINTS = 6;

// flag that indicates whether the robot is initializing serpentine
// moviment or not.
//int IS_IN_INITIAL_LOOP = 1;

/* Measurements */
const double MODULE_SIZE = 0.2;
const double GAP_SIZE = 0.01;
double WHOLE_LENGTH;

double angular_velocity(double L, double Kn, double a, double s, double i, double n) {
  double part1 = Kn*PI;
  double part2 = part1/L;
  
  double result = -4 * a * part2;
  result = result * sin(part2);
  result = result * sin(2 * part2 * s + 2 * part1 * i / n - part1 / n);
  return result;
}

double custom_velocity(double t, int i, double precision, double amplitude, double offset) {
  double value = precision * t;
  
  double result = -amplitude * sin(value);
  return result;
}

double iterate_custom_velocity(double t, int i, double precision, double amplitude, double offset) {
  return custom_velocity(t, i, precision, amplitude, offset);
}

/**
 * a: Amplitude
 * s: Normalized arc length of the body neutral axis. Is between 0 and 1.
 * T: Period of the wave.
*/ 
double attempt_2(double a, double s, double T, double t) {
  return a * cos(2 * PI * (s + t)/ T);
}

double iterate_attempt_2(double t, double i) {
  double amplitude = 0.02;
  double wavePeriod = 1.5;
  double arcLength = (MODULE_SIZE * (i + 1) + GAP_SIZE * i) / WHOLE_LENGTH;
  double result = attempt_2(amplitude, arcLength, wavePeriod, t);
  return result;
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // Initialize and get all motors
  WbDeviceTag motor[JOINTS];
  motor[0] = wb_robot_get_device("rotational motor 1");
  motor[1] = wb_robot_get_device("rotational motor 2");
  motor[2] = wb_robot_get_device("rotational motor 3");
  motor[3] = wb_robot_get_device("rotational motor 4");
  motor[4] = wb_robot_get_device("rotational motor 5");
  motor[5] = wb_robot_get_device("rotational motor 6");
  
  // Initialize motor position
  double motorPosition[JOINTS];
  
  // Set initial variables to all motors
  for (int i = 0; i < JOINTS; i++) {
    // Set maximum position to all motors
    wb_motor_set_position(motor[i], 0);
    
    // Initialize all motor positions
    motorPosition[i] = 0;
  }
  
  
  
  // Set all initial velocities to 0
  //for (int i = 0; i < JOINTS; i++) {
  //  wb_motor_set_velocity(motor[i], 0);
  //}

  // Calculate length of the robot
  WHOLE_LENGTH = MODULE_SIZE * (JOINTS + 1) + GAP_SIZE * JOINTS;
  
  printf("t\t");
  for (int i = 0; i < JOINTS; i++) {
    printf("\t%d", i);
  }
  printf("\n");
  double t = 0;
  //double precision = 0.6;
  //double amplitude = 0.5;
  //double offset = PI / (JOINTS+1);
  
  /* Start main control loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    printf("%f\t", t);
    for (int i = 0; i < JOINTS; i++) {
      motorPosition[i] += iterate_attempt_2(t, i);
      
      //double speed = iterate_velocity(t, i);
      wb_motor_set_position(motor[i], motorPosition[i]);
      printf("%f\t", motorPosition[i]);
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
