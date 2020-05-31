/*
 * File:          my_controller_2.c
 * Date: 24 May 2020
 * Description:
 * Author: João Pinto & Martim Viana
 * Modifications:
 */
 
// Webots library
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

// Default C libraries
#include <stdio.h>
#include <stdlib.h>
////////////////////////////////////////////////////////////////////////
// DEFINITIONS
#define TIME_STEP 32
#define PI 3.14159265359

// Number of joints that the robot uses
#define JOINTS 6

////////////////////////////////////////////////////////////////////////
// VARIABLES

// flag that indicates whether the robot is initializing serpentine
// moviment or not.
//int IS_IN_INITIAL_LOOP = 1;

/* Measurements */
const double MODULE_SIZE = 0.2;
const double GAP_SIZE = 0.01;
double WHOLE_LENGTH;

/* Sensors */
WbDeviceTag sensor;
double MAX_REACH;
double FOV;

// Point that robot want's to follow.
double destination;

// Amount of miliseconds the robot takes to refresh current values. The robot refreshes it's values only when
// it's head is perpendicular to the ground. In this case, when the moviment refreshes.
const int SAMPLING_PERIOD = 1000;

/* Actuators */
WbDeviceTag motor[JOINTS];

// Contains the newest values recieved by the radar. It's values are in meters.
const float *rangeImage;

// Maximum amplitude that frontal moviment is allowed to have.
const double MAX_AMPLITUDE = 1 / (1.2 * (double) JOINTS);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * a: Amplitude
 * s: Normalized arc length of the body neutral axis. Is between 0 and 1.
 * T: Period of the wave.
*/ 
double frontal_speed(double a, double s, double T, double t) {return a * cos(2 * PI * (s + t)/ T);}

double iterate_frontal_speed(double t, double i, double amplitude) {
  double wavePeriod = 1;
  double arcLength = (MODULE_SIZE * (i + 1) + GAP_SIZE * i) / WHOLE_LENGTH;
  double result = frontal_speed(amplitude, arcLength, wavePeriod, t);
  return result;
}

/**
 *
*/
void find_destination() {
  double v = wb_distance_sensor_get_value(sensor);
  if (v == v) {
    destination = wb_distance_sensor_get_value(sensor);
  }
  
}

/**
 *
*/
void read_sensor(WbDeviceTag sensor) {
  // Analyze image to find most interesting spot to explore and assign it.
  find_destination();
}

double calculate_amplitude(double distance) {
  printf("%f %f %f\n", distance, MAX_REACH, MAX_AMPLITUDE);
  return (distance / MAX_REACH) * MAX_AMPLITUDE;
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // Initialize and get all motors
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
  
  // Initialize sensors
  sensor = wb_robot_get_device("sensor");
  wb_distance_sensor_enable(sensor, SAMPLING_PERIOD);
  
  MAX_REACH = wb_distance_sensor_get_max_value(sensor);
  printf("%f %f\n", FOV, MAX_REACH);

  // Calculate length of the robot
  WHOLE_LENGTH = MODULE_SIZE * (JOINTS + 1) + GAP_SIZE * JOINTS;
  
  double t = 0;
  
  /* Start main control loop */
  while (wb_robot_step(TIME_STEP) != -1) {
    
    // Read sensors
    read_sensor(sensor);
    
    // Calculate required amplitude
    double amplitude = calculate_amplitude(destination);
    //double amplitude = MAX_AMPLITUDE;
    
    for (int i = 0; i < JOINTS; i++) {
    
      // set forward speed
      motorPosition[i] += iterate_frontal_speed(t, i, amplitude);
      
      // set positions
      wb_motor_set_position(motor[i], motorPosition[i]);
      
    }
    printf("%f\n", amplitude);
    
    // increment time according to TIME_STEP
    t += ((double) TIME_STEP) / 1000;
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
