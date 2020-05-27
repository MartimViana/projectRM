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

#define TIME_STEP 100
#define PI 3.14

/**
 * Connections between cylinders
*/
const int CONNECTION_AMT = 3;

/**
 * Acceleration
*/
const double ACCELERATION = 1;
const double LOOP_DURATION = 20;
double time;
int INITIAL_LOOP = 1;

const double UNIT_LENGTH = 0.2;
const double GAP_LENGTH = 0.01;
double WHOLE_LENGTH;

double get_simple_acceleration(double t, double loopDuration, double acc) {
  if (INITIAL_LOOP == 0) { 
    if (t < loopDuration/2) return acc;
    if (t > loopDuration/2) return -acc;
  }
  else {
    if (t < loopDuration/2) return acc/2;
    if (t > loopDuration/2) {
      INITIAL_LOOP = 0;
      return get_simple_acceleration(t, loopDuration, acc);
    }
  }
  return 0;
}

double get_acceleration_paper(double t, double T, double acc) {
  if (0 <= t && t < T/10) return acc;
  if (T/10 <= t && t < 9*T/10) return 0;
  if (9*T/10<=t && t < T) return -acc;
  return 0;
}


/* SERPENTINE CURVE */
/**
 * L - Whole length of snake body.
 * Kn - Numberr of wave shapes.
 * a - Initial winding angle of the curve.
 * s - Body length along the body curve.
*/
double serpentine_curve(double L, double Kn, double a, double s) {
  return (-2*Kn*PI*a)/L * sin((2*Kn*PI*s)/L);
}

void iterate_using_serpentine_curve(WbDeviceTag motor, double t, double i) {
 double L = WHOLE_LENGTH;
 double Kn = CONNECTION_AMT + 1;
 
 // more used as a 'scale' constant. Very useful for scalling up and down the acceleration.
 double a = 0.2;
 
 // the main iterator, responsible for making time enter in the literal equation.
 double s = CONNECTION_AMT*(i+1) + GAP_LENGTH*i + t;
 double speed = serpentine_curve(L, Kn, a, s);
 wb_motor_set_velocity(motor, speed);
}


/* BASED ON THE PAPER "DESIGN AND MODELING OF A SNAKE ROBOT BASED ON WORM-LIKE LOCOMOTION "*/
/* ANGULAR VELOCITY */
double angular_velocity(double L, double Kn, double a, double s, double i, double n) {
  double part1 = Kn*PI;
  double part2 = part1/L;
  
  double result = -4 * a * part2;
  result = result * sin(part2);
  result = result * sin(2 * part2 * s + 2 * part1 * i / n - part1 / n);
  return result;
}

void iterate_using_angular_velocity(WbDeviceTag motor, double t, double i) {
 double L = WHOLE_LENGTH;
 double Kn = CONNECTION_AMT + 1;
 
 // more used as a 'scale' constant. Very useful for scalling up and down the acceleration.
 double a = 0.2;
 
 // the main iterator, responsible for making time enter in the literal equation.
 double s = CONNECTION_AMT*(i+1) + GAP_LENGTH*i;
 double n = CONNECTION_AMT;
 double speed = angular_velocity(L, Kn, a, s, i, n);
 wb_motor_set_velocity(motor, speed);
}

/* RELATIVE ANGLE */
double relative_angle(double L, double Kn, double a, double s, double i, double n) {
  double part1 = Kn*PI;
  double part2 = part1/L;
  
  return -2 * a * sin(part2) * sin(2 * part2 * s + 2 * part1 * i / n - Kn * PI / n);
}

void iterate_using_relative_angle(WbDeviceTag motor, double t, double i) {
  double L = WHOLE_LENGTH;
  double Kn = CONNECTION_AMT + 1;
  double a = 0.01;
  double s = CONNECTION_AMT*(i+1) + GAP_LENGTH*i;
  double n = CONNECTION_AMT;
  double acceleration = relative_angle(L, Kn, a, s, i, n);
  wb_motor_set_acceleration(motor, acceleration);
}


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  WbDeviceTag motor[CONNECTION_AMT];
  motor[0] = wb_robot_get_device("rotational motor 1");
  motor[1] = wb_robot_get_device("rotational motor 2");
  motor[2] = wb_robot_get_device("rotational motor 3");
  
  for (int i = 0; i < CONNECTION_AMT; i++) {
    wb_motor_set_position(motor[i], INFINITY);
    wb_motor_set_velocity(motor[i], 0.0);
  }
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  double t = 0;
  WHOLE_LENGTH = UNIT_LENGTH * (CONNECTION_AMT+1) + GAP_LENGTH * CONNECTION_AMT;
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors
     */
     
     
    /* Process sensor data here */
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     for (int i = 0; i < CONNECTION_AMT; i++) {
       // Execute iteration function
       //iterate_using_serpentine_curve(motor[i], t, i);
       //printf("speed(%f) = %f\n", t, wb_motor_get_velocity(motor[i]));
     }
     //if (t >= LOOP_DURATION) t = 0;
     t++;
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
