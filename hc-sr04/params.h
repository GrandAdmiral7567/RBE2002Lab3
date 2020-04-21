#ifndef __PARAMS_H
#define __PARAMS_H

//PID gains -- you can edit the defaults later, after you tune the motors
float Kp = 5;
float Ki = 1.5;

float Kp2 = 1;
float Ki2 = 0.01;
float Kd2 = 0.1;


int bufferCountLeft[100] = {0};
int bufferIndexLeft = 0;

int bufferCountRight[100] = {0};
int bufferIndexRight = 0;

float Kd = 0;


/*
 * target wheel speeds; these are in encoder ticks per PID loop!
 * 
 * Even though ticks/loop are always reported in integer number of ticks
 * we use a float here so that the target can be fractional. In practice, 
 * the fraction will cause the error to be sometimes a fraction positive
 * and sometimes a fraction negative, but they'll all wash out in the end.
 */

float targetLeft;
float targetRight;


#endif
