/*
 * PID.cpp
 *
 *  Created on: Jun 16, 2017
 *      Author: jeff
 */

#include "Arduino.h"
#include <math.h>
#include <PID_v1.h>

#include "PID.h"

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

// code for a DC motor with PID controller
// L298n has IN1, IN2, PWM pins, the two IN pins control H bridge totem poles
void initialize_PID()
{
	PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
}
