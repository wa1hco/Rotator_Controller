/*
 * StateMachine.h
 *
 *  Created on: May 28, 2022
 *      Author: jeff
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_
#include "Arduino.h"

void service_rotation_azimuth();
void service_request_queue();
void rotator(byte rotation_action, byte rotation_type);
byte current_az_state();

#endif /* STATEMACHINE_H_ */