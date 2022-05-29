/*
 * Input.h
 *
 *  Created on: May 28, 2022
 *      Author: jeff
 */

#ifndef INPUT_H_
#define INPUT_H_
#include "Arduino.h"

void check_az_speed_pot();
void check_az_preset_potentiometer();
void initialize_rotary_encoders();
void check_preset_encoders();
void check_hco_buttons();
void check_buttons();
#endif /* INPUT_H_ */