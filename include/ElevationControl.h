/*
 * ElevationControl.h
 *
 *  Created on: May 28, 2022
 *      Author: jeff
 */

#ifndef ELEVATIONCONTROL_H_
#define ELEVATIONCONTROL_H_
#include "Arduino.h"

byte current_el_state();
void el_check_operation_timeout();
void yaesu_w_command ();
void read_elevation();
void report_current_elevation();
void update_el_variable_outputs(byte speed_voltage);
void el_position_pulse_interrupt_handler();
float correct_elevation(float elevation_in);
void service_rotation_elevation();

#endif /* ELEVATIONCONTROL_H_ */