//
// serial_command_processing.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef SERIAL_COMMAND_PROCESSING_H_
#define CHECK_SERIAL_COMMAND_PROCESSING_H_

#include "rotator_features.h"
#include "rotator_pins_custom_board.h"
#include "settings.h"
#include "macros.h"

#include "global_variables.h"

void submit_request(byte axis, byte request, int parm);
void check_serial();
void get_keystroke();
void clear_command_buffer();
void report_current_azimuth();

void yaesu_serial_command();
void yaesu_f_command();
void yaesu_o_command();
void yaesu_m_command();
void yaesu_x_command();
void yaesu_w_command();
void yaesu_p_command();
void clear_serial_buffer();

void update_az_variable_outputs(byte);
void print_wrote_to_memory();

#endif /* SERIAL_COMMAND_PROCESSING_H_ */