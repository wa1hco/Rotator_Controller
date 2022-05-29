//
// utilities_local.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef UTILITIES_LOCAL_H_
#define UTILITIES_LOCAL_H_

#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"

#include "global_variables.h"

void print_help();
void read_azimuth();
void check_for_dirty_configuration();
void brake_release(byte, boolean);
void check_brake_release();

#endif /* UTILITIES_LOCAL_H_ */