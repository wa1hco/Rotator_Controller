//
// eeprom_local.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef EEPROM_LOCAL_H_
#define EEPROM_LOCAL_H_

#include "rotator_features.h"
#include "rotator_pins.h"
#include "settings.h"
#include "macros.h"
#include "global_variables.h"


void initialize_eeprom_with_defaults();
void write_settings_to_eeprom();

#endif /* EEPROM_LOCAL_H_ */