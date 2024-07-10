//
// eeprom_local.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef EEPROM_LOCAL_H_
#define EEPROM_LOCAL_H_

#include "dependencies.h"

void initialize_eeprom_with_defaults();
void write_settings_to_eeprom();
void read_settings_from_eeprom();

#endif /* EEPROM_LOCAL_H_ */