/*
 * Display_MAX7221.h
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */

#ifndef DISPLAY_MAX7221_H_
#define DISPLAY_MAX7221_H_

#include <Arduino.h>
#include <avr/pgmspace.h>

//-----------------------------------Display_MAX7221 private functions--------------------------
void SPI_Transfer(uint8_t address, uint8_t data);
void initialize_MAX7221_display();
void update_Az_MAX7221_display();

//..................................Display MAX7221 public functions
void display_az_preset_MAX7221(int target_azimuth);

//-----------------------------------Display_MAX7221 private variables--------------------------

#endif /* DISPLAY_MAX7221_H_ */
