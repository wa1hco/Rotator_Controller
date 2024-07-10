/*
 * Dependencies.h
 *
 *  Created on: Feb 12, 2017
 *      Author: jeff
 */

#ifndef DEPENDENCIES_H_
#define DEPENDENCIES_H_

#include "WProgram.h"

#include "rotator_features.h"
#include "settings.h"
#include "rotator_pins_HCO_board.h"
#include "StateMachine.h"
#include "serial_command_processing.h"
#include "eeprom_local.h"
#include "Input.h"
#include "utilities_local.h"
#include "global_variables.h"

#define CODE_VERSION "2024070901"

/* ---------------------- dependency checking - don't touch this unless you know what you are doing ---------------------*/
// added Teensy 3.2, 3.1 as M20DX256
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__) || defined(__MK20DX256__)
#define OPTION_SERIAL1_SUPPORT
#endif

// added Teensy 3.2, 3.1 as M20DX256
#if defined(__AVR_ATmega2560__) || defined(__MK20DX256__)
#define OPTION_SERIAL2_SUPPORT
#define OPTION_SERIAL3_SUPPORT
#endif

// TODO, something funky with this, FEATURE_HOST_REMOTE_PROTOCOL doesn't work in Eclipse Arduino
#if (defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) || defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT)) && !defined(FEATURE_HOST_REMOTE_PROTOCOL)
#define FEATURE_HOST_REMOTE_PROTOCOL
#endif

#if !defined(FEATURE_REMOTE_UNIT_SLAVE) && !defined(FEATURE_YAESU_EMULATION) && !defined(FEATURE_EASYCOM_EMULATION)
#error "You need to activate FEATURE_YAESU_EMULATION or FEATURE_YAESU_EMULATION or make this unit a remote by activating FEATURE_REMOTE_UNIT_SLAVE"
#endif

#if defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT) && !defined(OPTION_SERIAL1_SUPPORT)
#error "You need hardware with Serial1 port for remote unit communications"
#undef FEATURE_HOST_REMOTE_PROTOCOL
#undef FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
#endif

#if defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) && !defined(OPTION_SERIAL1_SUPPORT)
#error "You need hardware with Serial1 port for remote unit communications"
#undef FEATURE_HOST_REMOTE_PROTOCOL
#undef FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT
#endif

#if defined(FEATURE_EL_POSITION_PULSE_INPUT) && !defined(FEATURE_ELEVATION_CONTROL)
#undef FEATURE_EL_POSITION_PULSE_INPUT
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT)
#error "You must turn off FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT if using FEATURE_REMOTE_UNIT_SLAVE"
#undef FEATURE_HOST_REMOTE_PROTOCOL
#undef FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
#endif //FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT)
#error "You must turn off FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT if using FEATURE_REMOTE_UNIT_SLAVE"
#undef FEATURE_HOST_REMOTE_PROTOCOL
#undef FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT
#endif

#if defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT) && (defined(FEATURE_AZ_POSITION_POTENTIOMETER)|| defined(FEATURE_AZ_POSITION_ROTARY_ENCODER)||defined(FEATURE_AZ_POSITION_PULSE_INPUT)||defined(FEATURE_AZ_POSITION_HMC5883L))
#error "You cannot get AZ position from remote unit and have a local azimuth sensor defined"
#endif

#if defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) && (defined(FEATURE_EL_POSITION_POTENTIOMETER)||defined(FEATURE_EL_POSITION_ROTARY_ENCODER)||defined(FEATURE_EL_POSITION_PULSE_INPUT)||defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB)||defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB))
#error "You cannot get EL position from remote unit and have a local elevation sensor defined"
#endif

#if (defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB) && defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB))
#error "You must select only one one library for the ADXL345"
#endif

#if defined(FEATURE_I2C_LCD) && !defined(FEATURE_WIRE_SUPPORT)
#define FEATURE_WIRE_SUPPORT
#endif

#if (defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB)||defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB)||defined(FEATURE_EL_POSITION_LSM303)||defined(FEATURE_AZ_POSITION_LSM303)) && !defined(FEATURE_WIRE_SUPPORT)
#define FEATURE_WIRE_SUPPORT
#endif

#if defined FEATURE_HCO_BOARD
  //#define FEATURE_WIRE_SUPPORT
  #define FEATURE_MAX7221_DISPLAY
  #define FEATURE_HCO_BUTTONS
  #define FEATURE_HCO_AZ_POSITION         // read + and - ends of pot with grounded wiper
  #define FEATURE_HCO_ADC
  #define FEATURE_FIR_FILTER
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_YAESU_EMULATION)
#error "You must turn off FEATURE_YAESU_EMULATION if using FEATURE_REMOTE_UNIT_SLAVE"
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && defined(FEATURE_EASYCOM_EMULATION)
#error "You must turn off FEATURE_EASYCOM_EMULATION if using FEATURE_REMOTE_UNIT_SLAVE"
#endif

#if !defined(FEATURE_ELEVATION_CONTROL) && defined(FEATURE_EL_PRESET_ENCODER)
#undef FEATURE_EL_PRESET_ENCODER
#endif

#if !defined(FEATURE_HCO_AZ_POSITION) && !defined(FEATURE_AZ_POSITION_POTENTIOMETER) && !defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) && !defined(FEATURE_AZ_POSITION_PULSE_INPUT) && !defined(FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT) && !defined(FEATURE_AZ_POSITION_HMC5883L)  && !defined(FEATURE_AZ_POSITION_LSM303)
#error "You must specify one AZ position sensor feature"
#endif

#if defined(FEATURE_ELEVATION_CONTROL) && !defined(FEATURE_EL_POSITION_POTENTIOMETER) && !defined(FEATURE_EL_POSITION_ROTARY_ENCODER) && !defined(FEATURE_EL_POSITION_PULSE_INPUT) && !defined(FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB) && !defined(FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB) && !defined(FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT) && !defined(FEATURE_EL_POSITION_LSM303)
#error "You must specify one EL position sensor feature"
#endif

#if (defined(FEATURE_AZ_PRESET_ENCODER) || defined(FEATURE_EL_PRESET_ENCODER) || defined(FEATURE_AZ_POSITION_ROTARY_ENCODER) || defined(FEATURE_EL_POSITION_ROTARY_ENCODER)) && !defined(FEATURE_ROTARY_ENCODER_SUPPORT)
#define FEATURE_ROTARY_ENCODER_SUPPORT
#endif

#if defined(FEATURE_REMOTE_UNIT_SLAVE) && !defined(FEATURE_ONE_DECIMAL_PLACE_HEADINGS)
#define FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#endif

#ifdef FEATURE_WIRE_SUPPORT
#include <Wire.h>
#endif

#ifdef FEATURE_LCD_DISPLAY
#include <LiquidCrystal.h>
#include "Display_LCD.h"
#endif

#ifdef FEATURE_MAX7221_DISPLAY
#include <SPI.h>
#include "Display_MAX7221.h"
#endif

#ifdef FEATURE_FIR_FILTER
#include <FIR.h>
#endif

#endif /* DEPENDENCIES_H_ */
