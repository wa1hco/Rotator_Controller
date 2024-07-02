/*
 * debug_calibration_settings.cpp
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */

#include "global_variables.h"
#include "display_calibration_settings.h"

#include <Arduino.h>
#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"

// called from \C command
// prints global variables related to calibration
void display_calibration_settings()
{
  Serial.println(F("Analog and cal settings: "));
  Serial.print("Vt: ");
  Serial.print(Vt);         // top voltage
  Serial.print(", Vb: ");
  Serial.print(Vb);         // bottom voltage
  Serial.println();  
  Serial.print("analog_az                    ");
  Serial.println(analog_az,                                 DEC);
  Serial.print("analog_az_full_ccw           ");
  Serial.println(configuration.analog_az_full_ccw,          DEC);
  Serial.print("analog_az_full_cw            ");
  Serial.println(configuration.analog_az_full_cw,           DEC);
  Serial.print("azimuth_starting_point       ");
  Serial.println(configuration.azimuth_starting_point,      DEC);
  Serial.print("azimuth_rotation_capability  ");
  Serial.println(configuration.azimuth_rotation_capability, DEC);
  Serial.print("azimuth:                     ");
  Serial.println(azimuth,                                   DEC);       
}