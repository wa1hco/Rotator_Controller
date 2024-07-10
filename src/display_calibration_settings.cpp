/*
 * debug_calibration_settings.cpp
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */
#include "dependencies.h"

#include "display_calibration_settings.h"

// called from \C command
// prints global variables related to calibration
void display_calibration_settings()
{
  Serial.println(F("Analog and cal settings: "));
  Serial.print("Vtop: ");
  Serial.print(Vtop);         // top voltage
  Serial.print(", Vbot: ");
  Serial.print(Vbot);         // bottom voltage
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