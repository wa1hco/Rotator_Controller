/* Arduino Rotator Controller "wa1hco Edition"
 *
   Jeff Millar, WA1HCO
   wa1hco@gmail.com
   
   Anthony Good, K3NG
   anthony.good@gmail.com

   Contributions from John Eigenbode, W3SA
   w3sa@arrl.net
   Contributions: AZ/EL testing and debugging, AZ/EL LCD Enhancements, original North center code, Az/El Rotator Control Connector Pins

   Contributions from Jim Balls, M0CKE
   makidoja@gmail.com
   Contributions: Rotary Encoder Preset Support
   
   Contributions from Gord, VO1GPK
   Contribution: FEATURE_ADAFRUIT_BUTTONS code

   ***************************************************************************************************************

    This program is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License
    
                              http://creativecommons.org/licenses/by-nc-sa/3.0/

                          http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

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
  Serial.print(azimuth,                                     DEC);   
  Serial.println();    
}