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