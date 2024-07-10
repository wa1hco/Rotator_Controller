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
 */

// serial_command_processing.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef SERIAL_COMMAND_PROCESSING_H_
#define CHECK_SERIAL_COMMAND_PROCESSING_H_

#include "dependencies.h"

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

#ifdef FEATURE_EASYCOM_EMULATION
void easycom_serial_commmand();
#endif

void update_az_variable_outputs(byte);
void print_wrote_to_memory();


#endif /* SERIAL_COMMAND_PROCESSING_H_ */