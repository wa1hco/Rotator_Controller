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

// eeprom_local.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef EEPROM_LOCAL_H_
#define EEPROM_LOCAL_H_

// eeprom functions
void initialize_eeprom_with_defaults();
void write_settings_to_eeprom();
void read_settings_from_eeprom();

#endif /* EEPROM_LOCAL_H_ */