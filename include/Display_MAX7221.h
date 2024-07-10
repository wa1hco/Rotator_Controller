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

 * Display_MAX7221.h
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */

#ifndef DISPLAY_MAX7221_H_
#define DISPLAY_MAX7221_H_

//-----------------------------------Display_MAX7221 private functions--------------------------
void SPI_Transfer(uint8_t address, uint8_t data);
void initialize_MAX7221_display();
void update_Az_MAX7221_display();

//..................................Display MAX7221 public functions
void display_az_preset_MAX7221(int target_azimuth);

//-----------------------------------Display_MAX7221 private variables--------------------------

#endif /* DISPLAY_MAX7221_H_ */
