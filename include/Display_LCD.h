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

 * Display_LCD.h
 *
 *  Created on: Feb 19, 2017
 *      Author: jeff
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void display_az_preset(int target_azimuth);
void display_az_preset_LCD(int target_azimuth);
void display_az_preset_MAX7221(int target_azimuth);
void display_az_string();
char *azimuth_direction(int azimuth_in);

void update_lcd_display();
void display_turning();
void clear_display_row(byte row_number);

void loadchars();
void printbigazimuth(int azimuth);
void printbigchar(int digit, int col, int row, int symbol);

//#define LCD_COLUMNS and ROWS
#define LCD_COLUMNS            20
#define LCD_ROWS                4

#ifdef FEATURE_I2C_LCD
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7
byte lcdcolor = GREEN;  // default color of I2C LCD display
#endif //FEATURE_I2C_LCD

#endif /* DISPLAY_H_ */
