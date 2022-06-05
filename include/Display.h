/*
 * Display.h
 *
 *  Created on: Feb 19, 2017
 *      Author: jeff
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void display_az_preset(int target_azimuth);
void display_az_string();
char *azimuth_direction(int azimuth_in);

void initialize_display();
void update_lcd_display();
void initialize_MAX7221_display();
void update_Az_MAX7221_display();
void display_turning();
void clear_display_row(byte row_number);
void display_calibration_settings();

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
