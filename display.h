/*
 * display.h
 *
 *  Created on: Feb 12, 2017
 *      Author: jeff
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void loadchars(LiquidCrystal lcd);
void printbigchar(LiquidCrystal lcd, int digit, int col, int row, int symbol = 0);
void printbigazimuth(LiquidCrystal lcd, int azimuth);

#endif /* DISPLAY_H_ */
