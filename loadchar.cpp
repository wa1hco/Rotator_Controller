/*
 * loadchar.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: jeff
 */
#include "Arduino.h"
#include <LiquidCrystal.h>

// define function to load special characters into in the LCD
void loadchars(LiquidCrystal lcd)
{
  // Define the custom characters, only loadchars() needs to know
  byte custchar[8][8] =
  {
	{    // 0, upper half
	 0b11111,
	 0b11111,
	 0b11111,
	 0b11111,
	 0b00000,
	 0b00000,
	 0b00000,
	 0b00000
	}, { // 1, lower right corner
	 0b00000,
	 0b00000,
	 0b00000,
	 0b00001,
	 0b00011,
	 0b00111,
	 0b01111,
	 0b11111
	}, { // 2, upper half
	 0b11111,
	 0b11111,
	 0b11111,
	 0b11111,
	 0b00000,
	 0b00000,
	 0b00000,
	 0b00000
	}, { // 3, lower left corner
	 0b00000,
	 0b00000,
	 0b00000,
	 0b10000,
	 0b11000,
	 0b11100,
	 0b11110,
	 0b11111
	}, { // 4, upper right corner
	 0b11111,
	 0b01111,
	 0b00111,
	 0b00011,
	 0b00001,
	 0b00000,
	 0b00000,
	 0b00000
	}, { // 5, lower half
	 0b00000,
	 0b00000,
	 0b00000,
	 0b00000,
	 0b11111,
	 0b11111,
	 0b11111,
	 0b11111
	}, { // 6, upper left corner
	 0b11111,
	 0b11110,
	 0b11100,
	 0b11000,
	 0b10000,
	 0b00000,
	 0b00000,
	 0b00000
	}, { // 7, deg sym0bol
	 0b00000,
	 0b00000,
	 0b01110,
	 0b10001,
	 0b10001,
	 0b10001,
	 0b01110,
	 0b00000
	}
  };
  lcd.createChar(0, custchar[0]);
  lcd.createChar(1, custchar[1]);
  lcd.createChar(2, custchar[2]);
  lcd.createChar(3, custchar[3]);
  lcd.createChar(4, custchar[4]);
  lcd.createChar(5, custchar[5]);
  lcd.createChar(6, custchar[6]);
  lcd.createChar(7, custchar[7]);
}




