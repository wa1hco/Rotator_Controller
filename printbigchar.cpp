/*
 * printbigchar.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: jeff
 */
//------------------------------------------------------------------------
// digit 0-9; col, row in LCD character, symbol
// only printbigchar() need to know about the font design
// prints a digit on a 4x4 character grid with special symbols
// 5th column of digit is blanked
// digit is 0-9 and -1 for blank
// row, col define the upper left corner of the digit display
// symbol put a deg symbol on the character
//
#include "Arduino.h"
#include <LiquidCrystal.h>
void printbigchar(LiquidCrystal lcd, int digit, int col, int row, int symbol = 0)
{
  // Define the large fonts, borrowed from GreenHerron RT-21 pictures
  // 255 defined as all pixels on, 254 defined as all pixels off
  #define UH  0
  #define LR  1
  //#define UH  2  // this one is a duplicate
  #define LL  3
  #define UR  4
  #define LH  5
  #define UL  6
  #define deg 7
  const byte bignums[10][4][4] =
  {
	{   // 0, zero
	  { LR,  UH,  UH,  LL},
	  {255, 254, 254, 255},
	  {255, 254, 254, 255},
	  { UR,  LH,  LH,  UL}
	},{ // 1, one
	  {254,  LR, 255, 254},
	  {254, 254, 255, 254},
	  {254, 254, 255, 254},
	  {254,  LH, 255,  LH}
	},{ // 2, two
	  { LR,  UH,  UH,  LL},
	  {254, 254,  LH,  UL},
	  { LR,  UH, 254, 254},
	  {255,  LH,  LH,  LH}
	},{ // 3, three
	  { LR,  UH,  UH,  LL},
	  {254, 254,  LH,  UL},
	  {254, 254,  UH,  LL},
	  { UR,  LH,  LH,  UL}
	},{ // 4, four
	  {255, 254, 254, 255},
	  {255, 254, 254, 255},
	  { UR,  UH,  UH, 255},
	  {254, 254, 254, 255}
	},{ // 5, five
	  {255,  UH,  UH,  UH},
	  { UR,  LH,  LH,  LL},
	  {254, 254, 254, 255},
	  { UR,  LH,  LH,  UL}
	},{ // 6, six
	  { LR,  UH,  UH,  LL},
	  {255, 254, 254, 254},
	  {255,  UH,  UH,  LL},
	  { UR,  LH,  LH,  UL}
	},{ // 7, seven
	  { UR,  UH,  UH, 255},
	  {254, 254,  LR,  UL},
	  {254, 254, 255, 254},
	  {254, 254, 255, 254}
	},{ // 8, eight
	  { LR,  UH,  UH,  LL},
	  { UR,  LH,  LH,  UL},
	  { LR,  UH,  UH,  LL},
	  { UR,  LH,  LH,  UL}
	},{ // 9, nine
	  { LR,  UH,  UH,  LL},
	  { UR,  LH,  LH, 255},
	  {254, 254, 254, 255},
	  { UR,  LH,  LH,  UL}
	}
  };

  // print the large characters
  if ((digit >= 0) & (digit <= 9)) // if valid digit
  {
	// loop over the font and write to lcd
	for (int i = 0; i < 4; i++)    // for each row of fond
	{
	  lcd.setCursor(col, row + i); // set the row address
	  for (int j = 0; j < 4; j++)  // for each column of font
	  {
		lcd.write(bignums[digit][i][j]); // increments the col address also
	  }
	  lcd.write(254); // write blank to 5th column
	}
	// add the degree symbol
	if (symbol == 1)
	{
	  lcd.setCursor(col + 4, row + 0);
	  lcd.write(7); // deg symbol
	}
	else if (symbol == 2) // not a useful symbol
	{
	  lcd.setCursor(col + 3, row);
	  lcd.write(4);
	  lcd.setCursor(col + 3, row + 1);
	  lcd.write(4);
	}
	lcd.setCursor(col + 4, row);
  }
  else if ( digit == -1) // print a blank
  {
	// loop over the cell to write a blank
	for (int i = 0; i < 4; i++)    // for each row of fond
	{
	  lcd.setCursor(col, row + i); //set the row address
	  for (int j = 0; j < 5; j++)  // for each column of font
	  {
		lcd.write(254); // increments the col address also
	  }
	}
  }
}





