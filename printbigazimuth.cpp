/*
 * printbigazimuth.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: jeff
 */
#include "Arduino.h"
#include <LiquidCrystal.h>

void printbigchar(LiquidCrystal lcd, int digit, int col, int row, int symbol = 0);

//-----------------------------------------------------------------------
// print the azimuth big characters
// blank most significant zero
// add degree symbol at the end
void printbigazimuth(LiquidCrystal lcd, int azimuth)
{
  int hundreds;
  int tens;
  int ones;

  hundreds =  azimuth / 100;
  tens     = (azimuth - 100 * hundreds) / 10;
  ones     = (azimuth - 100 * hundreds - 10 * tens);

  //printbigchar(LiquidCrystal lcd, byte digit, byte col, byte row, byte symbol = 0)
  printbigchar(lcd, hundreds, 0,   0,  0);
  printbigchar(lcd, tens,     5,   0,  0);
  printbigchar(lcd, ones,    10,   0,  1);  // flag degree symbol
}



