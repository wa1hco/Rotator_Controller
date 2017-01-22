
// define function to load special characters into in the LCD
void loadchars() 
{
  // Define the custom characters, only loadchars() needs to know
  byte custchar[8][8] = {
   { // 0
     B11111,
     B11111,
     B11111,
     B11111,
     B00000,
     B00000,
     B00000,
     B00000
   }, { // 1
     B00000,
     B00000,
     B00000,
     B00001,
     B00011,
     B00111,
     B01111,
     B11111
   }, { // 2
     B11111,
     B11111,
     B11111,
     B11111,
     B00000,
     B00000,
     B00000,
     B00000   
   }, { // 3
     B00000,
     B00000,
     B00000,
     B10000,
     B11000,
     B11100,
     B11110,
     B11111
   }, { // 4
     B11111,
     B01111,
     B00111,
     B00011,
     B00001,
     B00000,
     B00000,
     B00000
   }, { // 5
     B00000,
     B00000,
     B00000,
     B00000,
     B11111,
     B11111,
     B11111,
     B11111
   }, { // 6
     B11111,
     B11110,
     B11100,
     B11000,
     B10000,
     B00000,
     B00000,
     B00000
   }, { // 7
     B00000,
     B00000,
     B01110,
     B10001,
     B10001,
     B10001,
     B01110,
     B00000
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

// digit 0-9; col, row in LCD character, symbol
// only printbigchar() need to know about the font design
void printbigchar(byte digit, byte col, byte row, byte symbol = 0) 
{
  // Define the large fonts, borrowed from GreenHerron RT-21 pictures
  const byte bignums[10][4][4] = 
  {
    { //0 tested
      {  1,   2,   2,   3},
      {255, 254, 254, 255},
      {255, 254, 254, 255},
      {  4,   5,   5,   6}
    },{ //1 tested
      {254,   1, 255, 254},
      {254, 254, 255, 254},
      {254, 254, 255, 254},
      {254,   5, 255,   5}
    },{ //2 tested
      {  1,   2,   2,   3},
      {254, 254,   5,   6},
      {  1,   2, 254, 254},
      {255,   5,   5,   5}
    },{ //3
      {  1,   2,   2,   3},
      {254, 254,   5,   6},
      {254,   2,   2,   3},
      {  4,   5,   5,   6}
    },{ //4 tested
      {255, 254, 254, 255},
      {255, 254, 254, 255},
      {  4,   2,   2, 255},
      {254, 254, 254, 255}
    },{ // 5 tested
      {255,   2,   2,   2},
      {255, 254, 254, 254},
      {  4,   2,   2,   3},
      {  4,   5,   5,   6}
    },{ // 6 tested
      {  1,   2,   2,   3},
      {255, 254, 254, 254},
      {255,   2,   2,   3},
      {  4,   5,   5,   6}
    },{ // 7 tested
      {  2,   2,   2, 255},
      {254, 254,   5,   6},
      {  5,   2, 254, 254},
      {255, 254, 254, 254}
    },{ //8 tested
      {  1,   2,   2,   3},
      {  4,   5,   5,   6},
      {  1,   2,   2,   3},
      {  4,   5,   5,   6}
    },{ //9 tested
      {  1,   2,   2,   3},
      {  4,   5,   5, 255},
      {254, 254, 254, 255},
      {  4,   5,   5,   6}
    }
  }; 

  // print the large characters
  if ((digit > 0) & (digit < 9)) 
  {
    // loop over the font and write to lcd
    for (int i = 0; i < 4; i++)    // for each row
    {
      lcd.setCursor(col, row + i); //set the row address
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
    else if (symbol == 2) 
    { // not a useful symbol
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
    for (int i = 0; i < 4; i++)    // for each row
    {
      lcd.setCursor(col, row + i); //set the row address
      for (int j = 0; j < 5; j++)  // for each column of font
      {
        lcd.write(254); // increments the col address also
      }
    }
  }
}

// print the azimuth big characters
// blank most significant zero
// add degree symbol at the end
void printbigazimuth(int azimuth)
{
  int hundreds;
  int tens;
  int ones;
  
  hundreds = azimuth / 100;
  tens     = (azimuth - 100 * hundreds) / 10;
  ones     = (azimuth - 100 * hundreds - 10 * tens);
  if (hundreds > 0)
  {
    //printbigchar(byte digit, byte col, byte row, byte symbol = 0)
    printbigchar(hundreds, 0,   0,  0);     
  } 
  else
  {
    // flag printing blank
    printbigchar(-1, 0, 0, 0);
  }
  printbigchar(tens,     5,   0,  0);     
  printbigchar(ones,    10,   0,  1);  // flag degree symbol
}

