
// define function to load special characters into in the LCD
void loadchars() 
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

//------------------------------------------------------------------------
// digit 0-9; col, row in LCD character, symbol
// only printbigchar() need to know about the font design
// prints a digit on a 4x4 character grid with special symbols
// 5th column of digit is blanked
// digit is 0-9 and -1 for blank
// row, col define the upper left corner of the digit display
// symbol put a deg symbol on the character
//
void printbigchar(int digit, int col, int row, int symbol = 0) 
{
  // Define the large fonts, borrowed from GreenHerron RT-21 pictures
  // 255 defined as all pixels on, 254 defined as all pixels off
  #define UH  0
  #define LR  1
  //#define UH  2
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

// print the azimuth big characters
// blank most significant zero
// add degree symbol at the end
void printbigazimuth(int azimuth)
{
  int hundreds;
  int tens;
  int ones;
  
  hundreds =  azimuth / 100;
  tens     = (azimuth - 100 * hundreds) / 10;
  ones     = (azimuth - 100 * hundreds - 10 * tens);
  
  //printbigchar(byte digit, byte col, byte row, byte symbol = 0)
  printbigchar(hundreds, 0,   0,  0);     
  printbigchar(tens,     5,   0,  0);     
  printbigchar(ones,    10,   0,  1);  // flag degree symbol
}

