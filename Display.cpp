/*
 * Display.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: jeff
 */

#include <Arduino.h>
#include <LiquidCrystal.h>

#include "rotator_features.h"
#include "settings.h"
#include "macros.h"

#include "Display.h"

// global variables referenced below
extern LiquidCrystal lcd;
extern int azimuth;
extern byte az_state;
extern String last_direction_string;
extern unsigned long last_lcd_update;
extern byte push_lcd_update;
extern int target_azimuth;

//----------------------------------------------------------------------------------------
// Azimuth Pre-set value at Col 16 and row 2
void display_az_preset(int target_azimuth)
{
  String direction_string; // temporary string, not really direction
  char workstring[7];

  #ifdef FEATURE_AZ_PRESET_ENCODER
  unsigned int target = 0;
  #endif

  #ifdef FEATURE_AZ_PRESET_ENCODER
  target = az_encoder_raw_degrees;
  // wrap, twice if necessary
  if (target > (359*LCD_HEADING_MULTIPLIER)) {target = target - (360 * LCD_HEADING_MULTIPLIER);}
  if (target > (359*LCD_HEADING_MULTIPLIER)) {target = target - (360 * LCD_HEADING_MULTIPLIER);}

  if (preset_encoders_state == ENCODER_AZ_PENDING)
  {
    // position and blank the target display
    lcd.setCursor(16, 2);
    lcd.print("    ");
    direction_string = "";
    dtostrf(target/LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring);
    direction_string.concat(workstring);
    direction_string.concat(char(223)); // deg symbol
    lcd.setCursor(17, 2);
    lcd.print(direction_string);

    lcd_state_row_0 = LCD_TARGET_AZ;
    #ifdef DEBUG_DISPLAY
    if (debug_mode)
    {
      Serial.print(F("update_display: "));
      Serial.println(direction_string);
    }
    #endif //DEBUG_DISPLAY

  } else // not state = ENCODER_AZ_PENDING
  {
  #endif // not FEATURE_AZ_PRESET_ENCODER, display azimuth preset pot value

    // display the target azimuth all the time
    lcd.setCursor(16, 2);
    lcd.print("    ");
    direction_string = "";
    dtostrf(target_azimuth/LCD_HEADING_MULTIPLIER, 1, LCD_DECIMAL_PLACES, workstring);
    direction_string.concat(workstring);
    direction_string.concat(char(223)); // deg symbol
    lcd.setCursor(16, 2);
    lcd.print(direction_string);

    #ifdef DEBUG_DISPLAY
    if (debug_mode)
    {
      Serial.print(F("update_display: "));
      Serial.println(direction_string);
    }
    #endif //DEBUG_DISPLAY

  #ifdef FEATURE_AZ_PRESET_ENCODER
  } //(preset_encoders_state == ENCODER_AZ_PENDING)
  #endif //FEATURE_AZ_PRESET_ENCODER
}

//----------------------------------------------------------------
void display_az_string()
{
  String direction_string;
  direction_string = azimuth_direction(azimuth);  // NE, ENE, NNE, etc

  lcd.setCursor(16, 0);
  lcd.print("    ");
  lcd.setCursor(16, 0);
  lcd.print(direction_string);
  #ifdef DEBUG_AZ_STR
  if (debug_mode)
  {
    Serial.print(F("update_display: "));
    Serial.println(direction_string);
  }
  #endif //DEBUG_AZ_STR
}

//--------------------------------------------------------------
char *azimuth_direction(int azimuth_in)
{
  azimuth_in = azimuth_in / HEADING_MULTIPLIER;

  if (azimuth_in > 348) {return (char *) "N";}
  if (azimuth_in > 326) {return (char *) "NNW";}
  if (azimuth_in > 303) {return (char *) "NW";}
  if (azimuth_in > 281) {return (char *) "WNW";}
  if (azimuth_in > 258) {return (char *) "W";}
  if (azimuth_in > 236) {return (char *) "WSW";}
  if (azimuth_in > 213) {return (char *) "SW";}
  if (azimuth_in > 191) {return (char *) "SSW";}
  if (azimuth_in > 168) {return (char *) "S";}
  if (azimuth_in > 146) {return (char *) "SSE";}
  if (azimuth_in > 123) {return (char *) "SE";}
  if (azimuth_in > 101) {return (char *) "ESE";}
  if (azimuth_in >  78) {return (char *) "E";}
  if (azimuth_in >  56) {return (char *) "ENE";}
  if (azimuth_in >  33) {return (char *) "NE";}
  if (azimuth_in >  11) {return (char *) "NNE";}
  return (char *) "N";
}


//--------------------------------------------------------------
void initialize_display()
{
  #ifndef OPTION_INITIALIZE_YOURDUINO_I2C
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  #endif

  #ifdef OPTION_INITIALIZE_YOURDUINO_I2C
  lcd.begin (16,2);
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(LED_ON);
  #endif //OPTION_INITIALIZE_YOURDUINO_I2C

  #ifdef FEATURE_I2C_LCD
  lcd.setBacklight(lcdcolor);
  #endif //FEATURE_I2C_LCD

  lcd.setCursor(((LCD_COLUMNS-6)/2),0);
  lcd.print("wa1hco");
  if (LCD_COLUMNS < 20)
  {
    lcd.setCursor(((LCD_COLUMNS-15)/2),1);  // W3SA
  } else
  {
    lcd.setCursor(((LCD_COLUMNS-18)/2),1);
  }
  lcd.print("Rotor Controller");
  last_lcd_update = millis();

 //============================================
 // code inserted for font setup
 loadchars(); // configure the LCD for Big Fonts
}

//--------------------------------------------------------------
// started from update_display(), stripped elevation options

//Display Definitions
// Col 00-14, Row 0-3, Big fonts and deg symbol
// Col 15,    Row 0,   Deg symbol
// Col 15,    Row 1,   {0 to 9}, Speed number
// Col 15,    Row 2,   TBD
// Col 15,    Row 3,   Azimuth tenths
// Col 16-20, Row 0,   {N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW}
// Col 16-20, Row 1,   {CW, CCW}, Soft Limits
// Col 16-20, Row 2,   {000 to 359}, Preset knob position
// Col 16-20, Row 3,   {MAN, PRE, REM, M/S, M/C, S/C, OF1, OF2, DBG}
//-----------------------------------------------------------------------
void update_display()
{
  // update the LCD display
  static byte lcd_state_row_0 = 0;
  static byte lcd_state_row_1 = 0;

  String direction_string; // temporary string, not really direction

  static int last_azimuth = -1;

  // ---------------------------------------------------------------------
  // target from preset knob
  // azimuth_direction(azimuth)

  if (((millis() - last_lcd_update) > LCD_UPDATE_TIME) || (push_lcd_update))
  {
    // initialization
    if ((lcd_state_row_0 == 0) && (lcd_state_row_1 == 0))
    {
      lcd.clear();
      // lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0); // cursor to start of centered string
      lcd.setCursor(16, 0);
      //TODO, this seems uninitialized
      lcd.print(direction_string);
      lcd_state_row_0 = LCD_DIRECTION;
    }
    display_az_preset(target_azimuth);
    display_az_string();
    display_turning();
    push_lcd_update = 0;
  }

  // Large Azimuth Characters
  if ((millis()-last_lcd_update) > LCD_UPDATE_TIME)
  {
    if (last_azimuth != azimuth)
    {
      printbigazimuth(azimuth);
      last_azimuth = azimuth;
      lcd_state_row_1 = LCD_HEADING;
    }
  }
  if ((millis() - last_lcd_update) > LCD_UPDATE_TIME) {last_lcd_update = millis();}
  last_direction_string = direction_string;
} // update_big_display()

//----------------------------------------------------------------
void display_turning()
{
  String direction_string;
  #ifdef DEBUG_TURNING
    Serial.print("az_state = ");
    Serial.print(az_state);
    Serial.print(", az_queue = ");
    Serial.println(az_request_queue_state);
  #endif // DEBUG_TURNING
  if (az_state == IDLE) // if not idle, display CW or CCW messages
  {
    lcd.setCursor(16, 1);
    lcd.print("    ");
    //direction_string = "IDLE";
    //lcd.setCursor(16, 1);
    //lcd.print(direction_string);
    #ifdef DEBUG_TURNING
    if (debug_mode)
    {
      Serial.print(F("update_display: "));
      Serial.println(direction_string);
    }
    #endif //DEBUG_TURNING
  }

  if ((az_state == SLOW_START_CW) ||
	  (az_state == NORMAL_CW) ||
	  (az_state == SLOW_DOWN_CW) ||
	  (az_state == TIMED_SLOW_DOWN_CW))
  {
    lcd.setCursor(16, 1);
    lcd.print("    ");
    direction_string = "CW";
    lcd.setCursor(16, 1);
    lcd.print(direction_string);
    #ifdef DEBUG_TURNING
    if (debug_mode)
    {
      Serial.print(F("update_display: "));
      Serial.println(direction_string);
    }
    #endif //DEBUG_TURNING
  }

  if ((az_state == SLOW_START_CCW) ||
	  (az_state == NORMAL_CCW) ||
	  (az_state == SLOW_DOWN_CCW) ||
	  (az_state == TIMED_SLOW_DOWN_CCW))
  {
    lcd.setCursor(16, 1);
    lcd.print("    ");
    direction_string = "CCW";
    lcd.setCursor(16, 1);
    lcd.print(direction_string);
    #ifdef DEBUG_TURNING
    if (debug_mode)
    {
      Serial.print(F("update_display: "));
      Serial.println(direction_string);
    }
    #endif //DEBUG_TURNING
  }
}

//--------------------------------------------------------------
void clear_display_row(byte row_number)
{
  lcd.setCursor(0,row_number);
  for (byte x = 0; x < LCD_COLUMNS; x++)
  {
    lcd.print(" ");
  }
}

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

//-----------------------------------------------------------------------
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

  //printbigchar(LiquidCrystal lcd, byte digit, byte col, byte row, byte symbol = 0)
  printbigchar(hundreds, 0,   0,  0);
  printbigchar(tens,     5,   0,  0);
  printbigchar(ones,    10,   0,  1);  // flag degree symbol
}
