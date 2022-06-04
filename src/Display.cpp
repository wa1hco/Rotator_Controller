/*
 * Display.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: jeff
 */

#include <Arduino.h>
#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"

#include "global_variables.h"

#include "Display.h"

#ifdef FEATURE_LCD_DISPLAY
#include <LiquidCrystal.h>
#endif

#ifdef FEATURE_WIRE_SUPPORT
#include <Wire.h>
#endif

#ifdef FEATURE_MAX7221_DISPLAY
#include <SPI.h>
#endif

#ifdef FEATURE_LCD_DISPLAY
LiquidCrystal lcd(lcd_4_bit_rs_pin,
                  lcd_4_bit_enable_pin,
                  lcd_4_bit_d4_pin,
                  lcd_4_bit_d5_pin,
                  lcd_4_bit_d6_pin,
                  lcd_4_bit_d7_pin);
/* end of classic 4 bit interface LCD display section */
#endif

//-----------------------------------Display private variables--------------------------
unsigned long last_az_update;
String last_direction_string;\

// SPI transfer function for MAX7221 7 segment LED controller
void SPI_Transfer(uint8_t address, uint8_t data)
{
  SPI.beginTransaction(SPISettings((uint32_t) 400000, MSBFIRST, SPI_MODE0));
  digitalWrite(MAX7221_CS_PIN, LOW);
  SPI.transfer(address); // decode mode register
  SPI.transfer(data);    // bypass decoder for all digits
  digitalWrite(MAX7221_CS_PIN, HIGH);
  SPI.endTransaction();
  delayMicroseconds(10);

  #ifdef DEBUG_HCO_DISPLAY
  Serial.print("SPI_Transfer: addr, data ");
  Serial.print(address);
  Serial.print(", ");
  Serial.print(data);
  Serial.println();
  #endif
}

//------------------------------------------------------------------------------
// 7 segment display on I2C bus
// MAX7221 Register   Command Address
// NOP                0x00
#define DIGIT0        byte(0x01)
#define DIGIT1        byte(0x02)
#define DIGIT2        byte(0x03)
#define DIGIT3        byte(0x04)
//#define Digit4        byte(0x05)
//#define Digit5        byte(0x06)
//#define Digit6        byte(0x07)
//#define Digit7        byte(0x08)
#define DECODE_MODE   byte(0x09)  // BCD mode B (0-9, E, H, L, P, -) or raw, Digits 7 to 0
#define INTENSITY     byte(0x0A)  // PWM 15/16 to 1/16, bits 3 to 0
#define SCAN_LIMIT    byte(0x0B)  // 0 only to 0 to 7, bits 2 to 0
#define SHUTDOWN      byte(0x0C)  // 
#define TEST          byte(0x0F)  // 


// MAX7221 initialize
#ifdef FEATURE_MAX7221_DISPLAY
void initialize_MAX7221_display()
{  
  #ifdef DEBUG_HCO_DISPLAY
  Serial.println("init MAX7221");
  #endif
  
  // Register   D7 D6 D5 D4 D3 D2 D1 D0
  // Segment     x  a  b  c  d  e  f  g
  SPI.begin();
  delayMicroseconds(1000);
  SPI_Transfer(TEST,        byte(0x01)); // take it out of test mode
  delay(1000);                           // msec, display test delay
  SPI_Transfer(TEST,        byte(0x00)); // take it out of test mode
  SPI_Transfer(SHUTDOWN,    byte(0x01)); // normal operation
  SPI_Transfer(DECODE_MODE, byte(0x00)); // bypass decoder for all digits
  SPI_Transfer(INTENSITY,   byte(0x07)); // 8/16 intensity
  SPI_Transfer(SCAN_LIMIT,  byte(0x02)); // 4 digits

  // display 'hco' or 'HCO' with decode turned off
  SPI_Transfer(DIGIT0,      byte(0x17)); // h
  SPI_Transfer(DIGIT1,      byte(0x0D)); // c
  SPI_Transfer(DIGIT2,      byte(0x1D)); // o
  SPI_Transfer(DIGIT3,      byte(0x00)); // \b
  delay(3000); // 3 seconds

  // blank the display
  SPI_Transfer(DIGIT0,      byte(0x00)); // \b
  SPI_Transfer(DIGIT1,      byte(0x00)); // \b
  SPI_Transfer(DIGIT2,      byte(0x00)); // \b
  SPI_Transfer(DIGIT3,      byte(0x00)); // \b

  SPI_Transfer(DECODE_MODE, byte(0x0F)); // decode mode address, set hex decode
}
#endif

// Write azimuth to display
// assumes MAX7221 configured and set for digits
#ifdef FEATURE_MAX7221_DISPLAY
void update_Az_MAX7221_display()
{
  static uint32_t last_az_update = 0;
  uint32_t millis_now = millis();

  if ((millis_now - last_az_update) > DISPLAY_UPDATE_INTERVAL)
  { 
    #ifdef DEBUG_HCO_DISPLAY
    Serial.println("update MAX7221");
    #endif

    uint16_t binaryTemp;
    uint8_t digit[4]; // bcd digits

    last_az_update = millis_now;

    //get the azimuth
    binaryTemp = azimuth; 

    // binary to bcd and write to led
    digit[2] = (uint8_t) binaryTemp % 10; //
    binaryTemp /= 10;
    digit[1] = (uint8_t) binaryTemp % 10;  
    binaryTemp /= 10;
    digit[0] = (uint8_t) binaryTemp % 10;  // most significant digit
    binaryTemp /= 10;
 
    SPI_Transfer(DIGIT0, digit[0]); // \b
    SPI_Transfer(DIGIT1, digit[1]); // \b
    SPI_Transfer(DIGIT2, digit[2]); // \b
  } // if time to update digits
} // update_Az_MAX7221_display()
#endif

//----------------------------------------------------------------------------------------
// Azimuth Pre-set value at Col 16 and row 2
void display_az_preset(int target)
{
  #ifdef FEATURE_LCD_DISPLAY
	int hundreds;
	int tens;
	int ones;

	hundreds =  target / 100;
	tens     = (target - 100 * hundreds) / 10;
	ones     = (target - 100 * hundreds  - 10 * tens);

	// position and blank the target display
	lcd.setCursor(16, 2);

	if(target < 100)
	{
		lcd.print(' ');
	} else
	{
		lcd.print(hundreds);
	}

	if(target < 10)
	{
		lcd.print(' ');
	} else
	{
		lcd.print(tens);
	}
	lcd.print(ones);
	lcd.print(char(223));

	#ifdef DEBUG_DISPLAY
	if (debug_mode)
	{
		Serial.print(F("update_display: "));
		Serial.println(target);
	}
	#endif //DEBUG_DISPLAY
  #endif // feature lcd display
}

//----------------------------------------------------------------
void display_az_string()
{
  #ifdef FEATURE_LCD_DISPLAY
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
  #endif // feature lcd display
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
void initialize_lcd_display()
{
  #ifdef FEATURE_LCD_DISPLAY
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

  lcd.setCursor((LCD_COLUMNS-6)/2,0);
  lcd.print("wa1hco");
  lcd.setCursor((LCD_COLUMNS-18)/2,1);
  lcd.print("Rotor Controller");
  lcd.setCursor((LCD_COLUMNS-16)/2, 2);
  lcd.print("DC Motor, Az Pot");
  last_az_update = millis();

 //============================================
 // code inserted for font setup
 loadchars(); // configure the LCD for Big Fonts

 delay(3000);  // display intro screen for 3 seconds
 #endif // feature lcd display
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
#ifdef FEATURE_LCD_DISPLAY
void update_lcd_display()
{
  // update the LCD display
  static byte lcd_state_row_0 = 0;
  static byte lcd_state_row_1 = 0;

  String direction_string; // temporary string, not really direction

  static int last_azimuth = -1;

  // ---------------------------------------------------------------------
  // target from preset knob
  // azimuth_direction(azimuth)

  if (((millis() - last_az_update) > DISPLAY_UPDATE_TIME) || (push_lcd_update))
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
  if ((millis()-last_az_update) > DISPLAY_UPDATE_TIME)
  {
    if (last_azimuth != azimuth)
    {
      printbigazimuth(azimuth);
      last_azimuth = azimuth;
      lcd_state_row_1 = LCD_HEADING;
    }
  }
  if ((millis() - last_az_update) > DISPLAY_UPDATE_TIME) {last_lcd_update = millis();}
  last_direction_string = direction_string;
} // update_big_display()
#endif

//----------------------------------------------------------------
#ifdef FEATURE_LCD_DISPLAY
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
#endif

//--------------------------------------------------------------
#ifdef FEATURE_LCD_DISPLAY
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
#endif

//------------------------------------------------------------------------
// digit 0-9; col, row in LCD character, symbol
// only printbigchar() need to know about the font design
// prints a digit on a 4x4 character grid with special symbols
// 5th column of digit is blanked
// digit is 0-9 and -1 for blank
// row, col define the upper left corner of the digit display
// symbol put a deg symbol on the character
//
#ifdef FEATURE_LCD_DISPLAY
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
#endif

//-----------------------------------------------------------------------
// print the azimuth big characters
// blank most significant zero
// add degree symbol at the end
#ifdef FEATURE_LCD_DISPLAY
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
#endif

// adafruit has a build in readbuttons() for their LCD on I2C
#ifdef FEATURE_ADAFRUIT_BUTTONS
int readButtons()
{
	int buttons;
	buttons = lcd.readButtons();
	return buttons;
}
#endif


// code from http://www.freetronics.com.au/pages/16x2-lcd-shield-quickstart-guide
// This is more general code for reading buttons based on a resistor cascade

/*
   ADC voltages for the 5 buttons on analog input pin A0:

    RIGHT:  0.00V :   0 @ 8bit ;   0 @ 10 bit
    UP:     0.71V :  36 @ 8bit ; 145 @ 10 bit
    DOWN:   1.61V :  82 @ 8bit ; 329 @ 10 bit
    LEFT:   2.47V : 126 @ 8bit ; 505 @ 10 bit
    SELECT: 3.62V : 185 @ 8bit ; 741 @ 10 bit
 */

#ifdef FEATURE_LCD_BUTTONS

// Pins in use
#define BUTTON_ADC_PIN           A0  // A0 is the button ADC input
#define LCD_BACKLIGHT_PIN         3  // D3 controls LCD backlight
// ADC readings expected for the 5 buttons on the ADC input
#define RIGHT_10BIT_ADC           0  // right
#define UP_10BIT_ADC            145  // up
#define DOWN_10BIT_ADC          329  // down
#define LEFT_10BIT_ADC          505  // left
#define SELECT_10BIT_ADC        741  // right
#define BUTTONHYSTERESIS         10  // hysteresis for valid button sensing window
//return values for ReadButtons()
#define BUTTON_NONE               0  //
#define BUTTON_RIGHT              1  //
#define BUTTON_UP                 2  //
#define BUTTON_DOWN               3  //
#define BUTTON_LEFT               4  //
#define BUTTON_SELECT             5  //

byte ReadButtons()
{
   unsigned int buttonVoltage;
   byte button = BUTTON_NONE;   // return no button pressed if the below checks don't write to btn

   //read the button ADC pin voltage
   buttonVoltage = analogRead( BUTTON_ADC_PIN );
   //sense if the voltage falls within valid voltage windows
   if( buttonVoltage < ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
   }
   //handle button flags for just pressed and just released events
   if( ( buttonWas == BUTTON_NONE ) && ( button != BUTTON_NONE ) )
   {
      //the button was just pressed, set buttonJustPressed, this can optionally be used to trigger a once-off action for a button press event
      //it's the duty of the receiver to clear these flags if it wants to detect a new button change event
      buttonJustPressed  = true;
      buttonJustReleased = false;
   }
   if( ( buttonWas != BUTTON_NONE ) && ( button == BUTTON_NONE ) )
   {
      buttonJustPressed  = false;
      buttonJustReleased = true;
   }

   //save the latest button value, for change event detection next time round
   buttonWas = button;

   return( button );
}
#endif
