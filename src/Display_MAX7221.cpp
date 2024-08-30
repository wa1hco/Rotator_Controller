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

 * Display_MAX7221.cpp
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */

#include <Arduino.h>
#include "dependencies.h"

#ifdef FEATURE_MAX7221_DISPLAY

//-----------------------------------Display_MAX7221 private functions--------------------------
void SPI_Transfer(uint8_t address, uint8_t data);
void initialize_MAX7221_display();
void update_Az_MAX7221_display();

//..................................Display MAX7221 public functions
void display_az_preset_MAX7221(int target_azimuth);

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
#define DIGIT0        char(0x01)
#define DIGIT1        char(0x02)
#define DIGIT2        char(0x03)
#define DIGIT3        char(0x04)
// unused digits
//#define Digit4        char(0x05) 
//#define Digit5        char(0x06)
//#define Digit6        char(0x07)
//#define Digit7        char(0x08)
#define DECODE_MODE   char(0x09)  // BCD mode B (0-9, E, H, L, P, -) or raw, Digits 7 to 0
#define INTENSITY     char(0x0A)  // PWM 15/16 to 1/16, bits 3 to 0
#define SCAN_LIMIT    char(0x0B)  // 0 only to 0 to 7, bits 2 to 0
#define SHUTDOWN      char(0x0C)  // 
#define TEST          char(0x0F)  // 


// MAX7221 initialize and display startup sequence
void initialize_MAX7221_display()
{  
  // 7 segments and data register value
  //     A           40
  //   F   B      02    20
  //     G           01
  //   E   C      04    10
  //     D           08
  //         dp            80
  
  // Digits are numbered from left to right 0, 1, 2, 3

  SPI.begin();
  delay(1);                              //millisec
  SPI_Transfer(TEST,        char(0x01)); // put display in test mode, all segments lit
  delay(1000);                           // msec, display test delay
  SPI_Transfer(TEST,        char(0x00)); // take display out of test mode
  SPI_Transfer(SHUTDOWN,    char(0x01)); // normal operation, not shutdown
  SPI_Transfer(INTENSITY,   char(0x07)); // 8/16 intensity
  SPI_Transfer(SCAN_LIMIT,  char(0x03)); // 4 digits
  SPI_Transfer(DECODE_MODE, char(0x00)); // bypass decoder for all digits

  // display 'hco' 
  SPI_Transfer(DIGIT0,      char(0x17)); // h
  SPI_Transfer(DIGIT1,      char(0x0D)); // c
  SPI_Transfer(DIGIT2,      char(0x1D)); // o
  SPI_Transfer(DIGIT3,      char(0x00)); // \b
  delay(3000); // 3 seconds

  // blank the display
  SPI_Transfer(DIGIT0,      char(0x00)); // \b
  SPI_Transfer(DIGIT1,      char(0x00)); // \b
  SPI_Transfer(DIGIT2,      char(0x00)); // \b
  SPI_Transfer(DIGIT3,      char(0x00)); // \b

  // degree display mode decode digits 0, 1, 2, no decode on digit 3
  SPI_Transfer(DECODE_MODE, char(0x07)); // decode mode address, set hex decode
  
  #ifdef DEBUG_HCO_DISPLAY
  Serial.println("init MAX7221");
  #endif
}

// write to display in decode digits mode
void write_az_digits(int AzTemp)
{
  SPI_Transfer(DECODE_MODE, char(0x07)); // decode 3 az digits, no decode on 4th
  // binary to bcd and write to led
  char digit_2 = (char) (AzTemp % 10); // ones digit
  AzTemp /= 10;
  char digit_1 = (char) (AzTemp % 10); // tens digit
  AzTemp /= 10;
  char digit_0 = (char) (AzTemp % 10); // most significant digit

  SPI_Transfer(DIGIT0, digit_0); // hundreds
  SPI_Transfer(DIGIT1, digit_1); // tens
  SPI_Transfer(DIGIT2, digit_2); // ones
}

// Display update modes: 
//   azimuth, 3 digits from azimuth pot with blank 4th digit
//   preset,  3 digits from preset pot with P 4th digit
//   calibrate, "CALO" or "CALF" while buttons pressed
// update decode mode on each display update
void update_Az_MAX7221_display()
{
  static uint32_t last_az_update = 0;
  uint32_t millis_now = millis();

  if ((millis_now - last_az_update) < DISPLAY_UPDATE_INTERVAL)
  {
    return;
  }
  last_az_update = millis_now; // remember update time

  #ifdef DEBUG_HCO_DISPLAY
  Serial.println("update MAX7221");
  #endif

  // Display updates for each mode
  if(is_display_preset) // azimuth preset mode
  {
    write_az_digits(azimuth_preset); //also sets decode mode
    SPI_Transfer(DIGIT3, char(0x67)); // P, ABEFG,       40 20 04 02 01
    return;
  }

  if(is_CCW_cal_mode)
  {
    //display "CALO"
    SPI_Transfer(DECODE_MODE, char(0x00)); // set no decode
    SPI_Transfer(DIGIT0,      char(0x4E)); // C, ADEF,   40 08 04 02
    SPI_Transfer(DIGIT1,      char(0x77)); // A, ABCEFG, 40 20  10 04 02  01
    SPI_Transfer(DIGIT2,      char(0x0E)); // L, DEF,    08 04 02
    SPI_Transfer(DIGIT3,      char(0x7E)); // O, ABCDEF, 40 20 10 08 04 02
    return;
  }

  if(is_CW_cal_mode)
  {
    // display "CALF"
    SPI_Transfer(DECODE_MODE, char(0x00)); // set no decode
    SPI_Transfer(DIGIT0,      char(0x4E)); // C, ADEF,   40 08 04 02
    SPI_Transfer(DIGIT1,      char(0x77)); // A, ABCEFG, 40 20  10 04 02  01
    SPI_Transfer(DIGIT2,      char(0x0E)); // L, DEF,    08 04 02
    SPI_Transfer(DIGIT3,      char(0x47)); // F, AEFG,   40 04 02 01
    return;
  }
  
  // azimuth mode, not preset or CAL, falls through to here
  {
    write_az_digits(azimuth); // also sets decode modes
    SPI_Transfer(DIGIT3, char(0x00)); // blank last digit
  }

  #ifdef DEBUG_HCO_DISPLAY
  Serial.print("azimuth: ");
  Serial.print(azimuth);
  Serial.print(", ");
  Serial.print("digits: ");
  Serial.print(digit_0);
  Serial.print(", ");
  Serial.print(digit_1);
  Serial.print(", ");
  Serial.print(digit_2);
  Serial.println();
  #endif

} // update_Az_MAX7221_display()

#endif