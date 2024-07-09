/*
 * Display_MAX7221.cpp
 *
 *  Created on: July 1, 2024
 *      Author: jeff, wa1hco
 */

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "global_variables.h"
#include "Display_MAX7221.h"

#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"
#include "dependencies.h"



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
  SPI_Transfer(TEST,        char(0x01)); // put display in test mode, all segments lit
  delay(1000);                           // msec, display test delay
  SPI_Transfer(TEST,        char(0x00)); // take display out of test mode
  SPI_Transfer(SHUTDOWN,    char(0x01)); // normal operation, not shutdown
  SPI_Transfer(DECODE_MODE, char(0x00)); // bypass decoder for all digits
  SPI_Transfer(INTENSITY,   char(0x07)); // 8/16 intensity
  SPI_Transfer(SCAN_LIMIT,  char(0x03)); // 4 digits

  // display 'hco' or 'HCO' with decode turned off
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

  SPI_Transfer(DECODE_MODE, char(0x0F)); // decode mode address, set hex decode
}
#endif

// Write azimuth to display
// assumes MAX7221 configured and set for digits
void update_Az_MAX7221_display()
{
  static uint32_t last_az_update = 0;
  uint32_t millis_now = millis();

  if ((millis_now - last_az_update) > DISPLAY_UPDATE_INTERVAL)
  { 
    #ifdef DEBUG_HCO_DISPLAY
    Serial.println("update MAX7221");
    #endif

    int AzTemp;
    char digit_3;
    if(is_display_preset)
    {
      AzTemp = azimuth_preset;
      digit_3 = char(0x0E); // "P"
    }
    else
    {
      AzTemp = azimuth; // working variable display conversion
      digit_3 = char(0x0F); // blank
    }

    last_az_update = millis_now;

    // binary to bcd and write to led
    char digit_2 = (char) (AzTemp % 10); // ones digit
    AzTemp /= 10;
    char digit_1 = (char) (AzTemp % 10); // tens digit
    AzTemp /= 10;
    char digit_0 = (char) (AzTemp % 10); // most significant digit
 
    SPI_Transfer(DIGIT0, digit_0); // hundreds
    SPI_Transfer(DIGIT1, digit_1); // tens
    SPI_Transfer(DIGIT2, digit_2); // ones
    SPI_Transfer(DIGIT3, digit_3); // preset or blank
    

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

  } // if time to update digits
} // update_Az_MAX7221_display()

