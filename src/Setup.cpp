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


   ***************************************************************************************************************    

    ***** https://github.com/k3ng/k3ng_rotator_controller/wiki *****


    All copyrights are the property of their respective owners
*/

// Arduino environment
#include <global_variables.h>
#include <Arduino.h>
#include "eeprom_local.h"

#include "global_variables.h"
#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "dependencies.h"

#include "serial_command_processing.h"
#include "Input.h"
#include <MsTimer2.h>

extern FIR<float, 31> fir_top;
extern FIR<float, 31> fir_bot;

// define external functions
void initialize_serial();
void initialize_peripherals();
void initialize_pins();
void read_headings();
void initialize_interrupts();
void TimedService();

void setup() 
{
  delay(1000);
  initialize_serial();
  initialize_peripherals();
  read_settings_from_eeprom(); 
  initialize_pins();
  //initialize_PID();

  read_headings();
  #ifdef FEATURE_YAESU_EMULATION
  report_current_azimuth();      // Yaesu - report the azimuth right off the bat without a C command; the Arduino doesn't wake up quick enough
                                 // to get first C command from HRD and if HRD doesn't see anything it doesn't connect
  #endif //FEATURE_YAESU_EMULATION                                 

  #ifdef FEATURE_TIMED_BUFFER 
  timed_buffer_status = EMPTY;
  #endif //FEATURE_TIMED_BUFFER 
  
  #ifdef FEATURE_LCD_DISPLAY
  initialize_lcd_display();
  #endif

  #ifdef FEATURE_MAX7221_DISPLAY
  initialize_MAX7221_display();
  #endif

  initialize_rotary_encoders(); 
  initialize_interrupts();

  /*
  FIR filter designed with
  http://t-filter.appspot.com

  sampling frequency: 200 Hz

  * 0 Hz - 20 Hz
    gain = 1
    desired ripple = 3 dB
    actual ripple = 1.1761649821089057 dB

  * 50 Hz - 100 Hz
    gain = 0
    desired attenuation = -60 dB
    actual attenuation = -66.03868970041013 dB

  */
  #ifdef FEATURE_FIR_FILTER
  #define FILTER_TAP_NUM 15
  static float filter_taps[FILTER_TAP_NUM] = {
    -0.004942353838242621,
    -0.018993776250465885,
    -0.034906800521284226,
    -0.02689862294067124,
     0.03329097458407915,
     0.14355647714952716,
     0.25568358157357024,
     0.3034216074312329,
     0.25568358157357024,
     0.14355647714952716,
     0.03329097458407915,
    -0.02689862294067124,
    -0.034906800521284226,
    -0.018993776250465885,
    -0.004942353838242621
  };

  fir_top.setFilterCoeffs(filter_taps);
  fir_bot.setFilterCoeffs(filter_taps);
  #endif // FIR filter

  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TIME_BETWEEN_AZ_ADC_READ, TimedService); // interval, function call
  // interrupts enabled after this point
  MsTimer2::start(); 
} //setup()
