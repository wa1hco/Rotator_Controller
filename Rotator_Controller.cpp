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

     ASCII Art Schematic
 
                               +----------------Yaesu Pin 1
                               |
                               N
            D6---{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 2
                               |
                               N
            D7---{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
            A0-------------+---------------------Yaesu Pin 4
                           |
                        [0.01uF]
                           |
                          GND
 
            D10---{4.7K}---+---------------------Yaesu Pin 3
                           |
                         [10uF]
                           |
                          GND
                               
                               
            Not Connected------------------------Yaesu Pin 6
 
            GND----------------------------------Yaesu Pin 5

    Alternatively Yaesu Pin 3 can be connected to Pin 6 if variable speed functionality (X commands) are not desired.  This will feed +5V directly
    into the speed voltage pin and set the unit for maximum speed all the time

    Yaesu Azimuth Only Rotator Controller Connector Pins
 
       6 || 5
      4      3
        2  1
    1 - ground to rotate L
    2 - ground to rotate R
    3 - speed voltage (input); 4.5V = max speed
    4 - analog azimuth voltage (output); 0V = full CCW, ~4.9V = full CW
    5 - ground
    6 - +5V or so

    Yaesu Az/El Rotator Control Connector Pins
 
        7 | | 6
       3   8   1
        5     4
           2
 
    1 - 2 - 4.5 VDC corresponding to 0 to 180 degrees elevation
    2 - Connect to Pin 8 to rotate right (clockwise)
    3 - Connect to Pin 8 to rotate Down
    4 - Connect to Pin 8 to rotate left (counterclockwise)
    5 - Connect to Pin 8 to rotate Up
    6 - 2 - 4.5 VDC corresponding to 0 to 450 degrees rotation
    7 - 13 - 6 VDC at up to 200 mA
    8 - Common ground

     ASCII Art Schematic
 
                               +----------------Yaesu Pin 4
                               |
                               N
            D6--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 2
                               |
                               N
            D7--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
                               +----------------Yaesu Pin 5
                               |
                               N
            D8--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 3
                               |
                               N
            D9--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
            A0-----------------------------------Yaesu Pin 6
 
            A1-----------------------------------Yaesu Pin 1
 
            NC-----------------------------------Yaesu Pin 7
 
            GND----------------------------------Yaesu Pin 8

    Quick Start
 
    In order to test and calibrate your unit, connect the Serial Monitor to the COM port set for 9600 and carriage return
    All command letters must be uppercase.
    The backslash d (\d) command toggles debug mode which will periodically display key parameters.
 
    To test basic operation, send commands using Serial Monitor:
    Rotate left(CCW): L 
    Rotate right(CW): R
    Stop rotation: A or S commands
    Read the current azimuth: C
    Go to an azimuth automatically: M command (examples: M180 = go to 180 degrees, M010 = go to 10 degrees
 
    To calibrate the unit, send the O command and rotate to 180 degrees / full CCW and send a carriage return, then
    send the F command and rotate to 270 degrees / full CW and send a carriage return (assuming a 450 degrees rotation rotator).
    If you are operating a 360 degree rotation rotator, for the F command rotate to 180 degrees full CW, not 270.
 
    ( CW means clockwise (or LEFT on Yaesu rotators) and CCW means counter clockwise (or RIGHT on Yaesu rotators))
 
    To use this code with AZ/EL rotators, uncomment the FEATURE_ELEVATION_CONTROL line below
 
    It does properly handle the 450 degree rotation capability of the Yaesu rotators.
 
    This code has been successfully interfaced with non-Yaesu rotators. Email me if you have a rotator you would like to interface this to.
 
    With the addition of a reasonable capacity DC power supply and two relays, this unit could entirely replace a control unit if desired.

    9/12/11 W3SA JJE added code to correct elevation display which was not following A1 input (map function was not working using the variables)
    Added code to keep azimuth and elevation updated if changed from the rotor control unit.
    Added code to handle flipped azimuth of antenna when elevation passes 90 degrees.
    Changed LCD display to display Direction, Azimuth and Elevation of antenna(s) on first line of display and actual Rotor azimuth and elevation on the second line
    Then when the elevation has passed 90 degrees you would get:
        NNE A 15 E 75
 		RTR A 195 E 115
    Otherwise it would be
        NNE A 15 E 75
 		RTR A 15 E 75
 
 
    01/02/13 M0CKE added code for a cheap rotary encoder input for setting the azimuth direction, rotary encoder connected to pins 6 & 7 with the center to ground,
    degrees are set by turning the encoder cw or ccw with 1 click being 1 degree if turned slowly or 1 click being 10 degree by turning quickly, this replaces the 
    preset pot and can be enabled by uncommenting "#define FEATURE_AZ_PRESET_ENCODER" below.

*/

// Arduino environment
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <MsTimer2.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "rotator_features.h"
#include "rotator_pins_custom_board.h"
#include "dependencies.h"
#include "macros.h"
#include "settings.h"

// #include "PID.h"

//#define CODE_VERSION "2013091101"
//#define CODE_VERSION "2017021101"
#define CODE_VERSION "2021040701"

// Project functions
#include "global_variables.h"
#include "Service_Blink_LED.h"
#include "Display.h"
#include "serial_command_processing.h"
#include "eeprom_local.h"
#include "utilities_local.h"

void initialize_serial();
void initialize_peripherals();
void read_settings_from_eeprom();
void initialize_pins();
//void initialize_PID();
void initialize_rotary_encoders();
void check_serial();
void read_headings();
void service_request_queue();
void service_rotation_azimuth();
void az_check_operation_timeout();
void check_buttons();
void check_overlap();
void check_az_speed_pot();
void check_az_preset_potentiometer();
void output_debug();
void initialize_interrupts();
void profile_loop_time();
void TimedService();
void ReadAzimuthISR();

#ifdef FEATURE_ROTATION_INDICATOR_PIN
void service_rotation_indicator_pin();
#endif //FEATURE_ROTATION_INDICATOR_PIN  

// Enter at TimeInterval specified in global
// calls ReadAzimuthISR() to read ADCs
// Sets DisplayFlag when it's time to update Display
void TimedService() 
{
  ReadAzimuthISR();
}

//*********************************************************************************
// Entered at TimeBetweenInterrupt intervals, 
// Called from interrupt context, ADC reads occur at 2 ms, 500 Hz
// read both ADC input close together in time
// then do the peak processing
// This function is called from MsTimer2 every TIMEBETWEENINTERRUPTS msec
void ReadAzimuthISR() 
{
  #ifdef AZIMUTH_INTERRUPT
  unsigned int previous_raw_azimuth = raw_azimuth;

  #ifdef DEBUG_HEADING_READING_TIME
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;
  static float average_read_time = 0;
  #endif //DEBUG_HEADING_READING_TIME

  #ifdef HCO_BOARD // read + and - ends of pot with grounded wiper
  //measured 3.3v rail 3.29V
  //static const float Vmax = 3.3 * ROTOR_POT / (HCO_BOARD_RESISTOR + ROTOR_POT); // 1.98 max voltage expected
  // TODO adc read 653, but calculation says 616

  #define ROTOR_POT 823.0
  #define HCO_BOARD_RESISTOR 330.0

  static const int ADCmax = (int) (1023 * ROTOR_POT / (HCO_BOARD_RESISTOR + ROTOR_POT));  //  616 max ADC reading expected

  int   Az_adc_top = analogRead(PositionPosPin); // adc reading for top    of azimuth pot
  int   Az_adc_bot = analogRead(PositionNegPin); // adc reading for bottom of azimuth pot
  
  float Az_top     = 99.0; // init to uninitialized flag value
  float Az_bot     = 99.0;
  float Az_avg     = 99.0;

  // test for open wiper using fix point techniques
  if( (Az_adc_top < (ADCmax + (ADCmax >> 2))) | (Az_adc_bot < (ADCmax + (ADCmax >> 2)))) 
  {
    // convert top and bottom ADC reading to azimuth
    Az_top = (360 * Az_adc_top)            / ADCmax;
    Az_bot = (360 * (ADCmax - Az_adc_bot)) / ADCmax;

    Az_avg = (Az_top + Az_bot) / 2.0; //average the two readings, probably not that helpful
    analog_az = Az_avg;  // global for other
    previous_analog_az = Az_avg; // remember the az in case wiper glitches
  } else // wiper has gone intermittent, skip update
  {
    Az_top = 99.0; // flag variables not used
    Az_bot = 99.0;
    Az_avg = previous_analog_az; // wiper glitched, use previous az
    analog_az = Az_avg;
  }

  // map(value, fromLow, fromHigh, toLow, toHigh)
  
  //raw_azimuth = map(Az_avg, 0, 360, 0, 360);
  float ADC_ccw       = configuration.analog_az_full_ccw;
  float ADC_cw        = configuration.analog_az_full_cw;
  float Az_start      = configuration.azimuth_starting_point * HEADING_MULTIPLIER;
  float Az_capability = configuration.azimuth_rotation_capability;
  float Az_stop       = Az_start + Az_capability * HEADING_MULTIPLIER;

  raw_azimuth = map( Az_avg, ADC_ccw, ADC_cw, Az_start, Az_stop);

  #if 0 //#ifdef DEBUG_HCO_BOARD
  Serial.print("read_az: config, ccw, cw, start, capabilty, stop ");
  Serial.print(ADC_ccw);
  Serial.print(", ");
  Serial.print(ADC_cw);
  Serial.print(", ");
  Serial.print(Az_start);
  Serial.print(", ");
  Serial.print(Az_capability);
  Serial.print(", ");
  Serial.print(Az_stop);
  Serial.println("");
  #endif

    if (AZIMUTH_SMOOTHING_FACTOR > 0) 
    {
      raw_azimuth = (raw_azimuth * (1 - (AZIMUTH_SMOOTHING_FACTOR / 100))) + (previous_raw_azimuth * (AZIMUTH_SMOOTHING_FACTOR / 100));
    }  

    // wrap raw azimuth into azimuth
    azimuth = (int) raw_azimuth;
    if (azimuth >= 360) 
    {
      azimuth -= 360;
    } else if (raw_azimuth < 0) 
    {
      azimuth += 360;
    }

    #ifdef DEBUG_HCO_BOARD
    {
      static bool isFirstWrite = true;
      float Time_usec;
      // format for csv file with header
      if (isFirstWrite)
      {
        Serial.println("HCO Board analog");
        Serial.println("Time, adcTop, adcBot, azTop, azBot, raw, az "); // header line
        isFirstWrite = false;
      }
      Time_usec = micros();;
      Serial.print(Time_usec);
      Serial.print(", ");
      Serial.print(Az_adc_top);
      Serial.print(", ");
      Serial.print(Az_adc_bot);
      Serial.print(", ");
      Serial.print(Az_top);
      Serial.print(", ");
      Serial.print(Az_bot);
      Serial.print(", ");
      Serial.print(raw_azimuth);
      Serial.print(", ");
      Serial.print(azimuth);

      Serial.println("");
    }
    #endif // ifdef debug HCO

#endif

    #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
    // read analog input and convert it to degrees; this gets funky because of 450 degree rotation
    // Bearings:  180-------359-0--------270
    // Voltage:    0----------------------5
    // ADC:        0--------------------1023

    ADC_az = analogRead(rotator_analog_az);

    // map(value, fromLow, fromHigh, toLow, toHigh)
    raw_azimuth = (map(  ADC_az,
    		                 configuration.analog_az_full_ccw,
                         configuration.analog_az_full_cw,
                       ( configuration.azimuth_starting_point * HEADING_MULTIPLIER),
                       ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability) * HEADING_MULTIPLIER)));
    
    #ifdef FEATURE_AZIMUTH_CORRECTION
    raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_AZIMUTH_CORRECTION

    
    if (AZIMUTH_SMOOTHING_FACTOR > 0) 
    {
      raw_azimuth = (raw_azimuth*(1-(AZIMUTH_SMOOTHING_FACTOR/100))) + (previous_raw_azimuth*(AZIMUTH_SMOOTHING_FACTOR/100));
    }  
    if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) 
    {
      azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
      if (azimuth >= (360 * HEADING_MULTIPLIER)) 
      {
        azimuth = azimuth - (360 * HEADING_MULTIPLIER);
      }
    } else 
    {
      if (raw_azimuth < 0) 
      {
        azimuth = raw_azimuth + (360 * HEADING_MULTIPLIER);
      } else 
      {
        azimuth = raw_azimuth;
      }
    }
    #endif //FEATURE_AZ_POSITION_POTENTIOMETER
    
    #ifdef FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
    static unsigned long last_remote_unit_az_query_time = 0;
    
    // do we have a command result waiting for us?
    if (remote_unit_command_results_available == REMOTE_UNIT_AZ_COMMAND) 
    {
      #ifdef DEBUG_HEADING_READING_TIME
      average_read_time = (average_read_time + (millis()-last_time))/2.0;
      last_time = millis();
    
      if (debug_mode)
      {
        if ((millis()-last_print_time) > 1000)
        {
          Serial.print(F("read_azimuth: avg read frequency: "));
          Serial.println(average_read_time,2);
         last_print_time = millis();
        }
      }
      #endif //DEBUG_HEADING_READING_TIME
      raw_azimuth = remote_unit_command_result_float * HEADING_MULTIPLIER;
      
      #ifdef FEATURE_AZIMUTH_CORRECTION
      raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
      #endif //FEATURE_AZIMUTH_CORRECTION      
      
      if (AZIMUTH_SMOOTHING_FACTOR > 0) 
      {
        raw_azimuth = (raw_azimuth*(1-(AZIMUTH_SMOOTHING_FACTOR/100))) + (previous_raw_azimuth*(AZIMUTH_SMOOTHING_FACTOR/100));
      }      
      if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) 
      {
        azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
        if (azimuth >= (360 * HEADING_MULTIPLIER)) 
        {
          azimuth = azimuth - (360 * HEADING_MULTIPLIER);
        }
      } else 
      {
        if (raw_azimuth < 0) 
        {
          azimuth = raw_azimuth + (360 * HEADING_MULTIPLIER);
        } else 
        {
          azimuth = raw_azimuth;
        }
      }    
      remote_unit_command_results_available = 0;  
    } else 
    {
      // is it time to request the azimuth?
      if ((millis() - last_remote_unit_az_query_time) > AZ_REMOTE_UNIT_QUERY_TIME_MS)
      {
        if (submit_remote_command(REMOTE_UNIT_AZ_COMMAND)) 
        {
          last_remote_unit_az_query_time = millis();
        }
      }
    }
    #endif //FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
  
    #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
    static byte az_position_encoder_state = 0;
    
    az_position_encoder_state = ttable[az_position_encoder_state & 0xf][((digitalRead(az_rotary_position_pin2) << 1) | digitalRead(az_rotary_position_pin1))];
    byte az_position_encoder_result = az_position_encoder_state & 0x30;
    if (az_position_encoder_result) 
    {
      if (az_position_encoder_result == DIR_CW) 
      {
        configuration.last_azimuth = configuration.last_azimuth + AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
        #ifdef DEBUG_POSITION_ROTARY_ENCODER
        if (debug_mode){Serial.println(F("read_azimuth: AZ_POSITION_ROTARY_ENCODER: CW"));}
        #endif //DEBUG_POSITION_ROTARY_ENCODER
      }
      if (az_position_encoder_result == DIR_CCW) 
      {
        configuration.last_azimuth = configuration.last_azimuth - AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
        #ifdef DEBUG_POSITION_ROTARY_ENCODER
        if (debug_mode){Serial.println(F("read_azimuth: AZ_POSITION_ROTARY_ENCODER: CCW"));}   
        #endif //DEBUG_POSITION_ROTARY_ENCODER   
      }
      
      #ifdef OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT
      if (configuration.last_azimuth < configuration.azimuth_starting_point)
      {
        configuration.last_azimuth = configuration.azimuth_starting_point;
      }
      if (configuration.last_azimuth > (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))
      {
        configuration.last_azimuth = (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability);
      }    
      #else
      if (configuration.last_azimuth < 0)
      {
        configuration.last_azimuth += 360;
      }
      if (configuration.last_azimuth >= 360)
      {
        configuration.last_azimuth -= 360;
      }       
      #endif //OPTION_AZ_POSITION_ROTARY_ENCODER_HARD_LIMIT
      
      
      raw_azimuth = int(configuration.last_azimuth * HEADING_MULTIPLIER);
      
      #ifdef FEATURE_AZIMUTH_CORRECTION
      raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
      #endif //FEATURE_AZIMUTH_CORRECTION    
      
      if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) 
      {
        azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
      } else 
      {
        azimuth = raw_azimuth;
      }
      configuration_dirty = 1;
    }
    #endif //FEATURE_AZ_POSITION_ROTARY_ENCODER
    
    #ifdef FEATURE_AZ_POSITION_HMC5883L // compass magnetometer
    MagnetometerScaled scaled = compass.ReadScaledAxis(); //scaled values from compass.
    float heading = atan2(scaled.YAxis, scaled.XAxis);
    //  heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0) heading += 2*PI;
    if(heading > 2*PI) heading -= 2*PI;
    raw_azimuth = (heading * RAD_TO_DEG) * HEADING_MULTIPLIER; //radians to degree
    if (AZIMUTH_SMOOTHING_FACTOR > 0) {
      raw_azimuth = (raw_azimuth*(1-(AZIMUTH_SMOOTHING_FACTOR/100))) + (previous_raw_azimuth*(AZIMUTH_SMOOTHING_FACTOR/100));
    }    
    #ifdef FEATURE_AZIMUTH_CORRECTION
    raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_AZIMUTH_CORRECTION
    azimuth = raw_azimuth;
    #endif // FEATURE_AZ_POSITION_HMC5883L

    #ifdef FEATURE_AZ_POSITION_LSM303 // compass magnetometer and accelerometer
    lsm.read();
    float heading = atan2(lsm.magData.y,lsm.magData.x);
    //  heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0) heading += 2*PI;
    if(heading > 2*PI) heading -= 2*PI;
    raw_azimuth = (heading * RAD_TO_DEG) * HEADING_MULTIPLIER; //radians to degree
    if (AZIMUTH_SMOOTHING_FACTOR > 0) 
    {
      raw_azimuth = (raw_azimuth*(1-(AZIMUTH_SMOOTHING_FACTOR/100))) + (previous_raw_azimuth*(AZIMUTH_SMOOTHING_FACTOR/100));
    }    
    #ifdef FEATURE_AZIMUTH_CORRECTION
    raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_AZIMUTH_CORRECTION
    azimuth = raw_azimuth;
    #endif // FEATURE_AZ_POSITION_LSM303
    
    #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
    #ifdef DEBUG_POSITION_PULSE_INPUT
//    if (az_position_pule_interrupt_handler_flag) {
//      Serial.print(F("read_azimuth: az_position_pusle_interrupt_handler_flag: "));
//      Serial.println(az_position_pule_interrupt_handler_flag);
//      az_position_pule_interrupt_handler_flag = 0;
//    }
    #endif //DEBUG_POSITION_PULSE_INPUT
    
    //dddd
    
    static float last_az_position_pulse_input_azimuth = az_position_pulse_input_azimuth;
    
    if (az_position_pulse_input_azimuth != last_az_position_pulse_input_azimuth)
    {
        #ifdef DEBUG_POSITION_PULSE_INPUT
//        if (debug_mode)
//        {
//          Serial.print(F("read_azimuth: last_az_position_pulse_input_azimuth:"));
//          Serial.print(last_az_position_pulse_input_azimuth);
//          Serial.print(F(" az_position_pulse_input_azimuth:"));
//          Serial.print(az_position_pulse_input_azimuth);
//          Serial.print(F(" az_pulse_counter:"));
//          Serial.println(az_pulse_counter);
//        }   
        #endif //DEBUG_POSITION_PULSE_INPUT     
       configuration.last_azimuth = az_position_pulse_input_azimuth;
       configuration_dirty = 1;
       last_az_position_pulse_input_azimuth = az_position_pulse_input_azimuth;
       raw_azimuth = int(configuration.last_azimuth * HEADING_MULTIPLIER);
       #ifdef FEATURE_AZIMUTH_CORRECTION
       raw_azimuth = (correct_azimuth(raw_azimuth/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
       #endif //FEATURE_AZIMUTH_CORRECTION     
       if (raw_azimuth >= (360 * HEADING_MULTIPLIER)) 
       {
         azimuth = raw_azimuth - (360 * HEADING_MULTIPLIER);
       } else 
       {
        azimuth = raw_azimuth;
       }    
    }
    #endif //FEATURE_AZ_POSITION_PULSE_INPUT
  #endif // ifdef azimuth interrupt
} // ReadAzimuthISR()
 
/* ------------------ let's start doing some stuff now that we got the formalities out of the way --------------------*/
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

  #ifdef FEATURE_MAX6959_DISPLAY
  initialize_MAX6959_display();
  delay(2000);
  #endif
  
  initialize_rotary_encoders(); 
  initialize_interrupts();

  // setup the timer and start it
  // timer used to read values and run state machine
  int TimeBetweenInterrupts = 2; // msec
  MsTimer2::set(TimeBetweenInterrupts, TimedService); // 2ms period
  // interrupts enabled after this point
  MsTimer2::start(); 
}

/*-------------------------- here's where the magic happens --------------------------------*/
void loop() 
{ 
  check_serial();
  read_headings(); // read az and el if configured
  #ifndef FEATURE_REMOTE_UNIT_SLAVE
  service_request_queue();
  service_rotation_azimuth();
  az_check_operation_timeout();
  #ifdef FEATURE_TIMED_BUFFER
  check_timed_interval();
  #endif //FEATURE_TIMED_BUFFER
  read_headings();
  check_buttons();
  check_overlap();
  check_brake_release();  // manages the timing of pending brake operations
  #ifdef FEATURE_ELEVATION_CONTROL
  el_check_operation_timeout();
  #endif
  #endif // ifndef FEATURE_REMOTE_UNIT_SLAVE

  read_headings();

  #ifdef FEATURE_LCD_DISPLAY
  update_display(); // Azimuth display with the large fonts
  #endif

  #ifdef FEATURE_MAX6959_DISPLAY
  update_MAX6959_display();
  #endif

  read_headings();
  
  #ifndef FEATURE_REMOTE_UNIT_SLAVE
  #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
  check_az_manual_rotate_limit();
  #endif

  #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
  check_el_manual_rotate_limit();
  #endif
 
  #ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
  check_az_speed_pot();
  #endif
  
  #ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder or Preset Selector
  check_preset_encoders();
  #else
  check_az_preset_potentiometer();
  #endif //FEATURE_AZ_PRESET_ENCODER
  #endif //ndef FEATURE_REMOTE_UNIT_SLAVE
  
  //output_debug();
  
  check_for_dirty_configuration();
  
  read_headings();
  
  profile_loop_time();
  
  #ifdef FEATURE_REMOTE_UNIT_SLAVE 
  service_remote_unit_serial_buffer();
  #endif //FEATURE_REMOTE_UNIT_SLAVE
  
  // TODO, if defined call the function, but the function is not defined in Eclipse. even though it has the same ifdef
  #ifdef FEATURE_HOST_REMOTE_PROTOCOL
  //service_remote_communications_incoming_serial_buffer();
  #endif //FEATURE_HOST_REMOTE_PROTOCOL

  #ifdef FEATURE_JOYSTICK_CONTROL
  check_joystick();
  #endif //FEATURE_JOYSTICK_CONTROL

  #ifdef FEATURE_ROTATION_INDICATOR_PIN
  service_rotation_indicator_pin();
  #endif //FEATURE_ROTATION_INDICATOR_PIN  
  
  service_blink_led();
}

/* -------------------------------------- subroutines -----------------------------------------------
   Where the real work happens...
*/

void read_headings()
{
  read_azimuth();
  
  #ifdef FEATURE_ELEVATION_CONTROL
  read_elevation();
  #endif 
}

//--------------------------------------------------------------
void profile_loop_time()
{
  #ifdef DEBUG_PROFILE_LOOP_TIME 
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;

  average_loop_time = (average_loop_time + (millis()-last_time))/2.0;
  last_time = millis();
    
  if (debug_mode)
  {
    if ((millis()-last_print_time) > 1000)
    {
      Serial.print(F("avg loop time: "));
      Serial.println(average_loop_time,2);
      last_print_time = millis();
    }
  }
  #endif //DEBUG_PROFILE_LOOP_TIME  
}

//--------------------------------------------------------------
void check_az_speed_pot() 
{
#ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
  static unsigned long last_pot_check_time = 0;
  int pot_read = 0;
  byte new_azimuth_speed_voltage = 0;
   
  if (az_speed_pot && azimuth_speed_voltage && ((millis() - last_pot_check_time) > 500))
  {
    pot_read = analogRead(az_speed_pot);
    new_azimuth_speed_voltage = map(pot_read, SPEED_POT_LOW, SPEED_POT_HIGH, SPEED_POT_LOW_MAP, SPEED_POT_HIGH_MAP);
    if (new_azimuth_speed_voltage != normal_az_speed_voltage)
    {
      #ifdef DEBUG_AZ_SPEED_POT
      if (debug_mode)
      {
        Serial.print(F("check_az_speed_pot: normal_az_speed_voltage: "));
        Serial.print(normal_az_speed_voltage);
        Serial.print(F(" new_azimuth_speed_voltage:"));
        Serial.println(new_azimuth_speed_voltage);
      } 
      #endif //DEBUG_AZ_SPEED_POT     
      //analogWrite(azimuth_speed_voltage, new_azimuth_speed_voltage);
      normal_az_speed_voltage = new_azimuth_speed_voltage; 
      update_az_variable_outputs(normal_az_speed_voltage);
      #if defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED) && defined(FEATURE_ELEVATION_CONTROL)
      normal_el_speed_voltage = new_azimuth_speed_voltage; 
      update_el_variable_outputs(normal_el_speed_voltage);
      #endif //OPTION_EL_SPEED_FOLLOWS_AZ_SPEED
    }
    last_pot_check_time = millis();  
  }
#endif
}

//--------------------------------------------------------------
// read the preset pot if button or time
// deal with first indication of change and as pot continues to change
void check_az_preset_potentiometer()
{
  byte check_pot_flag = 0;
  static unsigned long last_pot_check_time = 0;
  static int last_pot_read = 9999;
  int pot_read = 0;
  int new_pot_azimuth = 0;
  byte button_read = 0;
  static byte pot_change_flag = 0;

  if (az_preset_pot) // if az preset pot pin defined
  {  
	// initialize last_pot_read the first time we hit this subroutine
    if (last_pot_read == 9999) last_pot_read = analogRead(az_preset_pot);

    // the preset pot can be stopped or in motion
    if (!pot_change_flag) // if not a preset move in progress
    {
      // two ways to decide to check the azimuth pot, button or time
      if (preset_start_button) // if we have a preset start button, check it
      { 
        button_read = digitalRead(preset_start_button);
        if (button_read == LOW) {check_pot_flag = 1;}
      } else // if not, check the pot every 250 mS
      {  
        if ((millis() - last_pot_check_time) < 250) {check_pot_flag = 1;}        
      }  

      // check the preset pot two ways, with and without change waiting
      if (check_pot_flag) // flag
      {
        check_pot_flag = 0;  
        pot_read = analogRead(az_preset_pot);
        new_pot_azimuth = map(pot_read, 
                              AZ_PRESET_POT_FULL_CW, 
                              AZ_PRESET_POT_FULL_CCW, 
                              AZ_PRESET_POT_FULL_CW_MAP, 
                              AZ_PRESET_POT_FULL_CCW_MAP);

        display_az_preset(new_pot_azimuth);
                              
        // if significant change in preset pot and significantly different from azimuth
        if ((abs(last_pot_read - pot_read) > 4) && 
        	(abs(new_pot_azimuth - (raw_azimuth/HEADING_MULTIPLIER)) > AZIMUTH_TOLERANCE)) 
        {
          pot_change_flag = 1;
          if (debug_mode) 
          {
            Serial.println(F("check_az_preset_potentiometer: in pot_changed_waiting"));
          }
          last_pot_read = pot_read;
        }              
      }
      last_pot_check_time = millis();
    } else // preset pot is moving
    {  
      pot_read = analogRead(az_preset_pot);
      // measure preset pot motion as change in value per read
      if (abs(pot_read - last_pot_read) > 3) // preset pot is moving
      {  
        last_pot_check_time = millis();
        last_pot_read = pot_read;
      } else // preset pot has stopped moving,
      { 
        // wait an additional time after preset pot has stopped moving
    	if ((millis() - last_pot_check_time) >= 250) // has it been a while since the last pot change?
        {  
		  new_pot_azimuth = map(pot_read,
								AZ_PRESET_POT_FULL_CW,
								AZ_PRESET_POT_FULL_CCW,
								AZ_PRESET_POT_FULL_CW_MAP,
								AZ_PRESET_POT_FULL_CCW_MAP);
	      display_az_preset(new_pot_azimuth);

		  submit_request(AZ, REQUEST_AZIMUTH_RAW, new_pot_azimuth*HEADING_MULTIPLIER);
		  pot_change_flag = 0;
		  last_pot_read = pot_read;
		  last_pot_check_time = millis();

		  #ifdef DEBUG_AZ_PRESET_POT
		  if (debug_mode)
		  {
		    Serial.print(F("check_az_preset_potentiometer: pot change - current raw_azimuth: "));
		    Serial.print(raw_azimuth/HEADING_MULTIPLIER);
		    Serial.print(F(" new_azimuth: "));
		    Serial.println(new_pot_azimuth);
		  }
		  #endif //DEBUG_AZ_PRESET_POT
        }
      }
    }    
  } //if (az_preset_pot)
}

//--------------------------------------------------------------
void initialize_rotary_encoders()
{
  #ifdef FEATURE_AZ_PRESET_ENCODER
  pinMode(az_rotary_preset_pin1, INPUT);
  pinMode(az_rotary_preset_pin2, INPUT);
  az_encoder_raw_degrees = raw_azimuth;
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWrite(az_rotary_preset_pin1, HIGH);
  digitalWrite(az_rotary_preset_pin2, HIGH);
  #endif //OPTION_ENCODER_ENABLE_PULLUPS
  #endif //FEATURE_AZ_PRESET_ENCODER
  
  #ifdef FEATURE_EL_PRESET_ENCODER
  pinMode(el_rotary_preset_pin1, INPUT);
  pinMode(el_rotary_preset_pin2, INPUT); 
  el_encoder_degrees = elevation;
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWrite(el_rotary_preset_pin1, HIGH);
  digitalWrite(el_rotary_preset_pin2, HIGH);  
  #endif //OPTION_ENCODER_ENABLE_PULLUPS
  #endif //FEATURE_EL_PRESET_ENCODER
  
  #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
  pinMode(az_rotary_position_pin1, INPUT);
  pinMode(az_rotary_position_pin2, INPUT);
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWrite(az_rotary_position_pin1, HIGH);
  digitalWrite(az_rotary_position_pin2, HIGH);
  #endif //OPTION_ENCODER_ENABLE_PULLUPS
  #endif //FEATURE_AZ_POSITION_ROTARY_ENCODER
  
  #ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
  pinMode(el_rotary_position_pin1, INPUT);
  pinMode(el_rotary_position_pin2, INPUT); 
  #ifdef OPTION_ENCODER_ENABLE_PULLUPS
  digitalWrite(el_rotary_position_pin1, HIGH);
  digitalWrite(el_rotary_position_pin2, HIGH);  
  #endif //OPTION_ENCODER_ENABLE_PULLUPS
  #endif //FEATURE_EL_POSITION_ROTARY_ENCODER    
}

//--------------------------------------------------------------
void check_preset_encoders()
{
  #ifdef FEATURE_AZ_PRESET_ENCODER
  static unsigned long last_encoder_change_time = 0;
  byte button_read = 0;
  byte number_columns = 0;
  static byte submit_encoder_change = 0;
  static unsigned long last_preset_start_button_start = 0;
  static unsigned long last_preset_start_button_kill = 0;
  static unsigned long last_encoder_move = 0;
  
  #ifdef FEATURE_AZ_PRESET_ENCODER
  static unsigned long az_timestamp[5];
  #endif //FEATURE_AZ_PRESET_ENCODER
  
  #ifdef FEATURE_EL_PRESET_ENCODER
  static unsigned long el_timestamp[5];
  #endif //FEATURE_EL_PRESET_ENCODER

  #ifdef FEATURE_AZ_PRESET_ENCODER
  az_encoder_state = ttable[az_encoder_state & 0xf][((digitalRead(az_rotary_preset_pin2) << 1) | digitalRead(az_rotary_preset_pin1))];
  unsigned char az_encoder_result = az_encoder_state & 0x30; 
  #endif //FEATURE_AZ_PRESET_ENCODER
  
  #ifdef FEATURE_EL_PRESET_ENCODER
  el_encoder_state = ttable[el_encoder_state & 0xf][((digitalRead(el_rotary_preset_pin2) << 1) | digitalRead(el_rotary_preset_pin1))];
  unsigned char el_encoder_result = el_encoder_state & 0x30;  
  #endif //FEATURE_EL_PRESET_ENCODER
 
  #ifdef FEATURE_AZ_PRESET_ENCODER
  if (az_encoder_result) // If rotary encoder modified 
  {                                     
    az_timestamp[0] = az_timestamp[1];   // Encoder step timer
    az_timestamp[1] = az_timestamp[2]; 
    az_timestamp[2] = az_timestamp[3]; 
    az_timestamp[3] = az_timestamp[4]; 
    az_timestamp[4] = millis();
    
    last_encoder_move = millis();
    
    unsigned long az_elapsed_time = (az_timestamp[4] - az_timestamp[0]); // Encoder step time difference for 10's step

    #ifdef OPTION_PRESET_ENCODER_RELATIVE_CHANGE
    if ((preset_encoders_state == ENCODER_IDLE) || (preset_encoders_state == ENCODER_EL_PENDING)) 
    {
      if (az_request_queue_state == IN_PROGRESS_TO_TARGET) 
      {
        az_encoder_raw_degrees = target_raw_azimuth;
      } else 
      {
        az_encoder_raw_degrees = raw_azimuth;
      }
    }
    #endif //OPTION_PRESET_ENCODER_RELATIVE_CHANGE
 
    if (az_encoder_result == DIR_CW) 
    {                 
      if (az_elapsed_time < 250 /* mSec */) {az_encoder_raw_degrees += (5*HEADING_MULTIPLIER);} else {az_encoder_raw_degrees += (1*HEADING_MULTIPLIER);};  // Single deg increase unless encoder turned quickly then 10's step         
      //if (az_encoder_raw_degrees >=(360*HEADING_MULTIPLIER)) {az_encoder_raw_degrees -= (360*HEADING_MULTIPLIER);};                    
      if (az_encoder_raw_degrees >((configuration.azimuth_starting_point+configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER)) 
      {
        az_encoder_raw_degrees = ((configuration.azimuth_starting_point*HEADING_MULTIPLIER)
           /* + ((configuration.azimuth_starting_point+configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER) - az_encoder_raw_degrees*/);
      }                                    
    }
    if (az_encoder_result == DIR_CCW) 
    {                     
      if (az_elapsed_time < 250 /* mSec */) {az_encoder_raw_degrees -= (5*HEADING_MULTIPLIER);} else {az_encoder_raw_degrees -= (1*HEADING_MULTIPLIER);};   // Single deg decrease unless encoder turned quickly then 10's step
      //if (az_encoder_raw_degrees < 0) {az_encoder_raw_degrees = (360*HEADING_MULTIPLIER);};                                
      if (az_encoder_raw_degrees < (configuration.azimuth_starting_point*HEADING_MULTIPLIER)) 
      {
        az_encoder_raw_degrees = (((configuration.azimuth_starting_point+configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER)
          /*- (az_encoder_raw_degrees-(configuration.azimuth_starting_point*HEADING_MULTIPLIER))*/);
      }                                         
    }
    last_encoder_change_time = millis();              // Encoder Check Timer

    #ifdef FEATURE_LCD_DISPLAY
    push_lcd_update = 1;                     // push an LCD update
    #endif //FEATURE_LCD_DISPLAY
    
    if (preset_encoders_state == ENCODER_IDLE) 
    {
      preset_encoders_state = ENCODER_AZ_PENDING;
    } else 
    {
      if (preset_encoders_state == ENCODER_EL_PENDING) 
      {
        preset_encoders_state = ENCODER_AZ_EL_PENDING;
      }
    }
    
    #ifdef DEBUG_PRESET_ENCODERS
    if (debug_mode) 
    {
      Serial.print(F("check_preset_encoders: az target: "));
      Serial.println(az_encoder_raw_degrees/HEADING_MULTIPLIER,1);
    }
    #endif //DEBUG_PRESET_ENCODERS
  #endif //FEATURE_AZ_PRESET_ENCODER
    
  } // if (az_encoder_result)
  
  #ifdef FEATURE_EL_PRESET_ENCODER
  
  #ifdef OPTION_PRESET_ENCODER_RELATIVE_CHANGE
  if ((preset_encoders_state == ENCODER_IDLE) || (preset_encoders_state == ENCODER_AZ_PENDING)) 
  {
    if (el_request_queue_state == IN_PROGRESS_TO_TARGET) 
    {
      el_encoder_degrees = target_elevation;
    } else 
    {
      el_encoder_degrees = elevation;
    }
  }
  #endif //OPTION_PRESET_ENCODER_RELATIVE_CHANGE  
  
  if (el_encoder_result) // If rotary encoder modified
  {                                      
    el_timestamp[0] = el_timestamp[1];  // Encoder step timer
    el_timestamp[1] = el_timestamp[2]; 
    el_timestamp[2] = el_timestamp[3]; 
    el_timestamp[3] = el_timestamp[4]; 
    el_timestamp[4] = millis();
    
    last_encoder_move = millis();
    
    unsigned long el_elapsed_time = (el_timestamp[4] - el_timestamp[0]); // Encoder step time difference for 10's step
 
    if (el_encoder_result == DIR_CW) // Rotary Encoder CW 0 - 359 Deg
    {                      
      
      if (el_elapsed_time < 250) {el_encoder_degrees += (5*HEADING_MULTIPLIER);} else {el_encoder_degrees += (1*HEADING_MULTIPLIER);};  // Single deg increase unless encoder turned quickly then 10's step
      if (el_encoder_degrees > (180*HEADING_MULTIPLIER)) {el_encoder_degrees = (180*HEADING_MULTIPLIER);};                                    
    }
    if (el_encoder_result == DIR_CCW) // Rotary Encoder CCW 359 - 0 Deg
    {                      
      if (el_elapsed_time < 250) {el_encoder_degrees -= (5*HEADING_MULTIPLIER);} else {el_encoder_degrees -= (1*HEADING_MULTIPLIER);};   // Single deg decrease unless encoder turned quickly then 10's step
      if (el_encoder_degrees < 0) {el_encoder_degrees = 0;};                                            
    }
    last_encoder_change_time = millis();              // Encoder Check Timer

    if (preset_encoders_state == ENCODER_IDLE) 
    {
      preset_encoders_state = ENCODER_EL_PENDING;
    } else 
    {
      if (preset_encoders_state == ENCODER_AZ_PENDING) 
      {
        preset_encoders_state = ENCODER_AZ_EL_PENDING;
      }
    }
    
    #ifdef DEBUG_PRESET_ENCODERS
    if (debug_mode) 
    {
      Serial.print(F("check_preset_encoders: el target: "));
      Serial.println(el_encoder_degrees/HEADING_MULTIPLIER,1);
    }
    #endif //DEBUG_PRESET_ENCODERS

  } // if (el_encoder_result)    
  
  #endif //FEATURE_EL_PRESET_ENCODER

  if ((preset_encoders_state != ENCODER_IDLE) && (!submit_encoder_change)) 
  {                           // Check button or timer
    if (preset_start_button) // if we have a preset start button, check it
    {
      button_read = digitalRead(preset_start_button);
      if (button_read == LOW) 
      {
        submit_encoder_change = 1;
        last_preset_start_button_start = millis();
      }
    } else 
    {  
     if ((millis() - last_encoder_change_time) > 2000) {submit_encoder_change = 1;}        //if enc not changed for more than 2 sec, rotate to target       
    }  
  } //if (!enc_changed_waiting) 

  if (preset_start_button) // if we have a preset start button, check it
  {                                         
    button_read = digitalRead(preset_start_button);   
    if ((button_read == LOW) && (!submit_encoder_change) && ((millis() - last_preset_start_button_start) > 250) 
    && ((millis() - last_preset_start_button_kill) > 250) && (preset_encoders_state == ENCODER_IDLE)) 
    {
      #ifdef DEBUG_PRESET_ENCODERS
      if (debug_mode) 
      {
        Serial.println(F("check_preset_encoders: preset button kill"));
      }
      #endif //DEBUG_PRESET_ENCODERS   
      #ifdef FEATURE_AZ_PRESET_ENCODER 
      if (az_state != IDLE) 
      {
        submit_request(AZ,REQUEST_KILL,0); 
      }
      #endif //FEATURE_AZ_PRESET_ENCODER
      #ifdef FEATURE_EL_PRESET_ENCODER
      if (el_state != IDLE) 
      {
        submit_request(EL,REQUEST_KILL,0); 
      }
      #endif //FEATURE_EL_PRESET_ENCODER
      last_preset_start_button_kill = millis();
    }
  }
    
  if ((submit_encoder_change) && (button_read == HIGH)) 
  {
    #ifdef DEBUG_PRESET_ENCODERS
    if (debug_mode) 
    {
      Serial.println(F("check_preset_encoders: submit_encoder_change "));
    }
    #endif //DEBUG_PRESET_ENCODERS

    if ((preset_encoders_state == ENCODER_AZ_PENDING) || (preset_encoders_state == ENCODER_AZ_EL_PENDING)) 
    {
      submit_request(AZ,REQUEST_AZIMUTH_RAW,az_encoder_raw_degrees);
    }
    
    #ifdef FEATURE_EL_PRESET_ENCODER
    if ((preset_encoders_state == ENCODER_EL_PENDING) || (preset_encoders_state == ENCODER_AZ_EL_PENDING)) 
    {
      submit_request(EL,REQUEST_ELEVATION,el_encoder_degrees);
    }    
    #endif //FEATURE_EL_PRESET_ENCODER
    
    preset_encoders_state = ENCODER_IDLE;
    submit_encoder_change = 0;
  } //if (submit_encoder_change)
  
  if ((preset_start_button) && (preset_encoders_state != ENCODER_IDLE) && ((millis() - last_encoder_move) > ENCODER_PRESET_TIMEOUT)) // timeout if we have a preset start button
  { 
    preset_encoders_state = ENCODER_IDLE;
    #ifdef FEATURE_LCD_DISPLAY
    push_lcd_update = 1;                     // push an LCD update
    #endif //FEATURE_LCD_DISPLAY    
  }
#endif //FEATURE_AZ_PRESET_ENCODER
} 

//--------------------------------------------------------------
#ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
void check_az_manual_rotate_limit() 
{
  if ((current_az_state() == ROTATING_CCW) && (raw_azimuth <= (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_az_manual_rotate_limit: stopping - hit AZ_MANUAL_ROTATE_CCW_LIMIT of "));
      Serial.println(AZ_MANUAL_ROTATE_CCW_LIMIT);
    } 
    #endif //DEBUG_AZ_MANUAL_ROTATE_LIMITS
    submit_request(AZ,REQUEST_KILL,0);       
  }
  if ((current_az_state() == ROTATING_CW) && (raw_azimuth >= (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_az_manual_rotate_limit: stopping - hit AZ_MANUAL_ROTATE_CW_LIMIT of "));
      Serial.println(AZ_MANUAL_ROTATE_CW_LIMIT);
    } 
    #endif //DEBUG_AZ_MANUAL_ROTATE_LIMITS
    submit_request(AZ,REQUEST_KILL,0);   
  }
}
#endif //#ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS

//--------------------------------------------------------------
#if defined(OPTION_EL_MANUAL_ROTATE_LIMITS) && defined(FEATURE_ELEVATION_CONTROL)
void check_el_manual_rotate_limit() 
{
  if ((current_el_state() == ROTATING_DOWN) && (elevation <= (EL_MANUAL_ROTATE_DOWN_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_DOWN_LIMIT of "));
      Serial.println(EL_MANUAL_ROTATE_DOWN_LIMIT);
    } 
    #endif //DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL,REQUEST_KILL,0);       
  }
  if ((current_el_state() == ROTATING_UP) && (elevation >= (EL_MANUAL_ROTATE_UP_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_UP_LIMIT of "));
      Serial.println(EL_MANUAL_ROTATE_UP_LIMIT);
    } 
    #endif //DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL,REQUEST_KILL,0);   
  }
}
#endif //#ifdef OPTION_EL_MANUAL_ROTATE_LIMITS


//--------------------------------------------------------------
void check_overlap()
{
  static byte overlap_led_status = 0;
  static unsigned long last_check_time;
  
  if ((overlap_led) && ((millis() - last_check_time) > 500)) 
  {
     //if ((analog_az > (500*HEADING_MULTIPLIER)) && (azimuth > (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (!overlap_led_status)) {
     if ((raw_azimuth > (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (!overlap_led_status)) 
     {
       digitalWrite(overlap_led, HIGH);
       overlap_led_status = 1;
       #ifdef DEBUG_OVERLAP
       if (debug_mode) 
       {
         Serial.print(F("check_overlap: in overlap\r\n"));
       }
       #endif //DEBUG_OVERLAP
     } else 
     {
       //if (((analog_az < (500*HEADING_MULTIPLIER)) || (azimuth < (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER))) && (overlap_led_status)) {
       if ((raw_azimuth < (ANALOG_AZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (overlap_led_status)) 
       {
         digitalWrite(overlap_led, LOW);
         overlap_led_status = 0;
         #ifdef DEBUG_OVERLAP
         if (debug_mode) 
         {
           Serial.print(F("check_overlap: overlap off\r\n"));
         }
         #endif //DEBUG_OVERLAP
       }        
     }
     last_check_time = millis();
  }
}



//--------------------------------------------------------------
#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_ANCILLARY_PIN_CONTROL)
byte get_analog_pin(byte pin_number)
{  
  byte return_output = 0;
           
  switch(pin_number)
  {
    case 0: return_output = A0; break;
    case 1: return_output = A1; break;
    case 2: return_output = A2; break;
    case 3: return_output = A3; break;
    case 4: return_output = A4; break;
    case 5: return_output = A5; break; 
    case 6: return_output = A6; break;    
  }  
  return return_output;
}
#endif //FEATURE_REMOTE_UNIT_SLAVE

//--------------------------------------------------------------
#ifdef FEATURE_REMOTE_UNIT_SLAVE
void remote_unit_serial_command()
{
    if (serial_read_event_flag[0])
    {
      Serial.print("EVS0");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    }
    
    if ((incoming_serial_byte != 10) && (serial0_buffer_index < COMMAND_BUFFER_SIZE))
    {
      //incoming_serial_byte = toupper(incoming_serial_byte);
      serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
      serial0_buffer_index++;
      if ((incoming_serial_byte == 13) || (serial0_buffer_index == COMMAND_BUFFER_SIZE))
      {
        serial0_buffer_carriage_return_flag = 1;
      } 
    }
}
#endif //FEATURE_REMOTE_UNIT_SLAVE 

//--------------------------------------------------------------
#ifdef FEATURE_REMOTE_UNIT_SLAVE  
void service_remote_unit_serial_buffer()
{
/*
  This implements a protocol for host unit to remote unit communications
  Remote Slave Unit Protocol Reference

    PG - ping
    AZ - read azimuth
    EL - read elevation
    DOxx - digital pin initialize as output; 
    DIxx - digital pin initialize as input
    DPxx - digital pin initialize as input with pullup
    DRxx - digital pin read
    DLxx - digital pin write low
    DHxx - digital pin write high
    DTxxyyyy - digital pin tone output
    NTxx - no tone
    ARxx - analog pin read
    AWxxyyy - analog pin write
    SWxy - serial write byte
    SDx - deactivate serial read event; x = port #
    SSxyyyyyy... - serial write sting; x = port #, yyyy = string of characters to send
    SAx - activate serial read event; x = port #
    RB - reboot

  Responses

    ER - report an error (remote to host only)
    EV - report an event (remote to host only)
    OK - report success (remote to host only)
    CS - report a cold start (remote to host only)

  Error Codes

    ER01 - Serial port buffer timeout
    ER02 - Command syntax error

  Events

    EVSxy - Serial port read event; x = serial port number, y = byte returned
*/
  
  String command_string;
  byte command_good = 0;
  
  if (serial0_buffer_carriage_return_flag) 
  {
    
    // TODO: if checksumming is turned on, parse out the checksum, validate it, return an error if invalid, chop it off if valid and continue
    
    if (serial0_buffer_index < 3)
    {
      Serial.println(F("ER02"));  // we don't have enough characters - syntax error
    } else 
    {
      command_string = String(char(toupper(serial0_buffer[0]))) + String(char(toupper(serial0_buffer[1])));
      
      #ifdef DEBUG_SERVICE_SERIAL_BUFFER
      Serial.print(F("serial_serial_buffer: command_string: "));
      Serial.print(command_string);
      Serial.print(F("$ serial0_buffer_index: "));
      Serial.println(serial0_buffer_index);
      #endif //DEBUG_SERVICE_SERIAL_BUFFER
  
      if ((command_string == "SS") && (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
      {  // this is a variable length command
        command_good = 1;
        for (byte x = 3;x < serial0_buffer_index;x++){
          switch(serial0_buffer[2]-48){
            case 0: Serial.write(serial0_buffer[x]); break;
            #ifdef OPTION_SERIAL1_SUPPORT
            case 1: Serial1.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT
            #ifdef OPTION_SERIAL2_SUPPORT
            case 2: Serial2.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT
            #ifdef OPTION_SERIAL3_SUPPORT
            case 3: Serial3.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT            
          }
        }
      }
      if (serial0_buffer_index == 3) 
      {
        if (command_string == "PG") {Serial.println(F("PG"));command_good = 1;}   // PG - ping
        if (command_string == "RB") {wdt_enable(WDTO_30MS); while(1) {}}         // RB - reboot
        if (command_string == "AZ") 
        {
          Serial.print(F("AZ"));
          if (raw_azimuth < 1000) {Serial.print("0");}
          if (raw_azimuth < 100) {Serial.print("0");}
          if (raw_azimuth < 10) {Serial.print("0");}
          Serial.println(raw_azimuth);
          command_good = 1;
        }
        #ifdef FEATURE_ELEVATION_CONTROL
        if (command_string == "EL") 
        {
          Serial.print(F("EL"));
          if (elevation >= 0) 
          {
            Serial.print("+");
          } else 
          {
            Serial.print("-");
          }
          if (abs(elevation) < 1000) {Serial.print("0");}
          if (abs(elevation) < 100) {Serial.print("0");}
          if (abs(elevation) < 10) {Serial.print("0");}
          Serial.println(abs(elevation));
          command_good = 1;      
        }  
        #endif //FEATURE_ELEVATION_CONTROL
      } // end of three byte commands

      if (serial0_buffer_index == 4) 
      {
        if ((command_string == "SA") & (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
        {
          serial_read_event_flag[serial0_buffer[2]-48] = 1;
          command_good = 1;
          Serial.println("OK");
        }
        if ((command_string == "SD") & (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
        {
          serial_read_event_flag[serial0_buffer[2]-48] = 0;
          command_good = 1;
          Serial.println("OK");          
        }          
      }
  
      if (serial0_buffer_index == 5) 
      {
        if (command_string == "SW"){ // Serial Write command
          switch (serial0_buffer[2])
          {
            case '0': Serial.write(serial0_buffer[3]); command_good = 1; break; 
            #ifdef OPTION_SERIAL1_SUPPORT
            case '1': Serial1.write(serial0_buffer[3]); command_good = 1; break;
            #endif //OPTION_SERIAL1_SUPPORT
          } 
        }
       
        if (command_string == "DO")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            #ifdef DEBUG_SERVICE_SERIAL_BUFFER
            Serial.print(F("service_serial_buffer: pin_value: "));
            Serial.println(pin_value);
            #endif //DEBUG_SERVICE_SERIAL_BUFFER
            Serial.println("OK");
            pinMode(pin_value,OUTPUT);
          }        
        }
        
        if (command_string == "DH")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            digitalWrite(pin_value,HIGH);
            Serial.println("OK");
          }         
        }   
       
        if (command_string == "DL")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            digitalWrite(pin_value,LOW);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DI")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            pinMode(pin_value,INPUT);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DP")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            //pinMode(pin_value,INPUT_PULLUP);
            pinMode(pin_value,INPUT);
            digitalWrite(pin_value,HIGH);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DR")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            byte pin_read = digitalRead(pin_value);
            Serial.print("DR");
            Serial.write(serial0_buffer[2]);
            Serial.write(serial0_buffer[3]);
            if (pin_read){
              Serial.println("1");
            } else 
            {
              Serial.println("0");
            }
          }         
        }    
        if (command_string == "AR"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int pin_read = analogRead(pin_value);
            Serial.print("AR");
            Serial.write(serial0_buffer[2]);
            Serial.write(serial0_buffer[3]);
            if (pin_read < 1000) {Serial.print("0");}
            if (pin_read < 100) {Serial.print("0");}
            if (pin_read < 10) {Serial.print("0");}
            Serial.println(pin_read);
          }         
        } 
        
        if (command_string == "NT"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            noTone(pin_value);
            Serial.println("OK");
          }         
        }    
        
      } //if (serial0_buffer_index == 5)
      
      if (serial0_buffer_index == 8) {
        if (command_string == "AW"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*100) + ((serial0_buffer[5]-48)*10) + (serial0_buffer[6]-48);
            if ((write_value >= 0) && (write_value < 256)){
              analogWrite(pin_value,write_value);
              Serial.println("OK");
              command_good = 1;
            }
          }         
        }        
      }

      if (serial0_buffer_index == 9) 
      {
        if (command_string == "DT"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*1000) + ((serial0_buffer[5]-48)*100) + ((serial0_buffer[6]-48)*10) + (serial0_buffer[7]-48);
            if ((write_value >= 0) && (write_value <= 9999))
            {
              tone(pin_value,write_value);
              Serial.println("OK");
              command_good = 1;
            }
          }         
        }        
      }      
      
 
      if (!command_good) {Serial.println(F("ER02"));}
    }
    serial0_buffer_carriage_return_flag = 0;
    serial0_buffer_index = 0; 
  } else 
  {
    if (((millis() - last_serial_receive_time) > REMOTE_BUFFER_TIMEOUT_MS) && serial0_buffer_index)
    {
      Serial.println(F("ER01"));
      serial0_buffer_index = 0; 
    }
  }
}
#endif //FEATURE_REMOTE_UNIT_SLAVE

//--------------------------------------------------------------
// check and act on button presses
void check_buttons()
{
  #if defined(FEATURE_ADAFRUIT_BUTTONS)
  int buttons = 0;
  buttons = readButtons();

  if (buttons & BUTTON_RIGHT) 
  {

  #elif defined(FEATURE_MAX6959_BUTTONS)

  int buttons = 0;
  buttons = read_MAX6959_buttons();
  if (buttons & MAX6959_BUTTON_RIGHT)
  {

  #else // not adafruit or max6959 buttons
  if (button_cw && (digitalRead(button_cw) == LOW)) 
  {
  #endif //FEATURE_ADAFRUIT_BUTTONS

    if (azimuth_button_was_pushed == 0) 
    {
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {Serial.println(F("check_buttons: button_cw pushed"));}       
      #endif //DEBUG_BUTTONS
      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER)) 
      {
      #endif      
      submit_request(AZ,REQUEST_CW,0);
      azimuth_button_was_pushed = 1;
      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      } else 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CW_LIMIT"));}
        #endif //DEBUG_BUTTONS
      }
      #endif            
    }

  } else // not button cw
  {
    #ifdef FEATURE_ADAFRUIT_BUTTONS
    if (buttons & BUTTON_LEFT) 
    {

    #elif defined(FEATURE_MAX6959_BUTTONS)
    int buttons = 0;
    buttons = read_MAX6959_buttons();
    if (buttons & MAX6959_BUTTON_LEFT)
    {

    #else
    if (button_ccw && (digitalRead(button_ccw) == LOW)) 
    {
    #endif //FEATURE_ADAFRUIT_BUTTONS
      if (azimuth_button_was_pushed == 0) 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) 
        {
          Serial.println(F("check_buttons: button_ccw pushed"));
        }         
        #endif //DEBUG_BUTTONS 
        #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
        if (raw_azimuth > (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER)) 
        {
        #endif  
        submit_request(AZ,REQUEST_CCW,0);
        azimuth_button_was_pushed = 1;
        #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
        } else 
        {
          #ifdef DEBUG_BUTTONS
          if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CCW_LIMIT"));}
          #endif //DEBUG_BUTTONS
        }
        #endif //OPTION_AZ_MANUAL_ROTATE_LIMITS      
      }
    }
  }

  #ifdef FEATURE_ADAFRUIT_BUTTONS
  if ((azimuth_button_was_pushed) && (!(buttons & 0x12))) 
  {
    #ifdef DEBUG_BUTTONS
    if (debug_mode) 
    {
      Serial.println(F("check_buttons: no button depressed"));
    }    
    #endif // DEBUG_BUTTONS
    submit_request(AZ,REQUEST_STOP,0);
    azimuth_button_was_pushed = 0;
  }
  
  #else // not adafruit buttons
  if ((azimuth_button_was_pushed) && (digitalRead(button_ccw) == HIGH) && (digitalRead(button_cw) == HIGH)) 
  {
    delay(200); // debouncing
    if ((digitalRead(button_ccw) == HIGH) && (digitalRead(button_cw) == HIGH)) 
    {
        #ifdef DEBUG_BUTTONS
      if (debug_mode) 
      {
        Serial.println(F("check_buttons: no AZ button depressed"));
      }    
      #endif // DEBUG_BUTTONS
      submit_request(AZ, REQUEST_STOP,0);
      azimuth_button_was_pushed = 0;
    }
  }
  #endif //FEATURE_ADAFRUIT_BUTTONS

  #ifdef FEATURE_ELEVATION_CONTROL
  #ifdef FEATURE_ADAFRUIT_BUTTONS
  if (buttons & 0x08) 
  {
  #else
  if (button_up && (digitalRead(button_up) == LOW)) 
  {
  #endif //FEATURE_ADAFRUIT_BUTTONS
    if (elevation_button_was_pushed == 0) 
    {
      submit_request(EL,REQUEST_UP,0);
      elevation_button_was_pushed = 1;
      #ifdef DEBUG_BUTTONS
      if (debug_mode) 
      {
        Serial.println(F("check_buttons: button_up pushed"));
      }      
      #endif //DEBUG_BUTTONS
    }
  } else 
  {
    #ifdef FEATURE_ADAFRUIT_BUTTONS
    if (buttons & 0x04) 
    {
    #else
    if (button_down && (digitalRead(button_down) == LOW)) 
    {
    #endif //FEATURE_ADAFRUIT_BUTTONS
      if (elevation_button_was_pushed == 0) 
      {
        submit_request(EL,REQUEST_DOWN,0);
        elevation_button_was_pushed = 1;
        #ifdef DEBUG_BUTTONS
        if (debug_mode) 
        {
          Serial.println(F("check_buttons: button_down pushed"));
        }
        #endif //DEBUG_BUTTONS        
      }
    }
  }

  #ifdef FEATURE_ADAFRUIT_BUTTONS
  if ((elevation_button_was_pushed) && (!(buttons & 0x0C))) 
  {
    #ifdef DEBUG_BUTTONS
    if (debug_mode) {
      Serial.println(F("check_buttons: no EL button depressed"));
    }    
    #endif // DEBUG_BUTTONS
    submit_request(EL,REQUEST_STOP,0);
    elevation_button_was_pushed = 0;
  }
  
  #else
  if ((elevation_button_was_pushed) && (digitalRead(button_up) == HIGH) && (digitalRead(button_down) == HIGH)) 
  {
    delay(200);
    if ((digitalRead(button_up) == HIGH) && (digitalRead(button_down) == HIGH)) 
    {
    #ifdef DEBUG_BUTTONS
    if (debug_mode)
    {
      Serial.println(F("check_buttons: no EL button depressed"));
    }    
    #endif // DEBUG_BUTTONS
      submit_request(EL,REQUEST_STOP,0);
      elevation_button_was_pushed = 0;
    }
  }
  #endif //FEATURE_ADAFRUIT_BUTTONS

  #endif //FEATURE_ELEVATION_CONTROL

  
  #ifdef FEATURE_PARK
  static byte park_button_pushed = 0;
  static unsigned long last_time_park_button_pushed = 0;
  
  if (button_park){
    if ((digitalRead(button_park) == LOW))
    {
      park_button_pushed = 1;
      last_time_park_button_pushed = millis();
      #ifdef DEBUG_BUTTONS
      if (debug_mode) 
      {
        Serial.println(F("check_buttons: button_park pushed"));
      }
      #endif //DEBUG_BUTTONS      
    } else 
    {
      if ((park_button_pushed) && ((millis() - last_time_park_button_pushed) >= 250))
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) 
        {
          Serial.println(F("check_buttons: executing park"));
        }
        #endif //DEBUG_BUTTONS         
        submit_request(AZ,REQUEST_AZIMUTH_RAW,PARK_AZIMUTH);
        #ifdef FEATURE_ELEVATION_CONTROL
        submit_request(EL,REQUEST_ELEVATION,PARK_ELEVATION);
        #endif // FEATURE_ELEVATION
        park_button_pushed = 0;
      }
    } 
    
  }
  #endif
  
  if (button_stop) 
  {
    if ((digitalRead(button_stop) == LOW)) 
    {
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {Serial.println(F("check_buttons: button_stop pushed"));} 
      #endif //DEBUG_BUTTONS
      submit_request(AZ,REQUEST_STOP,0);
      #ifdef FEATURE_ELEVATION_CONTROL
      submit_request(EL,REQUEST_STOP,0);
      #endif //FEATURE_ELEVATION_CONTROL
    }      
  }  
}

//--------------------------------------------------------------
void read_settings_from_eeprom()
{
  //EEPROM_readAnything(0,configuration);

  byte* p = (byte*)(void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++)
  {
    *p++ = EEPROM.read(ee++);  
  }
  
  if (configuration.magic_number == EEPROM_MAGIC_NUMBER) 
  {   
    #ifdef DEBUG_EEPROM
    if (debug_mode) 
    {
      Serial.print(F("read_settings_from_eeprom: reading settings from eeprom: "));
      Serial.print("analog_az_full_ccw");
      Serial.println(configuration.analog_az_full_ccw,DEC);
      Serial.print("analog_az_full_cw");
      Serial.println(configuration.analog_az_full_cw,DEC);
      Serial.print("analog_el_0_degrees");
      Serial.println(configuration.analog_el_0_degrees,DEC);
      Serial.print("analog_el_max_elevation");
      Serial.println(configuration.analog_el_max_elevation,DEC);
      Serial.print("azimuth_starting_point");
      Serial.println(configuration.azimuth_starting_point,DEC);
      Serial.print("azimuth_rotation_capability");
      Serial.println(configuration.azimuth_rotation_capability,DEC);
      Serial.print("last_azimuth:");
      Serial.println(configuration.last_azimuth,1);
      Serial.print("last_elevation");
      Serial.println(configuration.last_elevation,1);          
    }
    #endif //DEBUG_EEPROM
    
    #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
    raw_azimuth = int(configuration.last_azimuth*HEADING_MULTIPLIER);
    if (raw_azimuth >= (360*HEADING_MULTIPLIER))
    {
      azimuth = raw_azimuth - (360*HEADING_MULTIPLIER);
    } else 
    {
      azimuth = raw_azimuth;
    }
    #endif //FEATURE_AZ_POSITION_ROTARY_ENCODER
    
    #ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
    elevation = int(configuration.last_elevation*HEADING_MULTIPLIER);
    #endif //FEATURE_EL_POSITION_ROTARY_ENCODER
       
    #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
    raw_azimuth = int(configuration.last_azimuth*HEADING_MULTIPLIER);
    if (raw_azimuth >= (360*HEADING_MULTIPLIER))
    {
      azimuth = raw_azimuth - (360*HEADING_MULTIPLIER);
    } else 
    {
      azimuth = raw_azimuth;
    }
    az_position_pulse_input_azimuth = configuration.last_azimuth;
    #endif //FEATURE_AZ_POSITION_PULSE_INPUT    
    
    #ifdef FEATURE_EL_POSITION_PULSE_INPUT
    elevation = int(configuration.last_elevation*HEADING_MULTIPLIER);
    el_position_pulse_input_elevation = configuration.last_elevation;
    #endif //FEATURE_EL_POSITION_PULSE_INPUT    
    
//     #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
//    volatile float az_position_pulse_azimuth = 0;
//    #endif //FEATURE_AZ_POSITION_PULSE_INPUT    a

  } else {  // initialize eeprom with default values
    #ifdef DEBUG_EEPROM
    if (debug_mode) 
    {
      Serial.println(F("read_settings_from_eeprom: uninitialized eeprom, calling initialize_eeprom_with_defaults()"));
    }
    #endif //DEBUG_EEPROM  
    initialize_eeprom_with_defaults();
  }
}

//--------------------------------------------------------------
void az_check_operation_timeout()
{
  // check if the last executed rotation operation has been going on too long

  if (((millis() - az_last_rotate_initiation) > OPERATION_TIMEOUT) && (az_state != IDLE)) 
  {
    submit_request(AZ,REQUEST_KILL,0);
    #ifdef DEBUG_AZ_CHECK_OPERATION_TIMEOUT
    if (debug_mode) {Serial.println(F("az_check_operation_timeout: timeout reached, aborting rotation"));}
    #endif //DEBUG_AZ_CHECK_OPERATION_TIMEOUT
  }
}

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void clear_timed_buffer()
{
  timed_buffer_status = EMPTY;
  timed_buffer_number_entries_loaded = 0;
  timed_buffer_entry_pointer = 0;
}
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void initiate_timed_buffer()
{
  if (timed_buffer_status == LOADED_AZIMUTHS) 
  {
    timed_buffer_status = RUNNING_AZIMUTHS;
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[1]);
    last_timed_buffer_action_time = millis();
    timed_buffer_entry_pointer = 2;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS"));}
    #endif //DEBUG_TIMED_BUFFER
  } else 
  {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (timed_buffer_status == LOADED_AZIMUTHS_ELEVATIONS) 
    {
      timed_buffer_status = RUNNING_AZIMUTHS_ELEVATIONS;
      submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[1]);
      submit_request(EL,REQUEST_ELEVATION,timed_buffer_elevations[1]);
      last_timed_buffer_action_time = millis();
      timed_buffer_entry_pointer = 2;
      #ifdef DEBUG_TIMED_BUFFER
      if (debug_mode) {Serial.println(F("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS_ELEVATIONS"));}
      #endif //DEBUG_TIMED_BUFFER
    } else 
    {
      Serial.println(">");  // error
    }
    #endif
  }
}
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void print_timed_buffer_empty_message()
{  
  #ifdef DEBUG_TIMED_BUFFER
  if (debug_mode) {Serial.println(F("check_timed_interval: completed timed buffer; changing state to EMPTY"));}
  #endif //DEBUG_TIMED_BUFFER
}

#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void check_timed_interval()
{
  if ((timed_buffer_status == RUNNING_AZIMUTHS) && (((millis() - last_timed_buffer_action_time)/1000) > timed_buffer_interval_value_seconds)) 
  {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("check_timed_interval: executing next timed interval step - azimuths"));}
    #endif //DEBUG_TIMED_BUFFER
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[timed_buffer_entry_pointer-1]);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) 
    {
      clear_timed_buffer();
      print_timed_buffer_empty_message();
    }
  }
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((timed_buffer_status == RUNNING_AZIMUTHS_ELEVATIONS) && (((millis() - last_timed_buffer_action_time)/1000) > timed_buffer_interval_value_seconds)) 
  {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("check_timed_interval: executing next timed interval step - az and el"));}
    #endif //DEBUG_TIMED_BUFFER
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[timed_buffer_entry_pointer-1]);
    submit_request(EL,REQUEST_ELEVATION,timed_buffer_elevations[timed_buffer_entry_pointer-1]);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) 
    {
      clear_timed_buffer();
      print_timed_buffer_empty_message();
    }
  }
  #endif
}
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void yaesu_az_load_timed_intervals()
{
  int parsed_value = 0;

  clear_timed_buffer();

  parsed_value = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
  if ((parsed_value > 0) && (parsed_value < 1000)) {
    timed_buffer_interval_value_seconds = parsed_value;
    for (int x = 5; x < serial0_buffer_index; x = x + 4) {
      parsed_value = ((int(serial0_buffer[x])-48)*100) + ((int(serial0_buffer[x+1])-48)*10) + (int(serial0_buffer[x+2])-48);
      if ((parsed_value > -1) && (parsed_value < 361)) 
      {  // is it a valid azimuth?
        timed_buffer_azimuths[timed_buffer_number_entries_loaded] = parsed_value * HEADING_MULTIPLIER;
        timed_buffer_number_entries_loaded++;
        timed_buffer_status = LOADED_AZIMUTHS;
        if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
          submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[0]);  // array is full, go to the first azimuth
          timed_buffer_entry_pointer = 1;
          return;
        }
      } else 
      {   // we hit an invalid bearing
        timed_buffer_status = EMPTY;
        timed_buffer_number_entries_loaded = 0;
        Serial.println(F("?>"));  // error
        return;
      }
    }
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[0]);   // go to the first azimuth
    timed_buffer_entry_pointer = 1;

  } else 
  {
    Serial.println(F("?>"));  // error
  }
}
#endif //FEATURE_TIMED_BUFFER


//--------------------------------------------------------------
void output_debug()
{
  if (((millis() - last_debug_output_time) >= 3000) && (debug_mode)) 
  {
    Serial.flush();
    Serial.print("debug: \t");
    Serial.print(CODE_VERSION);
    Serial.print("\t\t");
    Serial.print(millis()/1000);
    Serial.print("\t\t");
    #ifdef DEBUG_MEMORY
    void* HP = malloc(4);
    if (HP) 
    {
      free (HP);
    }
    unsigned long free = (unsigned long)SP - (unsigned long)HP;
//    if (free > 2048) {
//      free = 0;
//    }
    Serial.print((unsigned long)free,DEC);
    Serial.print(F("b free"));
    #endif
    
    #ifdef FEATURE_YAESU_EMULATION
    Serial.print(F("\t\tGS-232"));    
    #ifdef OPTION_GS_232B_EMULATION
    Serial.print(F("B"));
    #endif
    #ifndef OPTION_GS_232B_EMULATION
    Serial.print(F("A"));
    #endif
    #endif //FEATURE_YAESU_EMULATION

    Serial.println();
       
    Serial.print(F("\tAZ: "));
    switch (az_state) 
    {
      case IDLE:                           Serial.print(F("IDLE")); break;
      case SLOW_START_CW:                  Serial.print(F("SLOW_START_CW")); break;
      case SLOW_START_CCW:                 Serial.print(F("SLOW_START_CCW")); break;
      case NORMAL_CW:                      Serial.print(F("NORMAL_CW")); break;
      case NORMAL_CCW:                     Serial.print(F("NORMAL_CCW")); break;
      case SLOW_DOWN_CW:                   Serial.print(F("SLOW_DOWN_CW")); break;
      case SLOW_DOWN_CCW:                  Serial.print(F("SLOW_DOWN_CCW")); break;
      case INITIALIZE_SLOW_START_CW:       Serial.print(F("INITIALIZE_SLOW_START_CW")); break;
      case INITIALIZE_SLOW_START_CCW:      Serial.print(F("INITIALIZE_SLOW_START_CCW")); break;
      case INITIALIZE_TIMED_SLOW_DOWN_CW:  Serial.print(F("INITIALIZE_TIMED_SLOW_DOWN_CW")); break;
      case INITIALIZE_TIMED_SLOW_DOWN_CCW: Serial.print(F("INITIALIZE_TIMED_SLOW_DOWN_CCW")); break;
      case TIMED_SLOW_DOWN_CW:             Serial.print(F("TIMED_SLOW_DOWN_CW")); break;
      case TIMED_SLOW_DOWN_CCW:            Serial.print(F("TIMED_SLOW_DOWN_CCW")); break;
    }
    
    Serial.print(F("\tQ: "));
    switch(az_request_queue_state)
    {
      case NONE:                           Serial.print(F("-")); break;
      case IN_QUEUE:                       Serial.print(F("IN_QUEUE")); break;
      case IN_PROGRESS_TIMED:              Serial.print(F("IN_PROGRESS_TIMED")); break;
      case IN_PROGRESS_TO_TARGET:          Serial.print(F("IN_PROGRESS_TO_TARGET")); break;
    }
    
    Serial.print(F("\tAZ: "));
    Serial.print(azimuth            / HEADING_MULTIPLIER, DECIMAL_PLACES);
    Serial.print(F(" (raw: "));
    Serial.print(raw_azimuth        / HEADING_MULTIPLIER, DECIMAL_PLACES);
    Serial.print(")");
    
    Serial.print(F("\tTarget: "));
    Serial.print(target_azimuth     / HEADING_MULTIPLIER, DECIMAL_PLACES);
    
    Serial.print(F(" (raw: "));
    Serial.print(target_raw_azimuth / HEADING_MULTIPLIER, DECIMAL_PLACES);
    Serial.print(")");

    #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
    Serial.print(F("\tAnalog: "));
    Serial.println(analog_az);
    #endif //FEATURE_AZ_POSITION_POTENTIOMETER
    
    if (normal_az_speed_voltage) 
    //{
      Serial.print(F("\tAZ Speed Norm: "));
      Serial.print(normal_az_speed_voltage, DEC);
    //}
    
    Serial.print(F(" Current: "));
    Serial.print(current_az_speed_voltage,DEC);
    
    if (az_speed_pot) 
    {
      Serial.print(F("\tAZ Speed Pot: "));
      Serial.print(analogRead(az_speed_pot));
    }    
    if (az_preset_pot) 
    {
      Serial.print(F("\tAZ Preset Pot Analog: "));
      Serial.print(analogRead(az_preset_pot));
      Serial.print(F("\tAZ Preset Pot Setting: "));
      Serial.print(map(analogRead(az_preset_pot), AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP));
    }
        
    Serial.println();

    #ifdef FEATURE_ELEVATION_CONTROL
    Serial.print(F("\tEL: "));
    switch (el_state) 
    {
      case IDLE: Serial.print(F("IDLE")); break;
      case SLOW_START_UP: Serial.print(F("SLOW_START_UP")); break;
      case SLOW_START_DOWN: Serial.print(F("SLOW_START_DOWN")); break;
      case NORMAL_UP: Serial.print(F("NORMAL_UP")); break;
      case NORMAL_DOWN: Serial.print(F("NORMAL_DOWN")); break;
      case SLOW_DOWN_DOWN: Serial.print(F("SLOW_DOWN_DOWN")); break;
      case SLOW_DOWN_UP: Serial.print(F("SLOW_DOWN_UP")); break;
      case TIMED_SLOW_DOWN_UP: Serial.print(F("TIMED_SLOW_DOWN_UP")); break;
      case TIMED_SLOW_DOWN_DOWN: Serial.print(F("TIMED_SLOW_DOWN_DOWN")); break;
    }    

    Serial.print(F("\tQ: "));
    switch (el_request_queue_state) 
    {
      case NONE: Serial.print(F("-")); break;
      case IN_QUEUE: Serial.print(F("IN_QUEUE")); break;
      case IN_PROGRESS_TIMED: Serial.print(F("IN_PROGRESS_TIMED")); break;
      case IN_PROGRESS_TO_TARGET: Serial.print(F("IN_PROGRESS_TO_TARGET")); break;     
    }
    #ifdef FEATURE_EL_POSITION_POTENTIOMETER
    Serial.print(F("\tEL Analog: "));
    Serial.print(analog_el);
    #endif //FEATURE_EL_POSITION_POTENTIOMETER
    Serial.print(F("\tEL: "));
    Serial.print(elevation/LCD_HEADING_MULTIPLIER,LCD_DECIMAL_PLACES);
    Serial.print(F("\tTarget: "));
    Serial.println(target_elevation/LCD_HEADING_MULTIPLIER,LCD_DECIMAL_PLACES);
    #endif //FEATURE_EL_POSITION_POTENTIOMETER

    #ifdef FEATURE_TIMED_BUFFER
    if (timed_buffer_status != EMPTY) 
    {
      Serial.print(F("\tTimed interval buff: "));
      switch (timed_buffer_status) 
      {
        //case EMPTY: Serial.print(F("EMPTY")); break;
        case LOADED_AZIMUTHS: Serial.print(F("LOADED_AZIMUTHS")); break;
        case RUNNING_AZIMUTHS: Serial.print(F("RUNNING_AZIMUTHS")); break;
        #ifdef FEATURE_ELEVATION_CONTROL
        case LOADED_AZIMUTHS_ELEVATIONS: Serial.print(F("LOADED_AZIMUTHS_ELEVATIONS")); break;
        case RUNNING_AZIMUTHS_ELEVATIONS: Serial.print(F("RUNNING_AZIMUTHS_ELEVATIONS")); break;
        #endif
      }
  
      Serial.print(F("\tInterval (secs): "));
      Serial.print(timed_buffer_interval_value_seconds,DEC);
      Serial.print(F("\tEntries: "));
      Serial.print(timed_buffer_number_entries_loaded,DEC);
      Serial.print(F("\tEntry ptr: "));
      Serial.print(timed_buffer_entry_pointer,DEC);
      Serial.print(F("\tSecs since last action: "));
      Serial.println((millis()-last_timed_buffer_action_time)/1000);
  
      if (timed_buffer_number_entries_loaded > 0) 
      {
        for (int x = 0;x < timed_buffer_number_entries_loaded; x++) 
        {
          Serial.print(x+1);
          Serial.print(F("\t:"));
          Serial.print(timed_buffer_azimuths[x]/HEADING_MULTIPLIER);
          #ifdef FEATURE_ELEVATION_CONTROL
          Serial.print(F("\t- "));
          Serial.print(timed_buffer_elevations[x]/HEADING_MULTIPLIER);
          #endif
          Serial.println();
        }
      }
    } //if (timed_buffer_status != EMPTY)
    #endif //FEATURE_TIMED_BUFFER

    Serial.print(F("\tAZ: "));
    Serial.print(configuration.azimuth_starting_point);
    Serial.print(F("+"));
    Serial.print(configuration.azimuth_rotation_capability);
    Serial.print(F("\tAZ ana: "));
    Serial.print(configuration.analog_az_full_ccw);
    Serial.print(F("-"));
    Serial.print(configuration.analog_az_full_cw);
    #ifdef FEATURE_ELEVATION_CONTROL
    Serial.print(F("\tEL ana: "));
    Serial.print(configuration.analog_el_0_degrees);
    Serial.print(F("-"));
    Serial.print(configuration.analog_el_max_elevation);
    #endif
    
    #ifdef FEATURE_HOST_REMOTE_PROTOCOL
    Serial.print(F("\n\tRemote: Command: "));
    Serial.print(remote_unit_command_submitted);
    Serial.print(F(" Good: "));
    Serial.print(remote_unit_good_results);
    Serial.print(F(" Bad: "));
    Serial.print(remote_unit_bad_results);
    Serial.print(F(" Index: "));
    Serial.print(serial1_buffer_index);
    Serial.print(F(" CmdTouts: "));
    Serial.print(remote_unit_command_timeouts);
    Serial.print(F(" BuffTouts: "));
    Serial.print(remote_unit_incoming_buffer_timeouts);
    Serial.print(F(" Result: "));
    Serial.println(remote_unit_command_result_float);
    #endif //#FEATURE_HOST_REMOTE_PROTOCOL
    
    #ifdef DEBUG_POSITION_PULSE_INPUT
    static unsigned long last_pulse_count_time = 0; 
    static unsigned long last_az_pulse_counter = 0;
    static unsigned long last_el_pulse_counter = 0;
    Serial.print(F("\n\tPulse counters: AZ: "));
    Serial.print(az_pulse_counter);
    Serial.print(F(" AZ Ambiguous: "));
    Serial.print(az_pulse_counter_ambiguous);
    Serial.print(" EL: ");
    Serial.print(el_pulse_counter);
    Serial.print(F(" EL Ambiguous: "));
    Serial.print(el_pulse_counter_ambiguous);
    Serial.print(F(" Rate per sec: AZ: "));
    Serial.print((az_pulse_counter-last_az_pulse_counter)/((millis()-last_pulse_count_time)/1000.0));
    Serial.print(F(" EL: "));
    Serial.println((el_pulse_counter-last_el_pulse_counter)/((millis()-last_pulse_count_time)/1000.0));
    last_az_pulse_counter = az_pulse_counter;
    last_el_pulse_counter = el_pulse_counter;
    last_pulse_count_time = millis();
    #endif //DEBUG_POSITION_PULSE_INPUT

    Serial.println(F("\n\n\n\n"));
    
    last_debug_output_time = millis(); 
  }
}



//--------------- Elevation -----------------------
#ifdef FEATURE_ELEVATION_CONTROL
void el_check_operation_timeout()
{
  // check if the last executed rotation operation has been going on too long

  if (((millis() - el_last_rotate_initiation) > OPERATION_TIMEOUT) && (el_state != IDLE)) 
  {
    submit_request(EL,REQUEST_KILL,0);
    #ifdef DEBUG_EL_CHECK_OPERATION_TIMEOUT
    if (debug_mode) 
    {
      Serial.println(F("el_check_operation_timeout: timeout reached, aborting rotation"));
    }
    #endif //DEBUG_EL_CHECK_OPERATION_TIMEOUT
  }
}
#endif

//--------------------------------------------------------------
//#ifdef FEATURE_ELEVATION_CONTROL
#ifdef FEATURE_YAESU_EMULATION
void yaesu_w_command ()
{
  // parse out W command
  // Short Format: WXXX YYY = azimuth YYY = elevation
  // Long Format : WSSS XXX YYY  SSS = timed interval   XXX = azimuth    YYY = elevation
  
  #ifdef FEATURE_ELEVATION_CONTROL
  int parsed_elevation = 0;
  #endif
  int parsed_azimuth = 0;
  //int parsed_value1 = 0;
  //int parsed_value2 = 0;

  if (serial0_buffer_index > 8) 
  {  // if there are more than 4 characters in the command buffer, we got a timed interval command
    #ifdef FEATURE_TIMED_BUFFER
    parsed_value1 = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
    if ((parsed_value1 > 0) && (parsed_value1 < 1000)) 
    {
      timed_buffer_interval_value_seconds = parsed_value1;
      for (int x = 5; x < serial0_buffer_index; x = x + 8) 
      {
        parsed_value1 = ((int(serial0_buffer[x])-48)*100) + ((int(serial0_buffer[x+1])-48)*10) + (int(serial0_buffer[x+2])-48);
        parsed_value2 = ((int(serial0_buffer[x+4])-48)*100) + ((int(serial0_buffer[x+5])-48)*10) + (int(serial0_buffer[x+6])-48);
        if ((parsed_value1 > -1) && (parsed_value1 < 361) && (parsed_value2 > -1) && (parsed_value2 < 181)) 
        {  // is it a valid azimuth?
          timed_buffer_azimuths[timed_buffer_number_entries_loaded] = (parsed_value1 * HEADING_MULTIPLIER);
          timed_buffer_elevations[timed_buffer_number_entries_loaded] = (parsed_value2 * HEADING_MULTIPLIER);
          timed_buffer_number_entries_loaded++;
          timed_buffer_status = LOADED_AZIMUTHS_ELEVATIONS;
          if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) 
          {   // is the array full?
            x = serial0_buffer_index;  // array is full, go to the first azimuth and elevation
          }
        } else 
        {   // we hit an invalid bearing
          timed_buffer_status = EMPTY;
          timed_buffer_number_entries_loaded = 0;
          Serial.println(F("?>"));  // error
          return;
        }
      }
    }
    timed_buffer_entry_pointer = 1;             // go to the first bearings
    parsed_azimuth = timed_buffer_azimuths[0];
    parsed_elevation = timed_buffer_elevations[0];
    #else
    Serial.println(F("Feature not activated ?>"));
    #endif //FEATURE_TIMED_BUFFER
  } else 
  {
    // this is a short form W command, just parse the azimuth and elevation and initiate rotation
    parsed_azimuth = (((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48)) * HEADING_MULTIPLIER;
    
    #ifdef FEATURE_ELEVATION_CONTROL
    parsed_elevation = (((int(serial0_buffer[5])-48)*100) + ((int(serial0_buffer[6])-48)*10) + (int(serial0_buffer[7])-48)) * HEADING_MULTIPLIER;
    #endif
  }

  if ((parsed_azimuth >= 0) && (parsed_azimuth <= (360*HEADING_MULTIPLIER))) 
  {
    submit_request(AZ,REQUEST_AZIMUTH,parsed_azimuth);
  } else 
  {
    #ifdef DEBUG_YAESU
    if (debug_mode) {Serial.println(F("yaesu_w_command: W command elevation error"));}
    #endif //DEBUG_YAESU
    Serial.println(F("?>"));      // bogus elevation - return and error and don't do anything
    return;
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((parsed_elevation >= 0) && (parsed_elevation <= (180 * HEADING_MULTIPLIER))) 
  {
    submit_request(EL,REQUEST_ELEVATION,parsed_elevation);
  } else 
  {
    #ifdef DEBUG_YAESU
    if (debug_mode) {Serial.println(F("yaesu_w_command: W command elevation error"));}
    #endif //DEBUG_YAESU
    Serial.println(F("?>"));      // bogus elevation - return and error and don't do anything
    return;
  }
  #endif //FEATURE_ELEVATION_CONTROL
  Serial.println();
}
#endif //FEATURE_YAESU_EMULATION
//#endif //FEATURE_ELEVATION_CONTROL

//--------------------------------------------------------------

#ifdef FEATURE_ELEVATION_CONTROL
void read_elevation()
{
  // read analog input and convert it to degrees

  unsigned int previous_elevation = elevation;
  static unsigned long last_measurement_time = 0;
  
  #ifdef DEBUG_HEADING_READING_TIME
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;
  static float average_read_time = 0;
  #endif //DEBUG_HEADING_READING_TIME  
  
  #ifndef FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT
  if ((millis() - last_measurement_time) > ELEVATION_MEASUREMENT_FREQUENCY_MS){
  #else
  if (1)
  {
  #endif
  
    #ifdef FEATURE_EL_POSITION_POTENTIOMETER
    analog_el = analogRead(rotator_analog_el);
    elevation = (map(analog_el, configuration.analog_el_0_degrees, configuration.analog_el_max_elevation, 0, (ELEVATION_MAXIMUM_DEGREES* HEADING_MULTIPLIER))) ;
    #ifdef FEATURE_ELEVATION_CORRECTION
    elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_ELEVATION_CORRECTION
    if (ELEVATION_SMOOTHING_FACTOR > 0) 
    {
      elevation = (elevation*(1-(ELEVATION_SMOOTHING_FACTOR/100))) + (previous_elevation*(ELEVATION_SMOOTHING_FACTOR/100));
    }
    if (elevation < 0) 
    {
      elevation = 0;
    }
    #endif //FEATURE_EL_POSITION_POTENTIOMETER
    
    
    #ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
    static byte el_position_encoder_state = 0;
    
    el_position_encoder_state = ttable[el_position_encoder_state & 0xf][((digitalRead(el_rotary_position_pin2) << 1) | digitalRead(el_rotary_position_pin1))];
    byte el_position_encoder_result = el_position_encoder_state & 0x30;
    if (el_position_encoder_result) 
    {
      if (el_position_encoder_result == DIR_CW) 
      {
        configuration.last_elevation = configuration.last_elevation + EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
        #ifdef DEBUG_POSITION_ROTARY_ENCODER
        if (debug_mode){Serial.println(F("read_elevation: EL_POSITION_ROTARY_ENCODER: CW/UP"));}
        #endif //DEBUG_POSITION_ROTARY_ENCODER
      }
      if (el_position_encoder_result == DIR_CCW) 
      {
        configuration.last_elevation = configuration.last_elevation - EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE;
        #ifdef DEBUG_POSITION_ROTARY_ENCODER
        if (debug_mode){Serial.println(F("read_elevation: EL_POSITION_ROTARY_ENCODER: CCW/DWN"));}   
        #endif //DEBUG_POSITION_ROTARY_ENCODER   
      }
      #ifdef OPTION_EL_POSITION_ROTARY_ENCODER_HARD_LIMIT
      if (configuration.last_elevation < 0)
      {
        configuration.last_elevation = 0;
      }
      if (configuration.last_elevation > ELEVATION_MAXIMUM_DEGREES)
      {
        configuration.last_elevation = ELEVATION_MAXIMUM_DEGREES ;
      }
      #endif
      
      elevation = int(configuration.last_elevation * HEADING_MULTIPLIER);
      #ifdef FEATURE_ELEVATION_CORRECTION
      elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
      #endif //FEATURE_ELEVATION_CORRECTION    
      configuration_dirty = 1;
    }
    #endif //FEATURE_EL_POSITION_ROTARY_ENCODER  
   
    #ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
    AccelerometerRaw raw = accel.ReadRawAxis();
    AccelerometerScaled scaled = accel.ReadScaledAxis();
    #ifdef DEBUG_ACCEL
    if (debug_mode) 
    {
      Serial.print(F("read_elevation: raw.ZAxis: "));
      Serial.println(raw.ZAxis);
    }
    #endif //DEBUG_ACCEL   
    elevation = (atan2(scaled.YAxis,scaled.ZAxis)* 180 * HEADING_MULTIPLIER)/M_PI;  
    #ifdef FEATURE_ELEVATION_CORRECTION
    elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_ELEVATION_CORRECTION      
    if (ELEVATION_SMOOTHING_FACTOR > 0) 
    {
      elevation = (elevation*(1-(ELEVATION_SMOOTHING_FACTOR/100))) + (previous_elevation*(ELEVATION_SMOOTHING_FACTOR/100));
    }    
  
    #endif //FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
    
    #ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB 
    sensors_event_t event; 
    accel.getEvent(&event);
    #ifdef DEBUG_ACCEL
    if (debug_mode) 
    {
      Serial.print(F("read_elevation: event.acceleration.z: "));
      Serial.println(event.acceleration.z);
    }
    #endif //DEBUG_ACCEL
    elevation = (atan2(event.acceleration.y,eventINITIALIZE_TIMED_SLOW_DOWN_CCW.acceleration.z)* 180 * HEADING_MULTIPLIER)/M_PI;  
    #ifdef FEATURE_ELEVATION_CORRECTION
    elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_ELEVATION_CORRECTION       
    #endif //FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
    
    #ifdef FEATURE_EL_POSITION_LSM303
    lsm.read();
    #ifdef DEBUG_ACCEL
    if (debug_mode) 
    {
      Serial.print(F("read_elevation: lsm.accelData.y: "));
      Serial.print(lsm.accelData.y);
      Serial.print(F(" lsm.accelData.z: "));
      Serial.println(lsm.accelData.z);
    }
    #endif //DEBUG_ACCEL
    elevation = (atan2(lsm.accelData.y,lsm.accelData.z)* 180 * HEADING_MULTIPLIER)/M_PI;
    #ifdef FEATURE_ELEVATION_CORRECTION
    elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
    #endif //FEATURE_ELEVATION_CORRECTION           
    #endif //FEATURE_EL_POSITION_LSM303
    
    #ifdef FEATURE_EL_POSITION_PULSE_INPUT
    #ifdef DEBUG_POSITION_PULSE_INPUT
//    if (el_position_pule_interrupt_handler_flag) 
//    {
//      Serial.print(F("read_elevation: el_position_pule_interrupt_handler_flag: "));
//      Serial.println(el_position_pule_interrupt_handler_flag);
//      el_position_pule_interrupt_handler_flag = 0;
//    }
    #endif //DEBUG_POSITION_PULSE_INPUT  
    
    static float last_el_position_pulse_input_elevation = el_position_pulse_input_elevation;
    
    if (el_position_pulse_input_elevation != last_el_position_pulse_input_elevation)
    {
      #ifdef DEBUG_POSITION_PULSE_INPUT
//      if (debug_mode)
//      {
//        Serial.print(F("read_elevation: el_position_pulse_input_elevation:"));
//        Serial.println(el_position_pulse_input_elevation);
//      }   
      #endif //DEBUG_POSITION_PULSE_INPUT     
      configuration.last_elevation = el_position_pulse_input_elevation;
      configuration_dirty = 1;
      last_el_position_pulse_input_elevation = el_position_pulse_input_elevation;
      elevation = int(configuration.last_elevation * HEADING_MULTIPLIER);
       
      #ifdef FEATURE_ELEVATION_CORRECTION
      elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
      #endif FEATURE_ELEVATION_CORRECTION
       
    }
    #endif //FEATURE_EL_POSITION_PULSE_INPUT  
    
    #ifdef FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT
    static unsigned long last_remote_unit_el_query_time = 0;
   
    // do we have a command result waiting for us?
    if (remote_unit_command_results_available == REMOTE_UNIT_EL_COMMAND) 
    {  
      #ifdef DEBUG_HEADING_READING_TIME
      average_read_time = (average_read_time + (millis()-last_time))/2.0;
      last_time = millis();
    
      if (debug_mode)
      {
        if ((millis()-last_print_time) > 1000)
        {
          Serial.print(F("read_elevation: avg read frequency: "));
          Serial.println(average_read_time,2);
         last_print_time = millis();
        }
      }
      #endif //DEBUG_HEADING_READING_TIME

      elevation = remote_unit_command_result_float * HEADING_MULTIPLIER;
      
      #ifdef FEATURE_ELEVATION_CORRECTION
      elevation = (correct_elevation(elevation/HEADING_MULTIPLIER)*HEADING_MULTIPLIER);
      #endif //FEATURE_ELEVATION_CORRECTION      
      
      if (ELEVATION_SMOOTHING_FACTOR > 0) 
      {
        elevation = (elevation*(1-(ELEVATION_SMOOTHING_FACTOR/100))) + (previous_elevation*(ELEVATION_SMOOTHING_FACTOR/100));
      }      
      remote_unit_command_results_available = 0;
    } else 
    { 
      // is it time to request the elevation?
      if ((millis() - last_remote_unit_el_query_time) > EL_REMOTE_UNIT_QUERY_TIME_MS)
      {
        if (submit_remote_command(REMOTE_UNIT_EL_COMMAND))
        {
          last_remote_unit_el_query_time = millis();
        }
      }
    }   
    #endif //FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT
    last_measurement_time = millis();
  }  
}
#endif

//--------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
void report_current_elevation() 
{
  #ifdef FEATURE_YAESU_EMULATION
  // The C2 command that reports elevation in +0nnn format

  String elevation_string;

  #ifndef OPTION_GS_232B_EMULATION
  if (elevation < 0)
  {
    Serial.print(F("-0"));
  } else 
  {
    Serial.print(F("+0"));
  }
  #endif
  #ifdef OPTION_GS_232B_EMULATION
  Serial.print(F("EL="));
  #endif
  elevation_string = String(abs(int(elevation/HEADING_MULTIPLIER)), DEC);
  if (elevation_string.length() == 1) 
  {
    Serial.print(F("00"));
  } else 
  {
    if (elevation_string.length() == 2) 
    {
      Serial.print(F("0"));
    }
  }
  Serial.println(elevation_string);
  #endif //FEATURE_YAESU_EMULATION
}
#endif

//--------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
void update_el_variable_outputs(byte speed_voltage)
{  
  #ifdef DEBUG_VARIABLE_OUTPUTS
  if (debug_mode) 
  {
    Serial.print(F("update_el_variable_outputs: speed_voltage: "));
    Serial.print(speed_voltage);
  }
  #endif //DEBUG_VARIABLE_OUTPUTS

  if (((el_state == SLOW_START_UP) ||
	   (el_state == NORMAL_UP) ||
	   (el_state == SLOW_DOWN_UP) ||
	   (el_state == TIMED_SLOW_DOWN_UP)) &&
	   (rotate_up_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_up_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_up_pwm,speed_voltage);    
  }
  
  if (((el_state == SLOW_START_DOWN)       ||
	   (el_state == NORMAL_DOWN)           ||
	   (el_state == SLOW_DOWN_DOWN)        ||
	   (el_state == TIMED_SLOW_DOWN_DOWN)) &&
	   (rotate_down_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_down_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_down_pwm,speed_voltage);   
  }

  if (((el_state == SLOW_START_DOWN)      ||
       (el_state == NORMAL_DOWN)          ||
	   (el_state == SLOW_DOWN_DOWN)       ||
	   (el_state == TIMED_SLOW_DOWN_DOWN) ||
       (el_state == SLOW_START_UP)        ||
	   (el_state == NORMAL_UP)            ||
       (el_state == SLOW_DOWN_UP)         ||
       (el_state == TIMED_SLOW_DOWN_UP))  &&
	   (rotate_up_down_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_up_down_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_up_down_pwm,speed_voltage);   
  }

  if (((el_state == SLOW_START_UP)       ||
	   (el_state == NORMAL_UP)           ||
	   (el_state == SLOW_DOWN_UP)        ||
	   (el_state == TIMED_SLOW_DOWN_UP)) &&
	   (rotate_up_freq))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_up_freq"));} 
    #endif //DEBUG_VARIABLE_OUTPUTS 
    tone(rotate_up_freq,map(speed_voltage,0,255,EL_VARIABLE_FREQ_OUTPUT_LOW,EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }
  
  if (((el_state == SLOW_START_DOWN)       ||
	   (el_state == NORMAL_DOWN)           ||
	   (el_state == SLOW_DOWN_DOWN)        ||
	   (el_state == TIMED_SLOW_DOWN_DOWN)) &&
	   (rotate_down_freq))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_down_freq"));} 
    #endif //DEBUG_VARIABLE_OUTPUTS 
    tone(rotate_down_freq,map(speed_voltage,0,255,EL_VARIABLE_FREQ_OUTPUT_LOW,EL_VARIABLE_FREQ_OUTPUT_HIGH));
  }  
  
  if (elevation_speed_voltage)
  {
    analogWrite(elevation_speed_voltage,speed_voltage);
  }
  
  if (debug_mode) {Serial.println();}
  
  current_el_speed_voltage = speed_voltage;
}
#endif //FEATURE_ELEVATION_CONTROL


//--------------------------------------------------------------
// rotator(), write the rotator controls, analogWrite(), digitalWrite()
// action: ACTIVATE, DEACTIVATE
// type: CW, CCW, UP, DOWN

// rotator control supported
//    rotate_cw, rotate_ccw
//    rotate_cw, H-bridge 1, 2
// new control mode for HCO board
//    motor_on, direction (cw/ccw)

//TODO This code sets the I/O for the brake and direction of rotation
//     Code above sets the speed for rotation
//     Layers, K3NG concept, I/O, Rotator type...needs more isolation
// inputs
//    rotation_action: is activate or deactivate
//    rotation_type: is CW or CCW
// global variables
//  pin numbers, otate_cw, rotate_ccw, rotate_cw_pwn, etc 
void rotator(byte rotation_action, byte rotation_type) 
{  
  #ifdef DEBUG_ROTATOR
  if (debug_mode) 
  {
    Serial.flush();
    Serial.print(F("rotator: rotation_action:"));
    Serial.print(rotation_action);
    Serial.print(F(" rotation_type:"));
    Serial.flush();
    Serial.print(rotation_type);
    Serial.print(F("->"));
    Serial.flush();
    //delay(1000);
  }   
  #endif //DEBUG_ROTATOR
  
  switch(rotation_type) // CW, CCW, UP, DOWN
  {
    case CW:
      #ifdef DEBUG_ROTATOR
      if (debug_mode) { Serial.print(F("CW ")); Serial.flush();}
      #endif //DEBUG_ROTATOR
      if (rotation_action == ACTIVATE) 
      {    
        #ifdef DEBUG_ROTATOR    
        if (debug_mode) { Serial.println(F("ACTIVATE")); Serial.flush();}
        #endif //DEBUG_ROTATOR
          brake_release(AZ, true);
          if (az_slowstart_active) // CW, Activate, slow start
          {
        	if (rotate_cw_pwm)
            {
            	analogWrite(rotate_cw_pwm,    0); //CW, ACTIVATE, slow start, write 0 to pwm
            }
            if (rotate_ccw_pwm)
            {
            	analogWrite( rotate_ccw_pwm,   0);
            	digitalWrite(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {analogWrite(rotate_cw_ccw_pwm, 0);}
            if (rotate_motor     ) {digitalWrite(rotate_motor,     LOW);}
            if (rotate_cw_freq)
            {
            	noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq)
            {
            	noTone(rotate_ccw_freq);
            }
          } else // CW, Activate, not slow start
          {
        	// CW, Activate, not slow start
        	if (rotate_cw_pwm)
            {
            	analogWrite(rotate_cw_pwm, normal_az_speed_voltage); //CW, ACTIVATE, slowstart, write speed to pwm
            }
            if (rotate_ccw_pwm)
            {
            	analogWrite(rotate_ccw_pwm,  0);  //CW, ACTIVATE, slowstart, write zero to pwm, turn off
            	digitalWrite(rotate_ccw_pwm, LOW);
            }
            if (rotate_cw_ccw_pwm) {analogWrite(rotate_cw_ccw_pwm, normal_az_speed_voltage);}
            if (rotate_motor     ) {digitalWrite(rotate_motor,     HIGH);}
            if (rotate_cw_freq)
            {
            	tone(rotate_cw_freq,
            		 map(normal_az_speed_voltage,
            		     0,
						 255,
						 AZ_VARIABLE_FREQ_OUTPUT_LOW,
						 AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }
            if (rotate_ccw_freq)
            {
            	noTone(rotate_ccw_freq);
            }
          } // if slow start active, else

          
          if (rotate_cw) // CW, Activate, not pwm
          {
        	  digitalWrite(rotate_cw,  ROTATE_PIN_ACTIVE_VALUE  );  // CW, ACTIVATE, start or continue
          }
          if (rotate_ccw)
          {
        	  digitalWrite(rotate_ccw, ROTATE_PIN_INACTIVE_VALUE  );  //CCW, ACTIVATE, stop
          }

          if (rotate_h1) // if pins defined CW, Activate, pwm or not
          {
        	  digitalWrite(rotate_h1, 1);
          }
          if (rotate_h2) // if pins defined CW, Activate, pwm or not
          {
            digitalWrite(rotate_h2, 0);
          }

          #ifdef DEBUG_ROTATOR     
          if (debug_mode) 
          {
            Serial.print(F("rotator: normal_az_speed_voltage:")); 
            Serial.println(normal_az_speed_voltage);
            Serial.flush();
          }
          #endif

      } else // CW, not ACTIVATE
      {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {Serial.println(F("DEACTIVATE"));Serial.flush();}
        #endif //DEBUG_ROTATOR

        // all the different ways to stop
        if (rotate_cw_pwm)
        {
        	analogWrite(rotate_cw_pwm,0);
        	digitalWrite(rotate_cw_pwm,LOW);
        }
        if (rotate_cw_ccw_pwm) {analogWrite(rotate_cw_ccw_pwm, 0); }
        if (rotate_motor     ) {digitalWrite(rotate_motor,     LOW);}
        if (rotate_cw)
        {
        	digitalWrite(rotate_cw,ROTATE_PIN_INACTIVE_VALUE);
        }
        if (rotate_cw_freq)
        {
        	noTone(rotate_cw_freq);
        }
        // if pins defined CW, Activate, pwm or not
        if (rotate_h1) { digitalWrite(rotate_h1, 0); }
        if (rotate_h2) { digitalWrite(rotate_h2, 0); }

      } 
      break; // case CW
    case CCW:
      #ifdef DEBUG_ROTATOR
      if (debug_mode) {Serial.print(F("CCW "));Serial.flush();}
      #endif //DEBUG_ROTATOR
      if (rotation_action == ACTIVATE) 
      {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) {Serial.println(F("ACTIVATE"));Serial.flush();}
          #endif //DEBUG_ROTATOR
          brake_release(AZ, true);

          if (az_slowstart_active) // CCW, Activate, slow satart
          {
            if (rotate_cw_pwm)
            {
            	analogWrite(rotate_cw_pwm,0);
            	digitalWrite(rotate_cw_pwm,LOW);
            }
            if (rotate_ccw_pwm)    { analogWrite(rotate_ccw_pwm,    0); }
            if (rotate_cw_ccw_pwm) { analogWrite(rotate_cw_ccw_pwm, 0); }
            if (rotate_motor     ) {digitalWrite(rotate_motor,    LOW);}
            if (rotate_cw_freq)
            {
            	noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq)
            {
            	noTone(rotate_ccw_freq);
            }
          }
          else // CCW, activate, not slow start
          {
            if (rotate_cw_pwm)
            {
            	analogWrite(rotate_cw_pwm,0);
            	digitalWrite(rotate_cw_pwm,LOW);
            }
            if (rotate_ccw_pwm)
            {
            	analogWrite(rotate_ccw_pwm,normal_az_speed_voltage);
            }
            if (rotate_cw_ccw_pwm) { analogWrite( rotate_cw_ccw_pwm, normal_az_speed_voltage); }
            if (rotate_motor     ) { digitalWrite(rotate_motor,                         HIGH);}
            if (rotate_cw_freq)
            {
            	noTone(rotate_cw_freq);
            }
            if (rotate_ccw_freq)
            {
            	tone(rotate_ccw_freq,map(normal_az_speed_voltage,
            			                 0,
										 255,
										 AZ_VARIABLE_FREQ_OUTPUT_LOW,
										 AZ_VARIABLE_FREQ_OUTPUT_HIGH));
            }
          }
          // CCW, Activate
          if (rotate_cw)
          {
        	  digitalWrite(rotate_cw,  ROTATE_PIN_INACTIVE_VALUE);
          }
          if (rotate_ccw) { digitalWrite(rotate_ccw, ROTATE_PIN_ACTIVE_VALUE); } 
          // CCW, Activate, pwm or not, if pins defined
          if (rotate_h1) { digitalWrite(rotate_h1, 0); }
          if (rotate_h2) { digitalWrite(rotate_h2, 1); }

          #ifdef DEBUG_ROTATOR
          if (debug_mode) 
          {
            Serial.print(F("rotator: normal_az_speed_voltage:")); 
            Serial.println(normal_az_speed_voltage);
            Serial.flush();
          }     
          #endif //DEBUG_ROTATOR
      } else // CCW, DEACTIVATE
      {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) {Serial.println(F("DEACTIVATE"));Serial.flush();}
        #endif //DEBUG_ROTATOR
        if (rotate_ccw_pwm)
        {
        	analogWrite( rotate_ccw_pwm,  0);
        	digitalWrite(rotate_ccw_pwm,LOW);
        }
        if (rotate_ccw)      { digitalWrite(rotate_ccw,ROTATE_PIN_INACTIVE_VALUE); }
        if (rotate_ccw_freq) { noTone(rotate_ccw_freq); }
        // CCW, Activate, pwm or not, if pins defined
        if (rotate_h1)  { digitalWrite(rotate_h1, 0); }
        if (rotate_h2)  { digitalWrite(rotate_h2, 0); }
      } // CCW, DEACTIVATE
      break; 

    #ifdef FEATURE_ELEVATION_CONTROL
    
    //TODO: add pwm and freq pins
    case UP:
      #ifdef DEBUG_ROTATOR
      if (debug_mode) { Serial.print(F("ROTATION_UP "));Serial.flush(); }
      #endif //DEBUG_ROTATOR
      if (rotation_action == ACTIVATE) 
      {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) { Serial.println(F("ACTIVATE"));Serial.flush(); }
          #endif //DEBUG_ROTATOR
          brake_release(EL, true);
          if (el_slowstart_active) 
          {
            if (rotate_up_pwm)      {analogWrite(rotate_up_pwm,0);}
            if (rotate_down_pwm)    {analogWrite(rotate_down_pwm,0);digitalWrite(rotate_down_pwm,LOW);}
            if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,0);}
            if (rotate_up_freq)     {noTone(rotate_up_freq);}
            if (rotate_down_freq)   {noTone(rotate_down_freq);}
          } else 
          {
            if (rotate_up_pwm)      {analogWrite(rotate_up_pwm,normal_el_speed_voltage);}
            if (rotate_down_pwm)    {analogWrite(rotate_down_pwm,0);digitalWrite(rotate_down_pwm,LOW);}
            if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,normal_el_speed_voltage);}
            if (rotate_up_freq)     {tone(rotate_up_freq,map(normal_el_speed_voltage,0,255,EL_VARIABLE_FREQ_OUTPUT_LOW,EL_VARIABLE_FREQ_OUTPUT_HIGH));}
            if (rotate_down_freq)   {noTone(rotate_down_freq);}
          }          
          if (rotate_up)         {digitalWrite(rotate_up,ROTATE_PIN_ACTIVE_VALUE);}
          if (rotate_down)       {digitalWrite(rotate_down,ROTATE_PIN_INACTIVE_VALUE);}
          if (rotate_up_or_down) {digitalWrite(rotate_up_or_down,ROTATE_PIN_ACTIVE_VALUE);}
      } else 
      {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) { Serial.println(F("DEACTIVATE"));Serial.flush(); }
        #endif //DEBUG_ROTATOR
        if (rotate_up)          {digitalWrite(rotate_up,ROTATE_PIN_INACTIVE_VALUE);}
        if (rotate_up_pwm)      {analogWrite(rotate_up_pwm,0);digitalWrite(rotate_up_pwm,LOW);}
        if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,0);}
        if (rotate_up_freq)     {noTone(rotate_up_freq);}
        if (rotate_up_or_down)  {digitalWrite(rotate_up_or_down,ROTATE_PIN_INACTIVE_VALUE);}
      } 
      break;
      
    case DOWN:
        #ifdef DEBUG_ROTATOR
        if (debug_mode) { Serial.print(F("ROTATION_DOWN ")); Serial.flush();}
        #endif //DEBUG_ROTATOR
        if (rotation_action == ACTIVATE) {
          #ifdef DEBUG_ROTATOR
          if (debug_mode) { Serial.println(F("ACTIVATE"));Serial.flush(); }
          #endif //DEBUG_ROTATOR
          brake_release(EL, true);
          if (el_slowstart_active) 
          {
            if (rotate_down_pwm)    {analogWrite(rotate_down_pwm,0);}
            if (rotate_up_pwm)      {analogWrite(rotate_up_pwm,0);digitalWrite(rotate_up_pwm,LOW);}
            if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,0);}
            if (rotate_up_freq)     {noTone(rotate_up_freq);}
            if (rotate_down_freq)   {noTone(rotate_down_freq);}
          } else 
          {
            if (rotate_down_pwm)    {analogWrite(rotate_down_pwm,normal_el_speed_voltage);}
            if (rotate_up_pwm)      {analogWrite(rotate_up_pwm,0);digitalWrite(rotate_up_pwm,LOW);}
            if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,normal_el_speed_voltage);}
            if (rotate_down_freq)   {tone(rotate_down_freq,map(normal_el_speed_voltage,0,255,EL_VARIABLE_FREQ_OUTPUT_LOW,EL_VARIABLE_FREQ_OUTPUT_HIGH));}
            if (rotate_up_freq)     {noTone(rotate_up_freq);}
          }          
          if (rotate_up)         {digitalWrite(rotate_up,ROTATE_PIN_INACTIVE_VALUE);}
          if (rotate_down)       {digitalWrite(rotate_down,ROTATE_PIN_ACTIVE_VALUE);}
          if (rotate_up_or_down) {digitalWrite(rotate_up_or_down,ROTATE_PIN_ACTIVE_VALUE);}
      } else 
      {
        #ifdef DEBUG_ROTATOR
        if (debug_mode) { Serial.println(F("DEACTIVATE"));Serial.flush(); }
        #endif //DEBUG_ROTATOR
        if (rotate_down)        {digitalWrite(rotate_down,ROTATE_PIN_INACTIVE_VALUE);}
        if (rotate_down_pwm)    {analogWrite(rotate_down_pwm,0);digitalWrite(rotate_down_pwm,LOW);}
        if (rotate_up_down_pwm) {analogWrite(rotate_up_down_pwm,0);}
        if (rotate_down_freq)   {noTone(rotate_down_freq);}
        if (rotate_up_or_down)  {digitalWrite(rotate_up_or_down,ROTATE_PIN_INACTIVE_VALUE);}
      }    
      break; 
     #endif //FEATURE_ELEVATION_CONTROL
  }  
  
  #ifdef DEBUG_ROTATOR
  if (debug_mode) 
  {
    Serial.println(F("rotator: exiting"));
    Serial.flush();
  }   
  #endif //DEBUG_ROTATOR  
}

//-----------------------------------------------------------------
// write to L298N motor control chip
// L298N is a dual H bridge motor controller
// IN1 defined as high side for positive rotation
// PWM on Teensy is 488.28 Hz
#ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
void rotator_speed(byte speed)
{
	if (speed > 0)
	{
		digitalWrite(IN1Pin,      1);
		digitalWrite(IN2Pin,      0);
		analogWrite( ENAPin,  speed);
	}
	if (speed < 0)
	{
		digitalWrite(IN1Pin,      1);
		digitalWrite(IN2Pin,      0);
		analogWrite( ENAPin, -speed);
	}
	if (speed == 0)
	{
		digitalWrite(IN1Pin,      0);
		digitalWrite(IN2Pin,      0);
		digitalWrite(ENAPin,      0);
	}
}
#endif

//--------------------------------------------------------------
void initialize_interrupts()
{ 
  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
  attachInterrupt(AZ_POSITION_PULSE_PIN_INTERRUPT, az_position_pulse_interrupt_handler, FALLING);
  #endif //FEATURE_AZ_POSITION_PULSE_INPUT
  
  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
  attachInterrupt(EL_POSITION_PULSE_PIN_INTERRUPT, el_position_pulse_interrupt_handler, FALLING);
  #endif //FEATURE_EL_POSITION_PULSE_INPUT 
}

//--------------------------------------------------------------
void initialize_pins()
{  
  if (serial_led) 
  {
    pinMode(serial_led, OUTPUT);
  } 
  if (overlap_led) 
  {
    pinMode(overlap_led, OUTPUT);
  }
  if (brake_az) 
  {
    // apparently, have to set output before setting output level
    // could lead to microsecond glitch on brake signal, not enough to pull relay
    pinMode(brake_az, OUTPUT);
    digitalWrite(brake_az, BRAKE_RELEASE_OFF);
  }
  if (az_speed_pot) 
  {
    pinMode(az_speed_pot, INPUT);
    digitalWrite(az_speed_pot, LOW);
  }
  if (az_preset_pot) 
  {
    pinMode(az_preset_pot, INPUT);
    digitalWrite(az_preset_pot, LOW);
  }
  
  if (preset_start_button) 
  {
    pinMode(preset_start_button, INPUT);
    digitalWrite(preset_start_button, HIGH);
  }  

  if (button_stop) 
  {
    pinMode(button_stop, INPUT);
    digitalWrite(button_stop, HIGH);
  }   

  #ifdef FEATURE_ELEVATION_CONTROL
  if (brake_el) 
  {
    // apparently, have to set output before setting output level
    // could lead to microsecond glitch on brake signal, not enough to pull relay
    pinMode(brake_el, OUTPUT);
    digitalWrite(brake_el, BRAKE_RELEASE_OFF);
  }  
  #endif //FEATURE_ELEVATION_CONTROL

  // Rotation control pins, ensure pins
  
  if (rotate_cw)         {pinMode(rotate_cw,         OUTPUT);}
  if (rotate_ccw)        {pinMode(rotate_ccw,        OUTPUT);}
  if (rotate_cw_pwm)     {pinMode(rotate_cw_pwm,     OUTPUT);}
  if (rotate_ccw_pwm)    {pinMode(rotate_ccw_pwm,    OUTPUT);}
  if (rotate_cw_ccw_pwm) {pinMode(rotate_cw_ccw_pwm, OUTPUT);}  
  if (rotate_motor)      {pinMode(rotate_motor,      OUTPUT);}
  if (rotate_cw_freq)    {pinMode(rotate_cw_freq,    OUTPUT);}
  if (rotate_ccw_freq)   {pinMode(rotate_ccw_freq,   OUTPUT);}  
  if (rotate_h1)         {pinMode(rotate_h1,         OUTPUT);}
  if (rotate_h2)         {pinMode(rotate_h2,         OUTPUT);}

  rotator(DEACTIVATE,  CW);
  rotator(DEACTIVATE, CCW);

  #ifndef FEATURE_AZ_POSITION_HMC5883L
  pinMode(rotator_analog_az, INPUT);
  #endif
  
  if (button_cw) 
  {
    pinMode(button_cw, INPUT);
    digitalWrite(button_cw, HIGH);
  }
  if (button_ccw) 
  {
    pinMode(button_ccw, INPUT);
    digitalWrite(button_ccw, HIGH);
  }
  
  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  
  if (azimuth_speed_voltage) 
  {                 // if azimuth_speed_voltage pin is configured, set it up for PWM output
    analogWrite(azimuth_speed_voltage, PWM_SPEED_VOLTAGE_X4);
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  pinMode(rotate_up, OUTPUT);
  pinMode(rotate_down, OUTPUT);
  if (rotate_up_or_down)  {pinMode(rotate_up_or_down,  OUTPUT);}
  if (rotate_up_pwm)      {pinMode(rotate_up_pwm,      OUTPUT);}
  if (rotate_down_pwm)    {pinMode(rotate_down_pwm,    OUTPUT);} 
  if (rotate_up_down_pwm) {pinMode(rotate_up_down_pwm, OUTPUT);}
  if (rotate_up_freq)     {pinMode(rotate_up_freq,     OUTPUT);}
  if (rotate_down_freq)   {pinMode(rotate_down_freq,   OUTPUT);}   
  rotator(DEACTIVATE,UP);
  rotator(DEACTIVATE,DOWN); 
  #ifdef FEATURE_EL_POSITION_POTENTIOMETER
  pinMode(rotator_analog_el, INPUT);
  #endif //FEATURE_EL_POSITION_POTENTIOMETER
  if (button_up) 
  {
    pinMode(button_up, INPUT);
    digitalWrite(button_up, HIGH);
  }
  if (button_down) 
  {
    pinMode(button_down, INPUT);
    digitalWrite(button_down, HIGH);
  }
  
  if (elevation_speed_voltage) 
  {                 // if elevation_speed_voltage pin is configured, set it up for PWM output
    analogWrite(elevation_speed_voltage, PWM_SPEED_VOLTAGE_X4);
    normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
    current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  }  
  
  read_elevation();
  #endif //FEATURE_ELEVATION_CONTROL
  
  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
  if (az_position_pulse_pin) 
  {
    pinMode(az_position_pulse_pin, INPUT);
    #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
    digitalWrite(az_position_pulse_pin, HIGH);
    #endif //OPTION_POSITION_PULSE_INPUT_PULLUPS    
  }
  #endif //FEATURE_AZ_POSITION_PULSE_INPUT
  
  
  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
  if (el_position_pulse_pin) 
  {
    pinMode(el_position_pulse_pin, INPUT);
    #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
    digitalWrite(el_position_pulse_pin, HIGH);
    #endif //OPTION_POSITION_PULSE_INPUT_PULLUPS
  }
  #endif //FEATURE_EL_POSITION_PULSE_INPUT  
  
  #ifdef FEATURE_PARK
  if (button_park)
  {
    pinMode(button_park, INPUT);
    digitalWrite(button_park, HIGH);
  }
  #endif //FEATURE_PARK
  
  #ifdef FEATURE_ROTATION_INDICATOR_PIN
  if (rotation_indication_pin)
  {
    pinMode(rotation_indication_pin, OUTPUT);
    digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_INACTIVE_STATE);
  }
  #endif //FEATURE_ROTATION_INDICATOR_PIN
  
  if (blink_led) 
  {
    pinMode(blink_led,OUTPUT);
  }
}  

//--------------------------------------------------------------
void initialize_serial()
{  
  Serial.begin(SERIAL_BAUD_RATE);
 
  #ifdef FEATURE_REMOTE_UNIT_SLAVE
  Serial.print(F("CS"));
  Serial.println(CODE_VERSION);
  #endif //FEATURE_REMOTE_UNIT_SLAVE
  
  #ifdef OPTION_SERIAL1_SUPPORT
  Serial1.begin(SERIAL1_BAUD_RATE);
  #endif //OPTION_SERIAL1_SUPPORT
  
  #ifdef OPTION_SERIAL2_SUPPORT
  Serial1.begin(SERIAL2_BAUD_RATE);
  #endif //OPTION_SERIAL2_SUPPORT
  
  #ifdef OPTION_SERIAL2_SUPPORT
  Serial1.begin(SERIAL2_BAUD_RATE);
  #endif //OPTION_SERIAL2_SUPPORT   
}

//--------------------------------------------------------------
void initialize_peripherals()
{  
  #ifdef FEATURE_WIRE_SUPPORT 
  Wire.begin();
  #endif
  
  #ifdef FEATURE_AZ_POSITION_HMC5883L
  compass = HMC5883L();
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if (error != 0) 
  {
    Serial.print(F("setup: compass error:"));
    Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
  }
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) 
  {
    Serial.print(F("setup: compass error:"));
    Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
  }
  #endif //FEATURE_AZ_POSITION_HMC5883L
  
  #ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
  accel = ADXL345();
  accel.SetRange(2, true);
  accel.EnableMeasurements();
  #endif //FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
  
  #ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
  accel.begin();
  #endif //FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
  
  #ifdef FEATURE_JOYSTICK_CONTROL
  pinMode(pin_joystick_x,INPUT);
  pinMode(pin_joystick_y,INPUT);  
  #endif //FEATURE_JOYSTICK_CONTROL
  
  #if defined(FEATURE_EL_POSITION_LSM303) || defined(FEATURE_AZ_POSITION_LSM303)
  if (!lsm.begin())
  {
    Serial.println(F("setup: LSM303 error"));
  }  
  #endif //FEATURE_EL_POSITION_LSM303 || FEATURE_AZ_POSITION_LSM303 
}

//--------------------------------------------------------------
// State Machine: Initialize, Start Up, Slow Down, Normal
// IDLE 
// SLOW_START_CW,                                 SLOW_START_CCW
// NORMAL_CW,                                         NORMAL_CCW 
// SLOW_DOWN_CW,                                   SLOW_DOWN_CCW 
// INITIALIZE_SLOW_START_CW,           INITIALIZE_SLOW_START_CCW
// INITIALIZE_TIMED_SLOW_DOWN_CW, INITIALIZE_TIMED_SLOW_DOWN_CCW 
// TIMED_SLOW_DOWN_CW,                       TIMED_SLOW_DOWN_CCW 
// INITIALIZE_DIR_CHANGE_TO_CW,     INITIALIZE_DIR_CHANGE_TO_CCW 
// INITIALIZE_NORMAL_CW,                   INITIALIZE_NORMAL_CCW 

// state transitions
//  Init normal cw -> normal cw
//  Init normal ccw -> normal ccw
//  Init slow start cw -> slow start cw
// INITIALIZE_SLOW_START_CCW      -> SLOW_START_CCW
// INITIALIZE_TIMED_SLOW_DOWN_CW  -> TIMED_SLOW_DOWN_CW
// INITIALIZE_TIMED_SLOW_DOWN_CCW -> TIMED_SLOW_DOWN_CVW
// INITIALIZE_DIR_CHANGE_TO_CW    -> TIMED_SLOW_DOWN_CCW
// INITIALIZE_DIR_CHANGE_TO_CCW   -> TIMED_SLOW_DOWN_CW
// SLOW_START_CW                  -> NORMAL_CW
// SLOW_START_CCW                 -> NORMAL_CCW


// manage state machine, call rotator()
void service_rotation_azimuth()
{   
  static byte az_direction_change_flag = 0;
  static byte az_initial_slow_down_voltage = 0;
  
  if (az_state == INITIALIZE_NORMAL_CW) 
  {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CW);        
    az_state = NORMAL_CW;
  }

  if (az_state == INITIALIZE_NORMAL_CCW) 
  {
    update_az_variable_outputs(normal_az_speed_voltage);
    rotator(ACTIVATE, CCW);        
    az_state = NORMAL_CCW;
  }
  
  if (az_state == INITIALIZE_SLOW_START_CW)
  {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE,CW);
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CW;
    #ifdef DEBUG_SERVICE_ROTATION
    if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_CW -> SLOW_START_CW"));}
    #endif //DEBUG_SERVICE_ROTATION
  }
  
  if (az_state == INITIALIZE_SLOW_START_CCW)
  {
    update_az_variable_outputs(AZ_SLOW_START_STARTING_PWM);
    rotator(ACTIVATE,CCW);    
    az_slowstart_start_time = millis();
    az_last_step_time = 0;
    az_slow_start_step = 0;
    az_state = SLOW_START_CCW;
    #ifdef DEBUG_SERVICE_ROTATION
    if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_CCW -> SLOW_START_CCW"));}
    #endif //DEBUG_SERVICE_ROTATION
  }  
  
  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CW) 
  {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS-1;
    az_state = TIMED_SLOW_DOWN_CW;
  }
  
  if (az_state == INITIALIZE_TIMED_SLOW_DOWN_CCW) 
  {
    az_direction_change_flag = 0;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS-1;
    az_state = TIMED_SLOW_DOWN_CCW;
  }  
  
  if (az_state == INITIALIZE_DIR_CHANGE_TO_CW) 
  {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS-1;
    az_state = TIMED_SLOW_DOWN_CCW;    
  }
  
  if (az_state == INITIALIZE_DIR_CHANGE_TO_CCW) 
  {
    az_direction_change_flag = 1;
    az_timed_slow_down_start_time = millis();
    az_last_step_time = millis();
    az_slow_down_step = AZ_SLOW_DOWN_STEPS-1;
    az_state = TIMED_SLOW_DOWN_CW;    
  }
  
  // slow start-------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW)) 
  { 
    if ((millis() - az_slowstart_start_time) >= AZ_SLOW_START_UP_TIME) 
    {  // is it time to end slow start?  
      #ifdef DEBUG_SERVICE_ROTATION
      if (debug_mode) {Serial.print(F("service_rotation: NORMAL_C"));}
      #endif //DEBUG_SERVICE_ROTATION
      if (az_state == SLOW_START_CW) 
      {
        az_state = NORMAL_CW;
        #ifdef DEBUG_SERVICE_ROTATION
        if (debug_mode) {Serial.println(F("W"));}
        #endif //DEBUG_SERVICE_ROTATION
      } else 
      {
        az_state = NORMAL_CCW;
        #ifdef DEBUG_SERVICE_ROTATION
        if (debug_mode) {Serial.println(F("CW"));}
        #endif //DEBUG_SERVICE_ROTATION
      }         
      update_az_variable_outputs(normal_az_speed_voltage); 
    } else // not ((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW)) 
    {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
      if (((millis() - az_last_step_time) > (AZ_SLOW_START_UP_TIME/AZ_SLOW_START_STEPS)) && (normal_az_speed_voltage > AZ_SLOW_START_STARTING_PWM))
      {
        #ifdef DEBUG_SERVICE_ROTATION
        if (debug_mode) 
        {
          Serial.print(F("service_rotation: step up: "));
          Serial.print(az_slow_start_step);
          Serial.print(F(" pwm: "));
          Serial.println((int)(AZ_SLOW_START_STARTING_PWM+((normal_az_speed_voltage-AZ_SLOW_START_STARTING_PWM)*((float)az_slow_start_step/(float)(AZ_SLOW_START_STEPS-1)))));
        }
        #endif //DEBUG_SERVICE_ROTATION
        update_az_variable_outputs((AZ_SLOW_START_STARTING_PWM+((normal_az_speed_voltage-AZ_SLOW_START_STARTING_PWM)*((float)az_slow_start_step/(float)(AZ_SLOW_START_STEPS-1)))));
        az_last_step_time = millis();  
        az_slow_start_step++;
      }   
    }    
  } //((az_state == SLOW_START_CW) || (az_state == SLOW_START_CCW))

  // timed slow down ------------------------------------------------------------------------------------------------------
  if (((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW)) && ((millis() - az_last_step_time) >= (TIMED_SLOW_DOWN_TIME/AZ_SLOW_DOWN_STEPS))) 
  {
    #ifdef DEBUG_SERVICE_ROTATION
    if (debug_mode) 
    {
      Serial.print(F("service_rotation: TIMED_SLOW_DOWN step down: "));
      Serial.print(az_slow_down_step);
      Serial.print(F(" pwm: "));
      Serial.println((int)(normal_az_speed_voltage*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS)));
    }
    #endif //DEBUG_SERVICE_ROTATION
    update_az_variable_outputs((int)(normal_az_speed_voltage*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS)));
    az_last_step_time = millis();  
    az_slow_down_step--;
    
    if (az_slow_down_step == 0) 
    { // is it time to exit timed slow down?
      #ifdef DEBUG_SERVICE_ROTATION
      if (debug_mode) {Serial.println(F("service_rotation: TIMED_SLOW_DOWN->IDLE"));}
      #endif //DEBUG_SERVICE_ROTATION
      rotator(DEACTIVATE,CW);
      rotator(DEACTIVATE,CCW);
      if (az_direction_change_flag) 
      {
        if (az_state == TIMED_SLOW_DOWN_CW) 
        {
          rotator(ACTIVATE,CCW);
          if (az_slowstart_active) 
          {
            az_state = INITIALIZE_SLOW_START_CCW;
          } else 
          {
            az_state = NORMAL_CCW;
          };
          az_direction_change_flag = 0;
        }
        if (az_state == TIMED_SLOW_DOWN_CCW) 
        {
          rotator(ACTIVATE,CW);
          if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CW;} else {az_state = NORMAL_CW;};
          az_direction_change_flag = 0;
        }
      } else 
      {
        az_state = IDLE;
        az_request_queue_state = NONE; 
      }           
    }
  }  //((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW))
  
  // slow down ---------------------------------------------------------------------------------------------------------------
  if ((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW)) 
  {     
    // is it time to do another step down?
    if (abs((target_raw_azimuth - raw_azimuth)/HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_AZ*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS))))
    {
      #ifdef DEBUG_SERVICE_ROTATION
      if (debug_mode) 
      {
        Serial.print(F("service_rotation: step down: "));
        Serial.print(az_slow_down_step);
        Serial.print(F(" pwm: "));
        Serial.println((int)(AZ_SLOW_DOWN_PWM_STOP+((az_initial_slow_down_voltage-AZ_SLOW_DOWN_PWM_STOP)*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS))));
      }
      #endif //DEBUG_SERVICE_ROTATION
      update_az_variable_outputs((AZ_SLOW_DOWN_PWM_STOP+((az_initial_slow_down_voltage-AZ_SLOW_DOWN_PWM_STOP)*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS))));
      az_slow_down_step--;      
    }   
  }  //((az_state == SLOW_DOWN_CW) || (az_state == SLOW_DOWN_CCW))
  
  // normal -------------------------------------------------------------------------------------------------------------------
  // if slow down is enabled, see if we're ready to go into slowdown
  if  ( ( (az_state == NORMAL_CW     ) || 
          (az_state == SLOW_START_CW ) || 
          (az_state == NORMAL_CCW    ) || 
          (az_state == SLOW_START_CCW)
        ) && 
        (az_request_queue_state == IN_PROGRESS_TO_TARGET) && 
        az_slowdown_active && 
        (abs((target_raw_azimuth - raw_azimuth)/HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_AZ)
      )  
  { 
    #ifdef DEBUG_SERVICE_ROTATION
    if (debug_mode) {Serial.print(F("service_rotation: SLOW_DOWN_C"));}   
    #endif //DEBUG_SERVICE_ROTATION    
    az_slow_down_step = AZ_SLOW_DOWN_STEPS-1;    
    if ((az_state == NORMAL_CW) || (az_state == SLOW_START_CW))
    {
      az_state = SLOW_DOWN_CW;
      #ifdef DEBUG_SERVICE_ROTATION
      if (debug_mode) {Serial.println(F("W"));}
      #endif //DEBUG_SERVICE_ROTATION
    } else 
    {
      az_state = SLOW_DOWN_CCW;
      #ifdef DEBUG_SERVICE_ROTATION
      if (debug_mode) {Serial.println(F("CW"));}
      #endif //DEBUG_SERVICE_ROTATION
    }
    if (AZ_SLOW_DOWN_PWM_START < current_az_speed_voltage) 
    {
      update_az_variable_outputs(AZ_SLOW_DOWN_PWM_START);
      az_initial_slow_down_voltage = AZ_SLOW_DOWN_PWM_START;
    } else 
    {
      az_initial_slow_down_voltage = current_az_speed_voltage;
    }
  }
  
  // check rotation target --------------------------------------------------------------------------------------------------------
  if  ( 
        (az_state != IDLE) && 
        (az_request_queue_state == IN_PROGRESS_TO_TARGET) 
      )  
  {
    if  (
          (az_state == NORMAL_CW) || 
          (az_state == SLOW_START_CW) || 
          (az_state == SLOW_DOWN_CW)
        )
    {
      if  (
            (abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
            (
              (raw_azimuth > target_raw_azimuth) && 
              ( (raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
            )
          ) 
      {
        delay(50);
        read_azimuth();
        if  (
              (abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
              ((raw_azimuth > target_raw_azimuth) && 
              ( (raw_azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
              )
            ) 
        {
          rotator(DEACTIVATE,CW);
          rotator(DEACTIVATE,CCW);
          az_state = IDLE;
          az_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
          if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
          #endif //DEBUG_SERVICE_ROTATION
        }
      }      
    } else 
    {
      if  ( 
            (abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER) ) || 
            ( (raw_azimuth < target_raw_azimuth) &&
              ( (target_raw_azimuth - raw_azimuth) < ( (AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
		        )
          ) 
      {
        delay(50);
        read_azimuth();
        if  (
              (abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
              (
                (raw_azimuth < target_raw_azimuth) && 
                ((target_raw_azimuth - raw_azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER))
              )
            ) 
        {
          rotator(DEACTIVATE,CW);
          rotator(DEACTIVATE,CCW);
          az_state = IDLE;
          az_request_queue_state = NONE;
          #ifdef DEBUG_SERVICE_ROTATION
          if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
          #endif //DEBUG_SERVICE_ROTATION
        }
      }        
    }
  }
} //service_rotation()

//--------------------------------------------------------------
// Called from loop(), before service_rotator_azimuth()
// if request in queue, process request
// queue means a single request, a pending operation
// REQUEST_STOP
// REQUEST_AZIMUTH, REQUEST_AZIMUTH_RAW
// REQUEST_CW, REQUEST_CCW, 
// REQUEST_UP, REQUEST_DOWN
// REQUEST_ELEVATION, REQUEST_KILL

void service_request_queue()
{ 
  int work_target_raw_azimuth = 0;
  byte direction_to_go = 0;
  
  if (az_request_queue_state == IN_QUEUE) 
  {  
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    if (debug_mode) {Serial.print(F("service_request_queue: AZ "));}
    #endif //DEBUG_SERVICE_REQUEST_QUEUE
    
    // REQUEST_STOP, REQUEST_AZIMUTH, REQUEST_AZIMUTH_RAW
    // REQUEST_CW, REQUEST_CCW, REQUEST_KILL
    switch(az_request)
    {
      case(REQUEST_STOP):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_STOP"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (az_state != IDLE)
        {
          if (az_slowdown_active) //request stop, not idle, slow down
          {
            if ((az_state == TIMED_SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CCW)) 
            {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE,CW);
              rotator(DEACTIVATE,CCW);
              az_state = IDLE;
              az_request_queue_state = NONE;            
            }
            if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW)) 
            {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CW;
              az_request_queue_state = IN_PROGRESS_TIMED; 
              az_last_rotate_initiation = millis();  
            }
            if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW)) 
            {
              az_state = INITIALIZE_TIMED_SLOW_DOWN_CCW;
              az_request_queue_state = IN_PROGRESS_TIMED; 
              az_last_rotate_initiation = millis();  
            }            
         
          } else // request stop, not idle, not slow down
          {
            rotator(DEACTIVATE,CW);
            rotator(DEACTIVATE,CCW);
            az_state = IDLE;
            az_request_queue_state = NONE;      
          }
        } else // request stop, az IDLE
        {
          az_request_queue_state = NONE; // nothing to do - we clear the queue 
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE        
        break; //REQUEST_STOP
        
      case(REQUEST_AZIMUTH):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_AZIMUTH"));} 
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if ((az_request_parm >= 0) && (az_request_parm <= (360*HEADING_MULTIPLIER))) 
        {
          target_azimuth = az_request_parm;
          target_raw_azimuth = az_request_parm;
          if (target_azimuth == (360*HEADING_MULTIPLIER)) {target_azimuth = 0;}      
          if ((target_azimuth > (azimuth - (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER))) &&
        	  (target_azimuth < (azimuth + (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER))))
          {
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) {Serial.print(F(" request within tolerance"));}
            #endif //DEBUG_SERVICE_REQUEST_QUEUE
          } else 
          {  // target azimuth is not within tolerance, we need to rotate
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) {Serial.print(F(" ->A"));}
            #endif //DEBUG_SERVICE_REQUEST_QUEUE
            work_target_raw_azimuth = target_azimuth;            
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) 
            {
              Serial.print(F(" work_target_raw_azimuth:"));
              Serial.print(work_target_raw_azimuth/HEADING_MULTIPLIER);
              Serial.print(F(" configuration.azimuth_starting_point:"));
              Serial.print(configuration.azimuth_starting_point);
              Serial.print(" ");
            }
            #endif //DEBUG_SERVICE_REQUEST_QUEUE            
            
            if (work_target_raw_azimuth < (configuration.azimuth_starting_point*HEADING_MULTIPLIER)) 
            {
              work_target_raw_azimuth = work_target_raw_azimuth + (360*HEADING_MULTIPLIER);
              target_raw_azimuth = work_target_raw_azimuth;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.print(F("->B"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            }
            if ((work_target_raw_azimuth + (360*HEADING_MULTIPLIER)) < 
                ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER)) 
            { // is there a second possible heading in overlap?
              if (abs(raw_azimuth - work_target_raw_azimuth) < abs((work_target_raw_azimuth+(360*HEADING_MULTIPLIER)) - raw_azimuth)) 
              { // is second possible heading closer?
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                if (debug_mode) {Serial.print(F("->C"));}
                #endif //DEBUG_SERVICE_REQUEST_QUEUE
                if (work_target_raw_azimuth  > raw_azimuth) 
                { // not closer, use position in non-overlap
                  direction_to_go = CW;                   
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  if (debug_mode) {Serial.println(F("->CW!"));}
                  #endif //DEBUG_SERVICE_REQUEST_QUEUE                  
                } else 
                {
                  direction_to_go = CCW;
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  if (debug_mode) {Serial.println(F("->CCW!"));}
                  #endif //DEBUG_SERVICE_REQUEST_QUEUE                    
                }          
              } else 
              { // go to position in overlap
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                if (debug_mode) {Serial.print(F("->D"));}
                #endif //DEBUG_SERVICE_REQUEST_QUEUE        
                target_raw_azimuth = work_target_raw_azimuth + (360*HEADING_MULTIPLIER);
                if ((work_target_raw_azimuth + (360*HEADING_MULTIPLIER)) > raw_azimuth) 
                {
                  direction_to_go = CW; 
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  if (debug_mode) {Serial.print(F("->CW!"));}
                  #endif //DEBUG_SERVICE_REQUEST_QUEUE                    
                } else 
                {         
                  direction_to_go = CCW;   
                  #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                  if (debug_mode) {Serial.print(F("->CCW!"));}
                  #endif //DEBUG_SERVICE_REQUEST_QUEUE                    
                }          
              }
            } else 
            {  // no possible second heading in overlap
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                if (debug_mode) {Serial.print(F("->E"));}
                #endif //DEBUG_SERVICE_REQUEST_QUEUE               
              if (work_target_raw_azimuth  > raw_azimuth) 
              {
                direction_to_go = CW;
              } else 
              {
                direction_to_go = CCW;
              }
            }      
          }
        } else 
        {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F("->F"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE             
          if ((az_request_parm > (360*HEADING_MULTIPLIER)) && (az_request_parm <= ((configuration.azimuth_starting_point+configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER))) 
          {
            target_azimuth = az_request_parm - (360*HEADING_MULTIPLIER);
            target_raw_azimuth = az_request_parm;
            if (az_request_parm > raw_azimuth) 
            {
              direction_to_go = CW;
            } else 
            {
              direction_to_go = CCW;
            }                     
          } else 
          {
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) 
            {
              Serial.print(F(" error: bogus azimuth request:"));
              Serial.println(az_request_parm);
            }
            #endif //DEBUG_SERVICE_REQUEST_QUEUE
            rotator(DEACTIVATE,CW);
            rotator(DEACTIVATE,CCW);
            az_state = IDLE;
            az_request_queue_state = NONE;            
            return;
          }
        }
        if (direction_to_go == CW)
        {
          if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)){
            az_state = INITIALIZE_DIR_CHANGE_TO_CW;
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CW"));}
            #endif //DEBUG_SERVICE_REQUEST_QUEUE
          } else 
          {          
            if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) 
            { // if we're already rotating CW, don't do anything
              //rotator(ACTIVATE,CW);
              if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CW;} else {az_state = INITIALIZE_NORMAL_CW;};     
            }     
          }
        }
        if (direction_to_go == CCW)
        {
          if ( ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active) )
          {
            az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
            #ifdef DEBUG_SERVICE_REQUEST_QUEUE
            if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CCW"));}
            #endif //DEBUG_SERVICE_REQUEST_QUEUE
          } else 
          {              
            if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) 
            { // if we're already rotating CCW, don't do anything
              //rotator(ACTIVATE,CCW);
              if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CCW;} else {az_state = INITIALIZE_NORMAL_CCW;}; 
            }
          }       
        }
        az_request_queue_state = IN_PROGRESS_TO_TARGET; 
        az_last_rotate_initiation = millis(); 
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_AZIMUTH    

      case(REQUEST_AZIMUTH_RAW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_AZIMUTH_RAW"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        target_raw_azimuth = az_request_parm;
        target_azimuth = target_raw_azimuth;
        if (target_azimuth >= (360*HEADING_MULTIPLIER)) {target_azimuth = target_azimuth - (360*HEADING_MULTIPLIER);}
        
        if (((abs(raw_azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER))) && (az_state == IDLE)) 
        {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F(" request within tolerance"));}          
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
          az_request_queue_state = NONE;
        } else 
        {
          if (target_raw_azimuth > raw_azimuth) 
          {
            if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active)){
              az_state = INITIALIZE_DIR_CHANGE_TO_CW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CW"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            } else 
            {              
              if ((az_state != INITIALIZE_SLOW_START_CW) && (az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) 
              { // if we're already rotating CW, don't do anything
                //rotator(ACTIVATE,CW);
                if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CW;} else {az_state = INITIALIZE_NORMAL_CW;};     
              }  
            }
          }
          if (target_raw_azimuth < raw_azimuth) 
          {
            if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active))
            {
              az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CCW"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            } else 
            {                  
              if ((az_state != INITIALIZE_SLOW_START_CCW) && (az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) 
              { // if we're already rotating CCW, don't do anything
               // rotator(ACTIVATE,CCW);
                if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CCW;} else {az_state = INITIALIZE_NORMAL_CCW;}; 
              }                
            }
          }   
          az_request_queue_state = IN_PROGRESS_TO_TARGET; 
          az_last_rotate_initiation = millis(); 
        }    
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_AZIMUTH_RAW
        
      case(REQUEST_CW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_CW"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) && (az_slowstart_active))
        {
          az_state = INITIALIZE_DIR_CHANGE_TO_CW;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CW"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
        } else 
        {
          if ((az_state != SLOW_START_CW) && (az_state != NORMAL_CW)) {
            //rotator(ACTIVATE,CW);
            if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CW;} else {az_state = INITIALIZE_NORMAL_CW;};
          }
        }
        az_request_queue_state = NONE;
        az_last_rotate_initiation = millis();
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_CW
        
      case(REQUEST_CCW):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println(F("REQUEST_CCW"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) && (az_slowstart_active))
        {
          az_state = INITIALIZE_DIR_CHANGE_TO_CCW;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F(" INITIALIZE_DIR_CHANGE_TO_CCW"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
        } else 
        {        
          if ((az_state != SLOW_START_CCW) && (az_state != NORMAL_CCW)) 
          {
            //rotator(ACTIVATE,CCW);
            if (az_slowstart_active) {az_state = INITIALIZE_SLOW_START_CCW;} else {az_state = INITIALIZE_NORMAL_CCW;};
          }
        }
        az_request_queue_state = NONE;  
        az_last_rotate_initiation = millis();
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_CCW
        
      case(REQUEST_KILL):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_KILL"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        rotator(DEACTIVATE,CW);
        rotator(DEACTIVATE,CCW);
        az_state = IDLE;
        az_request_queue_state = NONE;        
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_KILL        
    } // switch az request
  } // if az request in QUEUE
  
  #ifdef FEATURE_ELEVATION_CONTROL
  if (el_request_queue_state == IN_QUEUE)
  {
    #ifdef DEBUG_SERVICE_REQUEST_QUEUE
    if (debug_mode) {Serial.print(F("service_request_queue: EL "));}
    #endif //DEBUG_SERVICE_REQUEST_QUEUE
    switch(el_request) 
    {
      case(REQUEST_ELEVATION):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.print(F("REQUEST_ELEVATION "));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        target_elevation = el_request_parm;

        if (target_elevation > (ELEVATION_MAXIMUM_DEGREES*HEADING_MULTIPLIER))
        {
          target_elevation = ELEVATION_MAXIMUM_DEGREES * HEADING_MULTIPLIER;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F("REQUEST_ELEVATION: target_elevation > ELEVATION_MAXIMUM_DEGREES"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE          
        }
        
        #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
        if (target_elevation < (EL_MANUAL_ROTATE_DOWN_LIMIT*HEADING_MULTIPLIER))
        {
          target_elevation = EL_MANUAL_ROTATE_DOWN_LIMIT * HEADING_MULTIPLIER;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F("REQUEST_ELEVATION: target_elevation < EL_MANUAL_ROTATE_DOWN_LIMIT"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE          
        }
        if (target_elevation > (EL_MANUAL_ROTATE_UP_LIMIT*HEADING_MULTIPLIER))
        {
          target_elevation = EL_MANUAL_ROTATE_UP_LIMIT * HEADING_MULTIPLIER;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F("REQUEST_ELEVATION: target_elevation > EL_MANUAL_ROTATE_UP_LIMIT"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE          
        }               
        #endif //OPTION_EL_MANUAL_ROTATE_LIMITS
        
        if (abs(target_elevation - elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) 
        {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.println(F("requested elevation within tolerance"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
          //el_request_queue_state = NONE;
        } else 
        {
          if (target_elevation > elevation) 
          {
            if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active))
            {
              el_state = INITIALIZE_DIR_CHANGE_TO_UP;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.println(F(" INITIALIZE_DIR_CHANGE_TO_UP"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            } else {              
              if ((el_state != INITIALIZE_SLOW_START_UP) && (el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) 
              { // if we're already rotating UP, don't do anything
                if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_UP;} else {el_state = INITIALIZE_NORMAL_UP;};     
              }  
            }
          } //(target_elevation > elevation)
          if (target_elevation < elevation) 
          {
            if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active))
            {
              el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.println(F(" INITIALIZE_DIR_CHANGE_TO_DOWN"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            } else 
            {              
              if ((el_state != INITIALIZE_SLOW_START_DOWN) && (el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) 
              { // if we're already rotating DOWN, don't do anything
                if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_DOWN;} else {el_state = INITIALIZE_NORMAL_DOWN;};     
              }  
            }          
          }  //(target_elevation < elevation)         
        }  //(abs(target_elevation - elevation) < ELEVATION_TOLERANCE)
        el_request_queue_state = IN_PROGRESS_TO_TARGET;
        el_last_rotate_initiation = millis();        
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_ELEVATION
        
      case(REQUEST_UP):   
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println(F("REQUEST_UP"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) && (el_slowstart_active))
        {
          el_state = INITIALIZE_DIR_CHANGE_TO_UP;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.println(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_UP"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
        } else 
        {        
          if ((el_state != SLOW_START_UP) && (el_state != NORMAL_UP)) 
          {
            if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_UP;} else {el_state = INITIALIZE_NORMAL_UP;};
          }
        }
        el_request_queue_state = NONE;  
        el_last_rotate_initiation = millis();   
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_UP
        
      case(REQUEST_DOWN):         
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println(F("REQUEST_DOWN"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (((el_state == SLOW_START_UP) || (el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP)) && (el_slowstart_active))
        {
          el_state = INITIALIZE_DIR_CHANGE_TO_DOWN;
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.println(F("service_request_queue: INITIALIZE_DIR_CHANGE_TO_DOWN"));}
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
        } else 
        {        
          if ((el_state != SLOW_START_DOWN) && (el_state != NORMAL_DOWN)) 
          {
            if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_DOWN;} else {el_state = INITIALIZE_NORMAL_DOWN;};
          }
        }
        el_request_queue_state = NONE;  
        el_last_rotate_initiation = millis();       
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_DOWN    

      case(REQUEST_STOP):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println(F("REQUEST_STOP"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        if (el_state != IDLE)
        {
          if (el_slowdown_active) 
          {
            if ((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) 
            {  // if we're already in timed slow down and we get another stop, do a hard stop
              rotator(DEACTIVATE,UP);
              rotator(DEACTIVATE,DOWN);
              el_state = IDLE;
              el_request_queue_state = NONE;            
            }
            if ((el_state == SLOW_START_UP) || (el_state == NORMAL_UP)) 
            {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_UP;
              el_request_queue_state = IN_PROGRESS_TIMED; 
              el_last_rotate_initiation = millis();  
            }
            if ((el_state == SLOW_START_DOWN) || (el_state == NORMAL_DOWN)) 
            {
              el_state = INITIALIZE_TIMED_SLOW_DOWN_DOWN;
              el_request_queue_state = IN_PROGRESS_TIMED; 
              el_last_rotate_initiation = millis();  
            }                     
          } else 
          {
            rotator(DEACTIVATE,UP);
            rotator(DEACTIVATE,DOWN);
            el_state = IDLE;
            el_request_queue_state = NONE;      
          }
        } else 
        {       
          el_request_queue_state = NONE; //nothing to do, we're already in IDLE state
        }
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_STOP
        
      case(REQUEST_KILL):
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println(F("REQUEST_KILL"));}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE
        rotator(DEACTIVATE,UP);
        rotator(DEACTIVATE,DOWN);
        el_state = IDLE;
        el_request_queue_state = NONE;        
        #ifdef DEBUG_SERVICE_REQUEST_QUEUE
        if (debug_mode) {Serial.println();}
        #endif //DEBUG_SERVICE_REQUEST_QUEUE                
        break; //REQUEST_KILL           
    }    
  } //(el_request_queue_state == IN_QUEUE)
  #endif //FEATURE_ELEVATION_CONTROL  
}

//--------------------------------------------------------------
byte current_az_state()
{  
  if ((az_state  == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW))
  {
    return ROTATING_CW;
  }
  if ((az_state == SLOW_START_CCW) || (az_state == NORMAL_CCW) || (az_state == SLOW_DOWN_CCW) || (az_state == TIMED_SLOW_DOWN_CCW)) 
  {
    return ROTATING_CCW;
  }  
  return NOT_DOING_ANYTHING;
}

//--------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
byte current_el_state()
{
  if ((el_state == SLOW_START_UP)||(el_state == NORMAL_UP) || (el_state == SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_UP))
  {
    return ROTATING_UP;
  }
  if ((el_state == SLOW_START_DOWN)||(el_state == NORMAL_DOWN) || (el_state == SLOW_DOWN_DOWN) || (el_state == TIMED_SLOW_DOWN_DOWN)) 
  {
    return ROTATING_DOWN;
  }  
  return NOT_DOING_ANYTHING;  
}
#endif //FEATURE_ELEVATION_CONTROL

//--------------------------------------------------------------
#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
void az_position_pulse_interrupt_handler()
{
  #ifdef DEBUG_POSITION_PULSE_INPUT
  //az_position_pule_interrupt_handler_flag++;
  az_pulse_counter++;
  #endif //DEBUG_POSITION_PULSE_INPUT
  
  if (current_az_state() == ROTATING_CW) 
  {
    az_position_pulse_input_azimuth += AZ_POSITION_PULSE_DEG_PER_PULSE;
    last_known_az_state = ROTATING_CW;
  } else 
  {
    if (current_az_state() == ROTATING_CCW) 
    {
      az_position_pulse_input_azimuth -= AZ_POSITION_PULSE_DEG_PER_PULSE;
      last_known_az_state = ROTATING_CCW;
    } else 
    {
      if (last_known_az_state == ROTATING_CW)
      {
        az_position_pulse_input_azimuth += AZ_POSITION_PULSE_DEG_PER_PULSE;
      } else 
      {
        if (last_known_az_state == ROTATING_CCW)
        {
          az_position_pulse_input_azimuth -= AZ_POSITION_PULSE_DEG_PER_PULSE;
        }
      }
      #ifdef DEBUG_POSITION_PULSE_INPUT
      az_pulse_counter_ambiguous++;
      #endif //DEBUG_POSITION_PULSE_INPUT
    }
  }
  
  #ifdef OPTION_AZ_POSITION_PULSE_HARD_LIMIT
  if (az_position_pulse_input_azimuth < configuration.azimuth_starting_point) 
  {
    az_position_pulse_input_azimuth = configuration.azimuth_starting_point;
  }
  if (az_position_pulse_input_azimuth > (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)) 
  {
    az_position_pulse_input_azimuth = (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability);
  }
  #else
  if (az_position_pulse_input_azimuth < 0)
  {
    az_position_pulse_input_azimuth += 360;
  }
  if (az_position_pulse_input_azimuth >= 360)
  {
    az_position_pulse_input_azimuth -= 360;
  }
  #endif //OPTION_AZ_POSITION_PULSE_HARD_LIMIT
}
#endif //FEATURE_AZ_POSITION_PULSE_INPUT

//--------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
#ifdef FEATURE_EL_POSITION_PULSE_INPUT
void el_position_pulse_interrupt_handler()
{

  #ifdef DEBUG_POSITION_PULSE_INPUT
  //el_position_pule_interrupt_handler_flag++;
  el_pulse_counter++;
  #endif //DEBUG_POSITION_PULSE_INPUT
  
  if (current_el_state() == ROTATING_UP) 
  {
    el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
    last_known_el_state = ROTATING_UP;
  } else
  {
    if (current_el_state() == ROTATING_DOWN) 
    {
      el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
      last_known_el_state = ROTATING_DOWN;
    } else 
    {
      if (last_known_el_state == ROTATING_UP)
      {
        el_position_pulse_input_elevation += EL_POSITION_PULSE_DEG_PER_PULSE;
      } else 
      {
        if (last_known_el_state == ROTATING_DOWN){
          el_position_pulse_input_elevation -= EL_POSITION_PULSE_DEG_PER_PULSE;
        }
      }
      #ifdef DEBUG_POSITION_PULSE_INPUT
      el_pulse_counter_ambiguous++;
      #endif //DEBUG_POSITION_PULSE_INPUT
    }
  }   
  
  #ifdef OPTION_EL_POSITION_PULSE_HARD_LIMIT
  if (el_position_pulse_input_elevation < 0)
  {
    el_position_pulse_input_elevation = 0;
  }
  if (el_position_pulse_input_elevation > ELEVATION_MAXIMUM_DEGREES)
  {
    el_position_pulse_input_elevation = ELEVATION_MAXIMUM_DEGREES;
  }
  #endif //OPTION_EL_POSITION_PULSE_HARD_LIMIT
}
#endif //FEATURE_EL_POSITION_PULSE_INPUT
#endif //FEATURE_ELEVATION_CONTROL

//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
byte submit_remote_command(byte remote_command_to_send)
{ 
  if (remote_unit_command_submitted || suspend_remote_commands) 
  {
    return 0;
  } else 
  {
    switch(remote_command_to_send)
    {
      case REMOTE_UNIT_AZ_COMMAND:
        Serial1.println("AZ");
        if (remote_port_tx_sniff) {Serial.println("AZ");}
        remote_unit_command_submitted = REMOTE_UNIT_AZ_COMMAND;
        break;
      case REMOTE_UNIT_EL_COMMAND:
        Serial1.println("EL");
        if (remote_port_tx_sniff) {Serial.println("EL");}
        remote_unit_command_submitted = REMOTE_UNIT_EL_COMMAND;
        break;    
    }
    last_remote_unit_command_time = millis();   
    remote_unit_command_results_available = 0;
    return 1;
  }
}
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
byte is_ascii_number(byte char_in)
{ 
  if ((char_in > 47) && (char_in < 58)) 
  {
    return 1;
  } else 
  {
    return 0;
  }
}
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
void service_remote_communications_incoming_serial_buffer()
{
  byte good_data = 0;
  
  if (serial1_buffer_carriage_return_flag)
  {  
    #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
    if (debug_mode)
    {
      Serial.print(F("service_remote_communications_incoming_serial_buffer: serial1_buffer_index: "));
      Serial.print(serial1_buffer_index);
      Serial.print(F(" buffer: "));
      for (int x = 0;x < serial1_buffer_index;x++){
        Serial.write(serial1_buffer[x]);
      }
      Serial.println("$");
    }
    #endif //DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
    
    if (remote_unit_command_submitted)  
    {  // this was a solicited response
      switch(remote_unit_command_submitted)
      {
        case REMOTE_UNIT_AZ_COMMAND:
          if ((serial1_buffer_index == 7) && (serial1_buffer[0] == 'A') && (serial1_buffer[1] == 'Z') && 
          (is_ascii_number(serial1_buffer[2])) && (is_ascii_number(serial1_buffer[3])) && (is_ascii_number(serial1_buffer[4])) && (is_ascii_number(serial1_buffer[5])))
          {
            remote_unit_command_result_float = ((serial1_buffer[2]-48)*100) + ((serial1_buffer[3]-48)*10) + (serial1_buffer[4]-48) + ((serial1_buffer[5]-48)/10.0);           
            good_data = 1;
          }    
        break;
        case REMOTE_UNIT_EL_COMMAND:
          if ((serial1_buffer_index == 8) && (serial1_buffer[0] == 'E') && (serial1_buffer[1] == 'L') && 
          (is_ascii_number(serial1_buffer[3])) && (is_ascii_number(serial1_buffer[4])) && (is_ascii_number(serial1_buffer[5])) && (is_ascii_number(serial1_buffer[6])))
          {
            remote_unit_command_result_float = ((serial1_buffer[3]-48)*100) + ((serial1_buffer[4]-48)*10) + (serial1_buffer[5]-48) + ((serial1_buffer[6]-48)/10.0);           
            if (serial1_buffer[2] == '+') 
            {
              good_data = 1;
            }
            if (serial1_buffer[2] == '-') 
            {
              remote_unit_command_result_float = remote_unit_command_result_float * -1.0;
              good_data = 1;
            }             
          }         
        break;    
      }    
      if (good_data) 
      {
        remote_unit_command_results_available = remote_unit_command_submitted;
        remote_unit_good_results++;
        
        #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
        if (debug_mode)
        {
          Serial.print(F("service_remote_communications_incoming_serial_buffer: remote_unit_command_results_available: "));
          Serial.print(remote_unit_command_results_available);
          Serial.print(F(" remote_unit_command_result_float: "));
          Serial.println(remote_unit_command_result_float);
        }
        #endif //DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER        
        
        
      } else 
      {
        remote_unit_command_results_available = 0;
        remote_unit_bad_results++;
      }
      remote_unit_command_submitted = 0;           
    } else 
    {  // this was an unsolicited message
      
    }
    serial1_buffer_carriage_return_flag = 0;
    serial1_buffer_index = 0;
  }
 
  // has a command timed out?
  if ((remote_unit_command_submitted) && ((millis() - last_remote_unit_command_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS))
  {
    remote_unit_command_timeouts++; 
    remote_unit_command_submitted = 0;
    serial1_buffer_index = 0;
  }
  
  // have characters been in the buffer for some time but no carriage return?
  if ((serial1_buffer_index) && (!remote_unit_command_submitted) && ((millis() - serial1_last_receive_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS)) 
  {
    serial1_buffer_index = 0;
    remote_unit_incoming_buffer_timeouts++;
  }
}
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_AZIMUTH_CORRECTION
float correct_azimuth(float azimuth_in)
{
  if (sizeof(azimuth_calibration_from) != sizeof(azimuth_calibration_to))
  {
    return azimuth_in;
  }
  for (unsigned int x = 0;x < (sizeof(azimuth_calibration_from)-2);x++)
  {
    if ((azimuth_in >= azimuth_calibration_from[x]) && (azimuth_in <= azimuth_calibration_from[x+1]))
    {
      return (map(azimuth_in*10,azimuth_calibration_from[x]*10,azimuth_calibration_from[x+1]*10,azimuth_calibration_to[x]*10,azimuth_calibration_to[x+1]*10))/10.0;
    }
  }
  return(azimuth_in);
}
#endif //FEATURE_AZIMUTH_CORRECTION

//--------------------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CORRECTION
float correct_elevation(float elevation_in)
{
  if (sizeof(elevation_calibration_from) != sizeof(elevation_calibration_to))
  {
    return elevation_in;
  }
  for (unsigned int x = 0;x < (sizeof(elevation_calibration_from)-2);x++)
  {
    if ((elevation_in >= elevation_calibration_from[x]) && (elevation_in <= elevation_calibration_from[x+1]))
    {
      return (map(elevation_in*10,elevation_calibration_from[x]*10,elevation_calibration_from[x+1]*10,elevation_calibration_to[x]*10,elevation_calibration_to[x+1]*10))/10.0;
    }
  }
  return(elevation_in);
}
#endif //FEATURE_ELEVATION_CORRECTION

//--------------------------------------------------------------------------
#ifdef FEATURE_JOYSTICK_CONTROL
void check_joystick()
{
  int joystick_x = 0;
  int joystick_y = 0;

  static int joystick_resting_x = 0;
  static int joystick_resting_y = 0;

  static unsigned long last_joystick_az_action_time = 0;
  
  static byte joystick_azimuth_rotation = NOT_DOING_ANYTHING;

  #ifdef FEATURE_ELEVATION_CONTROL
  static byte joystick_elevation_rotation = NOT_DOING_ANYTHING;
  static unsigned long last_joystick_el_action_time = 0;  
  #endif //FEATURE_ELEVATION_CONTROL  

  if ((joystick_resting_x == 0) || (joystick_resting_y == 0)) 
  {  // initialize the resting readings if this is our first time here
  
    joystick_resting_x = analogRead(pin_joystick_x);
    joystick_resting_y = analogRead(pin_joystick_y);    
    
  } else 
  {           
    joystick_x = analogRead(pin_joystick_x);
    joystick_y = analogRead(pin_joystick_y);

    if ((millis() - last_joystick_az_action_time) > JOYSTICK_WAIT_TIME_MS) 
    {    
      #ifdef DEBUG_JOYSTICK
      static unsigned long last_debug_joystick_status = 0;
      
      if ((debug_mode) && ((millis() - last_debug_joystick_status) > 1000))
      {
        Serial.print("check_joystick: x: ");
        Serial.print(joystick_x);
        Serial.print("\ty: ");
        Serial.println(joystick_y); 
        last_debug_joystick_status = millis();
      }
      #endif //DEBUG_JOYSTICK
      
      #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
      if ((joystick_resting_x-joystick_x) < (joystick_resting_x * -0.2))
      {   // left
      #else
      if ((joystick_resting_x-joystick_x) > (joystick_resting_x * 0.2)) 
      {
      #endif
        #ifdef DEBUG_JOYSTICK
        if (debug_mode){Serial.println("check_joystick: L");}
        #endif //DEBUG_JOYSTICK
        if (current_az_state() != ROTATING_CCW) 
        {
          submit_request(AZ, REQUEST_CCW, 0);    
        }      
        joystick_azimuth_rotation = ROTATING_CCW; 
        last_joystick_az_action_time = millis();
        
      } else 
      {
        #ifndef OPTION_JOYSTICK_REVERSE_X_AXIS
        if ((joystick_resting_x-joystick_x) > (joystick_resting_x * 0.2)) {  // right
        #else
        if ((joystick_resting_x-joystick_x) < (joystick_resting_x * -0.2)) 
        {
        #endif
          #ifdef DEBUG_JOYSTICK
          if (debug_mode){Serial.println("check_joystick: R");}
          #endif //DEBUG_JOYSTICK
          if (current_az_state() != ROTATING_CW) 
          {
            submit_request(AZ,REQUEST_CW,0);        
          }      
          joystick_azimuth_rotation = ROTATING_CW;
          last_joystick_az_action_time = millis();
          
        } else 
        { // joystick is in X axis resting position
          if (joystick_azimuth_rotation != NOT_DOING_ANYTHING) 
          {
            if (current_az_state() != NOT_DOING_ANYTHING) 
            {
              submit_request(AZ,REQUEST_STOP,0);
              last_joystick_az_action_time = millis();
            }
            joystick_azimuth_rotation = NOT_DOING_ANYTHING;
          }
        }
      }
    }
     
    if ((millis() - last_joystick_el_action_time) > JOYSTICK_WAIT_TIME_MS) 
    {         
      #ifdef FEATURE_ELEVATION_CONTROL
      #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
      if ((joystick_resting_y-joystick_y) > (joystick_resting_y * 0.2)) 
      {  // down
      #else
      if ((joystick_resting_y-joystick_y) < (joystick_resting_y * -0.2)) 
      {
      #endif
        #ifdef DEBUG_JOYSTICK
        if (debug_mode){Serial.println("check_joystick: D");}
        #endif //DEBUG_JOYSTICK
        if (current_el_state() != ROTATING_DOWN) 
        {
          submit_request(EL,REQUEST_DOWN,0);
        }
        joystick_elevation_rotation = ROTATING_DOWN;
        last_joystick_el_action_time = millis();        
      } else 
      {
        #ifndef OPTION_JOYSTICK_REVERSE_Y_AXIS
        if ((joystick_resting_y-joystick_y) < (joystick_resting_y * -0.2)) 
        { // up
        #else
        if ((joystick_resting_y-joystick_y) > (joystick_resting_y * 0.2)) 
        {
        #endif
          #ifdef DEBUG_JOYSTICK
          if (debug_mode){Serial.println("check_joystick: U");}
          #endif //DEBUG_JOYSTICK
          if (current_el_state() != ROTATING_UP) 
          {
            submit_request(EL,REQUEST_UP,0);        
          }
          joystick_elevation_rotation = ROTATING_UP;
          last_joystick_el_action_time = millis();
          
        } else 
        {  // Y axis is in resting position
          if (joystick_elevation_rotation != NOT_DOING_ANYTHING) 
          {
            if (current_el_state() != NOT_DOING_ANYTHING) 
            {
              submit_request(EL,REQUEST_STOP,0);
              last_joystick_el_action_time = millis();
            }
            joystick_elevation_rotation = NOT_DOING_ANYTHING;
          }        
        }
      }
      #endif //FEATURE_ELEVATION_CONTROL
    }
  }
}
#endif //FEATURE_JOYSTICK_CONTROL

//-------------------------------------------------------------------------- 
#ifdef FEATURE_ROTATION_INDICATOR_PIN
void service_rotation_indicator_pin()
{
  static byte rotation_indication_pin_state = 0;
  static unsigned long time_rotation_went_inactive = 0;
  
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((!rotation_indication_pin_state) && ((az_state != IDLE) || (el_state != IDLE)))
  {
  #else
  if ((!rotation_indication_pin_state) && ((az_state != IDLE)))
  {  
  #endif
    if (rotation_indication_pin)
    {
      digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_ACTIVE_STATE);
    }
    rotation_indication_pin_state = 1; 
    #ifdef DEBUG_ROTATION_INDICATION_PIN
    if (debug_mode){Serial.println(F("service_rotation_indicator_pin: active"));}
    #endif
  }
  
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((rotation_indication_pin_state) && (az_state == IDLE) && (el_state == IDLE)){
  #else
  if ((rotation_indication_pin_state) && (az_state == IDLE))
  {  
  #endif
    if (time_rotation_went_inactive == 0)
    {
      time_rotation_went_inactive = millis();
    } else 
    {
      if ((millis() - time_rotation_went_inactive) >= ((ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS * 1000)+(ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES * 60 * 1000)))
      {
        if (rotation_indication_pin)
        {
          digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_INACTIVE_STATE);
        }
        rotation_indication_pin_state = 0;   
        time_rotation_went_inactive = 0; 
        #ifdef DEBUG_ROTATION_INDICATION_PIN
        if (debug_mode){Serial.println(F("service_rotation_indicator_pin: inactive"));}
        #endif    
      }
    }
  } 
}
#endif //FEATURE_ROTATION_INDICATOR_PIN  

