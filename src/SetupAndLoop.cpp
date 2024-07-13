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
#include <Arduino.h>

#include "dependencies.h"
#include "initialize_functions.h"

extern FIR<float, 31> lpf_top;
extern FIR<float, 31> lpf_bot;

// loop functions
void az_check_operation_timeout();
void check_overlap();
void profile_loop_time();
void service_blink_led();
void check_hco_buttons();
void check_az_manual_rotate_limit();

// define external functions
void read_headings();
void TimedService();

// define debounced buttons
Bounce debounceCCW = Bounce();
Bounce debounceCW  = Bounce();

void setup() 
{
  delay(500);

  // setup debounded rotation control buttons
  debounceCCW.attach(button_ccw_pin, INPUT_PULLUP);
  debounceCW.attach( button_cw_pin,  INPUT_PULLUP);
  debounceCCW.interval(20); // msec, wait time to settle
  debounceCW.interval( 20);

  initialize_serial();
  initialize_peripherals();
  read_settings_from_eeprom(); 
  initialize_pins();
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

  lpf_top.setFilterCoeffs(filter_taps);
  lpf_bot.setFilterCoeffs(filter_taps);
  #endif // FIR filter

  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TIME_BETWEEN_AZ_ADC_READ, TimedService); // interval, function call
  // interrupts enabled after this point
  MsTimer2::start(); 
} //setup()

/*---------------- here's where the magic happens --------------------*/
// functions mostly communicate via global variables
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

  #ifdef FEATURE_HCO_BUTTONS
  check_hco_buttons();
  #else
  check_buttons();
  #endif

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

  #ifdef FEATURE_MAX7221_DISPLAY
  update_Az_MAX7221_display();
  #endif

  read_headings();
  
  #ifndef FEATURE_REMOTE_UNIT_SLAVE
  check_az_manual_rotate_limit();

  #ifdef OPTION_EL_MANUAL_ROTATE_LIMITS
  check_el_manual_rotate_limit();
  #endif
 
  #ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
  check_az_speed_pot();
  #endif
  
  #ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder or Preset Selector
  check_preset_encoders();
  #endif //FEATURE_AZ_PRESET_ENCODER

  #ifdef FEATURE_AZ_PRESET_POT
  check_az_preset_potentiometer();
  #endif //FEATURE_AZ_PRESET_POT

  #endif //ifndef FEATURE_REMOTE_UNIT_SLAVE
  
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

//--------------------------------------------------------------
// check and act on button presses
// button status read from global variables counting press time
// Need to have press time greater than about 100 msec to get past bounce

// if CW pressed and rotator near max limit and CCW long pressed, then calibrate fullscale
// if CCW pressed and rotator near max limit and CW long pressed, then calibrate offse
// function called from ISR at TIME_BETWEEN_AZ_ADC_READ msec
void check_hco_buttons()
{
  static bool is_fullscale_cal_mode = false;
  static bool is_offset_cal_mode    = false;

  debounceCCW.update();
  debounceCW.update();

  // Rotation ---------------------------------------------------------------
  // check for cw button press and not ccw button
  if (debounceCW.fell() && debounceCCW.read()) 
  {
    if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER)) 
    {
      submit_request(AZ, REQUEST_CW, 0); // on first detection of press
    } else 
    {
      #ifdef DEBUG_HCO_BUTTONS
      if (debug_mode) 
      {
        Serial.print(F("check_hco_buttons: exceeded CW_LIMIT"));
        Serial.print("raw azimuth ");
        Serial.print(raw_azimuth);
        Serial.print(" !< CW limit ");
        Serial.println(AZ_MANUAL_ROTATE_CW_LIMIT * HEADING_MULTIPLIER);
      }
      #endif
    }

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_hco_buttons: button_cw_fell"));}       
    #endif
  } // CW fell, CCW high

  // check for ccw button and not cw button
  if (debounceCCW.fell() && debounceCW.read()) // only ccw pressed
  {
    if (raw_azimuth > (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER)) 
    {
      submit_request(AZ, REQUEST_CCW, 0); // on first detection of press
    } else 
    {
      #ifdef DEBUG_HCO_BUTTONS
      if (debug_mode)
      {
        Serial.print(F("check_hco_buttons: exceeded CCW_LIMIT, "));
        Serial.print("raw azimuth ");
        Serial.print(raw_azimuth);
        Serial.print(" !> CCW limit ");
        Serial.println(AZ_MANUAL_ROTATE_CCW_LIMIT * HEADING_MULTIPLIER);
      }
      #endif
    }

    #ifdef DEBUG_HCO_BUTTONS
    Serial.println(F("check_hco_buttons: CCW_fell"));      
    #endif
  } //CCW fell, CW high

  // handle release of cw button press 
  if (debounceCW.rose() && debounceCCW.read())
  {
    submit_request(AZ, REQUEST_STOP, 0);

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_buttons: CW rose, CCW high"));}
    #endif

  } // if cw button released

  // handle release of ccw button press 
  if (debounceCCW.rose() && debounceCW.read())
  {
    submit_request(AZ, REQUEST_STOP,0);

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_buttons: CCW rose, CW high"));}
    #endif

  } // if ccw button released

  // -------------Calibration ------------------------------------
  // Set fullscale calibration mode, rotate to full cw, hold while press ccw button for 2 seconds
  if ((button_cw_press_time > button_ccw_press_time) && (button_ccw_press_time > BUTTON_LONG_PRESS))
  {
    is_fullscale_cal_mode = true;
  }

  // Set offset calibration mode, rotate to full ccw, hold while press cw button for 2 seconds
  if ((button_ccw_press_time > button_cw_press_time) && (button_cw_press_time > BUTTON_LONG_PRESS))
  {
    is_offset_cal_mode = true;
  }

  // handle release of other button (ccw), at end of full scale (cw) calibration
  if ( is_fullscale_cal_mode && (button_ccw_press_time == 0))
  {
    is_fullscale_cal_mode = false;

    configuration.analog_az_full_cw = analog_az; // azimuth before mapping
    write_settings_to_eeprom();  // write the cal
    print_wrote_to_memory();     // message about updating cal eeprom
    read_settings_from_eeprom(); // print on serial port if debugging on
  }
 
  // handle release of other button (cw), at end of offset (cvw) calibration
  if ( is_offset_cal_mode && (button_cw_press_time == 0))
  {
    is_offset_cal_mode = false;

    configuration.analog_az_full_ccw = analog_az; // azimuth before mapping
    write_settings_to_eeprom();  // write the cal
    print_wrote_to_memory();     // message about updating cal eeprom
    read_settings_from_eeprom(); // print on serial port if debugging on
  }
} // check_hco_buttons()

