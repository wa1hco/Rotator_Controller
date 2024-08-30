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

// if both buttons pressed, check for long press indicating calibration
// if CW pressed first and CCW long pressed, then calibrate full CW (fullscale)
// if CCW pressed first and CW long pressed, then calibrate full CCW (offset)
// function called from ISR at TIME_BETWEEN_AZ_ADC_READ msec
void check_hco_buttons()
{
  extern bool is_CW_cal_mode;
  extern bool is_CCW_cal_mode;

  // button state, pressed = low, fell = pressed, rose = released
  // if board marked V3, then pressed = low
  // if board after V3, push button switch is normally closed to ground
  #define     PRESSED HIGH
  #define NOT_PRESSED LOW

  debounceCCW.update();
  debounceCW.update();

  // Rotation ---------------------------------------------------------------
  // buttons say rotate CW
  if (debounceCW.changed() &&
      debounceCW.read() == PRESSED && 
      debounceCCW.read() == NOT_PRESSED)
  {
    submit_request(AZ, REQUEST_CW, 0); // on first detection of press

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_hco_buttons: CW fell, CCW not pressed"));}       
    #endif

    return;
  } // rotate CW

  // buttons say rotate CCW
  if (debounceCCW.changed() &&
      debounceCCW.read() == PRESSED && 
      debounceCW.read() == NOT_PRESSED) // only ccw pressed
  {
    submit_request(AZ, REQUEST_CCW, 0); // on first detection of press

    #ifdef DEBUG_HCO_BUTTONS
    Serial.println(F("check_hco_buttons: CCW_fell, CW not pressed"));      
    #endif

    return;
  } // rotate CCW

  // buttons say stop CW rotation
  if (debounceCW.changed() &&
      debounceCW.read() == NOT_PRESSED
      && debounceCCW.read() == NOT_PRESSED)
  {
    submit_request(AZ, REQUEST_STOP, 0);

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_buttons: CW rose, CCW not pressed"));}
    #endif

    return;
  } // stop CW

  // buttons say stop CCW rotation
  if (debounceCCW.changed() &&
      debounceCCW.read() == NOT_PRESSED && 
      (debounceCW.read() == NOT_PRESSED))
  {
    submit_request(AZ, REQUEST_STOP, 0);

    #ifdef DEBUG_HCO_BUTTONS
    if (debug_mode) {Serial.println(F("check_buttons: CCW rose, CW not pressed"));}
    #endif

    return;
  } // stop CCW

  #ifdef FEATURE_HCO_BUTTON_CALIBRATION
  // -------------Calibration using HCO_BUTTONS ------------------------------------
  // rotate to limit of travel
  // continue to press and hold button in direction of travel
  // press and hold other button for >BUTTON_LONG_PRESS millisec
  // Display indicates calibration mode
  // Calibration complete when either button released

  // Set fullscale calibration modes based on button presses
  // rotate to full cw, hold while press ccw button for 2 seconds
  if (debounceCW.read() == PRESSED && debounceCCW.read() == PRESSED) //both buttons pressed
  { 
    submit_request(AZ, REQUEST_STOP, 0); // either calibration or confusion, so stop

    if (debounceCW.duration() >= debounceCCW.duration() && // CW pressed first
        debounceCCW.duration() > BUTTON_LONG_PRESS &&      // CCW long press
        !is_CCW_cal_mode &&
        !is_CW_cal_mode)
    {
      is_CW_cal_mode = true; // put in calibration mode

      #ifdef DEBUG_HCO_BUTTON_CALIBRATION
      Serial.print("check_hco_buttons, fullscale cal buttons:");
      Serial.print(", CW.duration ");
      Serial.print(debounceCCW.duration());
      Serial.print(", CCW.duration");
      Serial.print(debounceCCW.duration());
      Serial.print(", azimuth ");
      Serial.print(azimuth);
      Serial.println();
      #endif

      return;
    } // if CW cal mode

    // rotate to full ccw, hold while press cw button for 2 seconds
    if (debounceCCW.duration() > debounceCW.duration() && // CCW pressed first
        debounceCW.duration() > BUTTON_LONG_PRESS && // CW long press
        !is_CW_cal_mode &&
        !is_CCW_cal_mode)
    {
      is_CCW_cal_mode = true;

      #ifdef DEBUG_HCO_BUTTON_CALIBRATION
      Serial.print("check_hco_buttons, fullscale cal buttons:");
      Serial.print(", CW.duration ");
      Serial.print(debounceCW.duration());
      Serial.print(", CCW.duration");
      Serial.print(debounceCCW.duration());
      Serial.print(", azimuth ");
      Serial.print(azimuth);
      Serial.println();
      #endif

      return;
    } // if CCW cal mode
  } // both buttons pressed

  // Unset calibration modes and calibrate after both buttons released
  // calibrate when full scale (cw) calibration and release of both buttons
  if (is_CW_cal_mode && (debounceCCW.read() == NOT_PRESSED && debounceCW.read() == NOT_PRESSED) )
  {
    is_CW_cal_mode = false;

    configuration.Raz_full_cw = Raz; // azimuth before mapping
    write_settings_to_eeprom();  // write the cal
    print_wrote_to_memory();     // message about updating cal eeprom
    read_settings_from_eeprom(); // print on serial port if debugging on

    #ifdef DEBUG_HCO_BUTTON_CALIBRATION
    Serial.print("check_hco_buttons: ");
    Serial.print("fullscale calibrate, ");
    Serial.print("Rtop ");
    Serial.print(Raz);
    Serial.print(" azimuth ");
    Serial.print(azimuth);
    Serial.println();
    #endif
    return;
  } // buttons released, cal CW
 
  // calibrate when offset (cww) calibration and release of both button
  if (is_CCW_cal_mode && (debounceCCW.read() == NOT_PRESSED && debounceCW.read() == NOT_PRESSED) )
  {
    is_CCW_cal_mode = false;

    configuration.Raz_full_ccw = Raz; // azimuth before mapping
    write_settings_to_eeprom();  // write the cal
    print_wrote_to_memory();     // message about updating cal eeprom
    read_settings_from_eeprom(); // print on serial port if debugging on
  
    #ifdef DEBUG_HCO_BUTTON_CALIBRATION
    Serial.print("check_hco_buttons: offset calibration");
    Serial.print(", Raz ");
    Serial.print(Raz);
    Serial.print(", azimuth ");
    Serial.print(azimuth);
    Serial.println();
    #endif

    return;
} // buttons released, cal CCW
  #endif // FEATURE_HCO_BUTTON_CALIBRATION

} // check_hco_buttons()

