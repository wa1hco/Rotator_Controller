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
#include "rotator_pins_HCO_board.h"
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
#include "Input.h"

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
        	(abs(new_pot_azimuth - (AzFiltered/HEADING_MULTIPLIER)) > AZIMUTH_TOLERANCE)) 
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
// check and act on button presses
// button status read from global variables counting press time
// Need to have press time greater than about 100 msec to get past bounce

// if CW pressed and rotator near max limit and CCW long pressed, then calibrate fullscale
// if CCW pressed and rotator near max limit and CW long pressed, then calibrate offset

// function called from ISR at TIME_BETWEEN_INTERRUPTS msec
void check_hco_buttons()
{
  #ifdef DEBUG_HCO_BUTTONS
  Serial.print("check_hco_buttons: ");
  #endif

  #ifdef DEBUG_HCO_BUTTONS
  Serial.print("CW: ");
  Serial.print(digitalRead(button_cw));
  Serial.print(" ");
  #endif

  // check for clockwise rotation and not ccw rotation
  if ((button_cw_press_time > BUTTON_BOUNCE_DELAY) && (button_ccw_press_time == 0)) // might be first press or continued press
    {
    if (!is_cw_button_pressed) // still low
    {
      #ifdef DEBUG_HCO_BUTTONS
      if (debug_mode) 
      {
        Serial.println(F("check_buttons: button_cw pushed"));
      }       
      #endif

      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER)) 
      {
      #endif    

      submit_request(AZ, REQUEST_CW, 0); // on first detection of press
      is_cw_button_pressed = true;

      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      } else 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CW_LIMIT"));}
        #endif //DEBUG_BUTTONS
      }
      #endif            
    }
  } // if button_cw

  #ifdef DEBUG_HCO_BUTTONS
  Serial.print("CCW: ");
  Serial.print(digitalRead(button_ccw));
  Serial.print(" ");
  #endif

  // check for counter clockwise rotation and cw button not pressed
  if ((button_ccw_press_time > BUTTON_BOUNCE_DELAY) && (button_cw_press_time == 0))
  {
    if (!is_ccw_button_pressed) // still low
    {
      #ifdef DEBUG_HCO_BUTTONS
      Serial.println(F("check_buttons: button_ccw pushed"));      
      #endif

      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      if (raw_azimuth > (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER)) 
      {
      #endif  

      submit_request(AZ, REQUEST_CCW, 0); // on first detection of press
      is_ccw_button_pressed = true;

      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      } else 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CCW_LIMIT"));}
        #endif //DEBUG_BUTTONS
      }
      #endif //OPTION_AZ_MANUAL_ROTATE_LIMITS      
    }
  } // if button pressed

  #ifdef DEBUG_HCO_BUTTONS
  Serial.print("isAzPressed ");
  Serial.print(isAzButtonPressed);
  Serial.print(" ");
  #endif

  // handle release of cw button press
  // if button was press and no buttons pressed now
  if (is_cw_button_pressed && (button_cw_press_time == 0))
  {
      #ifdef DEBUG_HCO_BUTTONS
      Serial.println(F("check_buttons: no AZ button depressed"));
      #endif

      submit_request(AZ, REQUEST_STOP,0);
      is_cw_button_pressed = false;
  } // if cw button released

  if ( is_cw_button_cal_press && (button_ccw_press_time == 0))
  {
    configuration.analog_az_full_cw = analog_az; // azimuth is a global
    write_settings_to_eeprom();
    print_wrote_to_memory();   
    read_settings_from_eeprom(); // print on serial port if debugging on
    is_cw_button_cal_press = false;
  }

  if ( is_ccw_button_cal_press && (button_cw_press_time == 0))
  {
    configuration.analog_az_full_ccw = analog_az; // azimuth is a global
    write_settings_to_eeprom();
    print_wrote_to_memory();  
    read_settings_from_eeprom();  // print on serial port if debugging on
    is_ccw_button_cal_press = false;
  }

  // handle release of ccw button press for normal rotation
  if (is_ccw_button_pressed && (button_ccw_press_time == 0)) 
  {
    if ((digitalRead(button_ccw) == HIGH) && (digitalRead(button_cw) == HIGH)) // still no buttons pressed
    {
      #ifdef DEBUG_HCO_BUTTONS
      Serial.println(F("check_buttons: no AZ button depressed"));
      #endif

      submit_request(AZ, REQUEST_STOP,0);
      is_ccw_button_pressed = false;

      is_ccw_button_cal_press = false;
    }
  } // if ccw button released
 
  // calibration buttons
  // hold cw or ccw to end, continue holding while press and hold other button for 2 seconds
  // handle fullscale calibration
  // ccw button pressed longer than cw, long press on cw button
  if ((button_cw_press_time > button_ccw_press_time) && (button_ccw_press_time > BUTTON_LONG_PRESS))
  {
    is_cw_button_cal_press = true;
  }

  // handle offset calibration
  // cw button pressed longer than ccw, long press on ccw button
  if ((button_ccw_press_time > button_cw_press_time) && (button_cw_press_time > BUTTON_LONG_PRESS))
  {
    is_ccw_button_cal_press = true;
  }

  #ifdef DEBUG_HCO_BUTTONS
  Serial.println();
  #endif

} // check_hco_buttons()

//--------------------------------------------------------------
// check and act on button presses
void check_buttons()
{
  #if defined(FEATURE_ADAFRUIT_BUTTONS)
  int buttons = 0;
  buttons = readButtons();
  if (buttons & BUTTON_RIGHT) 
  {
  #else // not adafruit
  if (button_cw && (digitalRead(button_cw) == LOW)) 
  {
  #endif //FEATURE_ADAFRUIT_BUTTONS

    if (!isAzButtonPressed) 
    {
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {Serial.println(F("check_buttons: button_cw pushed"));}       
      #endif //DEBUG_BUTTONS
      #ifdef OPTION_AZ_MANUAL_ROTATE_LIMITS
      if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER)) 
      {
      #endif      
      submit_request(AZ,REQUEST_CW, 0);
      isAzButtonPressed = true;
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

    #else
    if (button_ccw && (digitalRead(button_ccw) == LOW)) 
    {
    #endif //FEATURE_ADAFRUIT_BUTTONS
      if (!isAzButtonPressed) 
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
        isAzButtonPressed = true;
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

  #if defined(FEATURE_ADAFRUIT_BUTTONS)
  if ((isAzButtonPressed) && (!(buttons & 0x12))) 
  {
    #ifdef DEBUG_BUTTONS
    if (debug_mode) 
    {
      Serial.println(F("check_buttons: no button depressed"));
    }    
    #endif // DEBUG_BUTTONS
    submit_request(AZ,REQUEST_STOP,0);
    isAzButtonPressed = 0;
  }

  #else // not adafruit buttons
  if ((isAzButtonPressed) && (digitalRead(button_ccw) == HIGH) && (digitalRead(button_cw) == HIGH)) 
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
      //submit_request(AZ, REQUEST_STOP,0);
      isAzButtonPressed = false;
    }
  }
  #endif //adafruit or directly connected buttons

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
