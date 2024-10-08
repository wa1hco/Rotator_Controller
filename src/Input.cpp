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

*/

// Arduino environment
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <avr/io.h>

// Project configuration
#include "dependencies.h"

// Project functions

//--------------------------------------------------------------
void check_az_speed_pot() 
{
#ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
  static unsigned long last_az_preset_check_time = 0;
  int pot_read = 0;
  byte new_azimuth_speed_voltage = 0;
   
  if (az_speed_pot && azimuth_speed_voltage && ((millis() - last_az_preset_check_time) > 500))
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
    last_az_preset_check_time = millis();  
  }
#endif
}

//-------------------------------Azimuth Preset-------------------------------
// Turn Azimuth preset pot knob to initiate rotor movement
// Movement starts when pot stops turning and direction of movement is known
// Display indicates destination azimuth for a few sec when turning detected
// 7 segment display has indication of preset display mode, decimal points, flicker, something
// On startup, no motion is detected, preset value is not used
// pressing CW or CCW button stops preset movement
// read the preset pot if preset button or time to read

// deal with first indication of change and as pot continues to change
// this function is call from loop()
void check_az_preset_potentiometer()
{
  #ifdef FEATURE_AZ_PRESET_POT
  int                   millis_now;
  static unsigned long  last_az_preset_check_time = 0;
  int                   az_preset_interval        = 0;  // will not be zero at first check
  static int            turning_stopped_count     = AZ_PRESET_STOP_COUNT; 
  static int            last_pot_read             = 0;
  int                   pot_read                  = 0;  // current pot setting
  int                   pot_rate                  = 0;

  if (!az_preset_pot) // if az preset pot pin not defined
  {  
    return;
  }

  if (last_az_preset_check_time == 0) // first pass through check az preset
  {
    last_pot_read = analogRead(az_preset_pot);
    last_az_preset_check_time = millis();

    #ifdef DEBUG_AZ_PRESET_POT
    Serial.print("check_az_preset, first check: ");
    Serial.print("millis ");
    Serial.print(last_az_preset_check_time);
    Serial.print(", preset ");
    Serial.print(last_pot_read);
    Serial.print(", rate ");
    Serial.print(pot_rate);
    Serial.println();
    #endif

    return;
  }

  millis_now = millis();
  az_preset_interval = millis_now - last_az_preset_check_time;
  
  // execute preset algorithm at intervals
  if ((az_preset_interval) < AZ_PRESET_CHECK_INTERVAL)
  {
    return;
  }

  // Time to process az preset, read pot and measure turning rate
  last_az_preset_check_time = millis_now;

  pot_read = analogRead(az_preset_pot);
  pot_rate = pot_read - last_pot_read; // pot adc value change per preset interval
  last_pot_read = pot_read;
  // set global apply calibration to pot reading, updated on each preset interval
  #ifdef FEATURE_AZ_PRESET_POT
  azimuth_preset = map(pot_read,
                        AZ_PRESET_POT_FULL_CCW,
                        AZ_PRESET_POT_FULL_CW,
                        AZ_PRESET_POT_FULL_CCW_MAP,
                        AZ_PRESET_POT_FULL_CW_MAP);

  #ifdef DEBUG_AZ_PRESET_POT
  Serial.print("check_az_preset: ");
  Serial.print("millis ");
  Serial.print(millis_now);
  Serial.print(", preset ");
  Serial.print(pot_read);
  Serial.print(", rate ");
  Serial.print(pot_rate);
  //Serial.print("\n");
  #endif
  
  #endif
  
  // stop_count hold info about turning vs not turning
  // stop_count increments when stopped, but limited by later code
  if (abs(pot_rate) > AZ_PRESET_RATE)
  {
    turning_stopped_count = 0;

    #ifdef DEBUG_AZ_PRESET_POT
    Serial.print(" stop count ");
    Serial.print(turning_stopped_count);
    Serial.print(" turning");
    Serial.print("\n");
    #endif
  }
  else // not currently turning
  {
    turning_stopped_count += 1; //pre-increment

    #ifdef DEBUG_AZ_PRESET_POT
    Serial.print(" stop count ");
    Serial.print(turning_stopped_count);
    Serial.print(" not turning");
    Serial.print("\n");
    #endif
  }

  // detect time after transition from turning to not turning
  // stop_count state machine
  //  set to 0 on turning
  //  increments when stopped
  //  triggers rotation when passing through AZ_PRESET_STOP_COUNT
  //  stop_count limited if greater than AZ_PRESET_STOP_COUNT
  // if-else sequence based on turning stop count
  if (turning_stopped_count == 0) // currently turning, display preset
  {
    is_display_preset = true;
  }
  else if (turning_stopped_count > 0 && 
           turning_stopped_count < AZ_PRESET_STOP_COUNT) // waiting for end of movement
  {
    is_display_preset = true; // continue display

    #ifdef DEBUG_AZ_PRESET_STATE
    Serial.print("check_az_preset: turning ");
    Serial.print("stop count ");
    Serial.print(turning_stopped_count);
    Serial.print("\n");
    #endif    

    #ifdef FEATURE_LCD_DISPLAY
    display_az_preset_LCD(new_pot_azimuth);
    #endif
  
  }
  else if (turning_stopped_count == AZ_PRESET_STOP_COUNT) // trigger rotation
  {
    is_display_preset = false;

    // initiate movement to preset
    submit_request(AZ, REQUEST_AZIMUTH_RAW, azimuth_preset * HEADING_MULTIPLIER);
    
    #ifdef DEBUG_AZ_PRESET_STATE
    Serial.print("check_az_preset: stopping ");
    Serial.print("stop count ");
    Serial.print(turning_stopped_count);
    Serial.print(", pot read ");
    Serial.print(pot_read);
    Serial.print(", azimuth ");
    Serial.print(azimuth_preset);
    Serial.print("\n");
    #endif
  }
  else if (turning_stopped_count > AZ_PRESET_STOP_COUNT) // waiting for movement
  {
    is_display_preset = false;
    turning_stopped_count = AZ_PRESET_STOP_COUNT; // gets pre-incremented before next test

    #ifdef DEBUG_AZ_PRESET_STATE
    Serial.print("check_az_preset: stopped ");
    Serial.print("stop count ");
    Serial.print(turning_stopped_count);
    Serial.print("\n");
    #endif
  }
#endif
}  // if az_preset_interval

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
  //byte number_columns = 0;
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
// obsolete, slated for removal
void check_buttons()
{
  #if defined(FEATURE_ADAFRUIT_BUTTONS)
  int buttons = 0;
  buttons = readButtons();
  if (buttons & BUTTON_RIGHT) 
  {
  #else // not adafruit
  if (button_cw_pin && (digitalRead(button_cw_pin) == LOW)) 
  {
  #endif //FEATURE_ADAFRUIT_BUTTONS or not
    if (!isAzButtonPressed) 
    {
      #ifdef DEBUG_BUTTONS
      if (debug_mode) {Serial.println(F("check_buttons: button_cw_pin pushed"));}       
      #endif //DEBUG_BUTTONS
      if (raw_azimuth < (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER)) 
      {
      submit_request(AZ,REQUEST_CW, 0);
      isAzButtonPressed = true;
      } else 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CW_LIMIT"));}
        #endif //DEBUG_BUTTONS
      }
    }
  } else // not button cw
  {
    #ifdef FEATURE_ADAFRUIT_BUTTONS
    if (buttons & BUTTON_LEFT) 
    {

    #else
    if (button_ccw_pin && (digitalRead(button_ccw_pin) == LOW)) 
    {
    #endif //FEATURE_ADAFRUIT_BUTTONS
      if (!isAzButtonPressed) 
      {
        #ifdef DEBUG_BUTTONS
        if (debug_mode) 
        {
          Serial.println(F("check_buttons: button_ccw_pin pushed"));
        }         
        #endif //DEBUG_BUTTONS 
        if (raw_azimuth > (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER)) 
        {
        submit_request(AZ,REQUEST_CCW,0);
        isAzButtonPressed = true;
        } else 
        {
          #ifdef DEBUG_BUTTONS
          if (debug_mode) {Serial.println(F("check_buttons: exceeded AZ_MANUAL_ROTATE_CCW_LIMIT"));}
          #endif //DEBUG_BUTTONS
        }
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
  if ((isAzButtonPressed) && (digitalRead(button_ccw_pin) == HIGH) && (digitalRead(button_cw_pin) == HIGH)) 
  {
    delay(200); // debouncing
    if ((digitalRead(button_ccw_pin) == HIGH) && (digitalRead(button_cw_pin) == HIGH)) 
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
} // check_buttons()
