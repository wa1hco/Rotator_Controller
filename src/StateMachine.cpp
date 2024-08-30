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

// Project configuration
#include "dependencies.h"

// Project functions
void service_request_queue();
void submit_request(byte axis, byte request, int parm);

//--------------------------------------------------------------
void submit_request(byte axis, byte request, int parm)
{ 
  #ifdef DEBUG_SUBMIT_REQUEST 
  if (debug_mode) {Serial.print(F("submit_request: "));}
  #endif //DEBUG_SUBMIT_REQUEST
  
  if (axis == AZ) 
  {
    az_request = request;
    az_request_parm = parm;
    az_request_queue_state = IN_QUEUE;

    #ifdef DEBUG_SUBMIT_REQUEST
    if (debug_mode) {Serial.print(F("AZ "));Serial.print(request);Serial.print(F(" "));Serial.println(parm);}
    #endif //DEBUG_SUBMIT_REQUEST

  } 
}


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
//  pin numbers, rotate_cw, rotate_ccw, rotate_cw_pwn, etc 
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
                  map(normal_az_speed_voltage, 0, 255,
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
    if (abs((target_raw_azimuth - azimuth)/HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_AZ*((float)az_slow_down_step/(float)AZ_SLOW_DOWN_STEPS))))
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
        (abs((target_raw_azimuth - azimuth)/HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_AZ)
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
    if (
          (az_state == NORMAL_CW) || 
          (az_state == SLOW_START_CW) || 
          (az_state == SLOW_DOWN_CW)
        )
    {
      if  (
            (abs(azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
            (
              (azimuth > target_raw_azimuth) && 
              ( (azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
            )
          ) 
      {
        delay(50);
        read_azimuth();
        if  (
              (abs(azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
              ((azimuth > target_raw_azimuth) && 
              ( (azimuth - target_raw_azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
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
            (abs(azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER) ) || 
            ( (azimuth < target_raw_azimuth) &&
              ( (target_raw_azimuth - azimuth) < ( (AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER) )
		        )
          ) 
      {
        delay(50);
        read_azimuth();
        if  (
              (abs(azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER)) || 
              (
                (azimuth < target_raw_azimuth) && 
                ((target_raw_azimuth - azimuth) < ((AZIMUTH_TOLERANCE+5)*HEADING_MULTIPLIER))
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
        if ((az_request_parm >= 0) && (az_request_parm <= (360 * HEADING_MULTIPLIER))) 
        {
          target_azimuth = az_request_parm;
          target_raw_azimuth = az_request_parm;
          if (target_azimuth == (360*HEADING_MULTIPLIER)) {target_azimuth = 0;}      
          if ((target_azimuth > (azimuth - (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))) &&
        	    (target_azimuth < (azimuth + (AZIMUTH_TOLERANCE * HEADING_MULTIPLIER))))
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
              Serial.println(work_target_raw_azimuth/HEADING_MULTIPLIER);
              Serial.print(F(" configuration.azimuth_starting_point:"));
              Serial.println(configuration.azimuth_starting_point);
              Serial.print(" ");
            }
            #endif //DEBUG_SERVICE_REQUEST_QUEUE            
            
            if (work_target_raw_azimuth < (configuration.azimuth_starting_point * HEADING_MULTIPLIER)) 
            {
              work_target_raw_azimuth = work_target_raw_azimuth + (360 * HEADING_MULTIPLIER);
              target_raw_azimuth = work_target_raw_azimuth;
              #ifdef DEBUG_SERVICE_REQUEST_QUEUE
              if (debug_mode) {Serial.print(F("->B"));}
              #endif //DEBUG_SERVICE_REQUEST_QUEUE
            }
            if ((work_target_raw_azimuth + (360*HEADING_MULTIPLIER)) < 
                ((configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)*HEADING_MULTIPLIER)) 
            { // is there a second possible heading in overlap?
              if (abs(azimuth - work_target_raw_azimuth) < abs((work_target_raw_azimuth+(360*HEADING_MULTIPLIER)) - azimuth)) 
              { // is second possible heading closer?
                #ifdef DEBUG_SERVICE_REQUEST_QUEUE
                if (debug_mode) {Serial.print(F("->C"));}
                #endif //DEBUG_SERVICE_REQUEST_QUEUE
                if (work_target_raw_azimuth  > azimuth) 
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
                if ((work_target_raw_azimuth + (360*HEADING_MULTIPLIER)) > azimuth) 
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
              if (work_target_raw_azimuth  > azimuth) 
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
            if (az_request_parm > azimuth) 
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
        
        if (((abs(azimuth - target_raw_azimuth) < (AZIMUTH_TOLERANCE*HEADING_MULTIPLIER))) && (az_state == IDLE)) 
        {
          #ifdef DEBUG_SERVICE_REQUEST_QUEUE
          if (debug_mode) {Serial.print(F(" request within tolerance"));}          
          #endif //DEBUG_SERVICE_REQUEST_QUEUE
          az_request_queue_state = NONE;
        } else 
        {
          if (target_raw_azimuth > azimuth) 
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
          if (target_raw_azimuth < azimuth) 
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
