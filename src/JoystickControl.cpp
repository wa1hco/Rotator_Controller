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
  static byte joystick_elevation_rotati = NOT_DOING_ANYTHING;
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