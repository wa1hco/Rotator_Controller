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
#include <EEPROM.h>
#include <avr/io.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "dependencies.h"

//#define CODE_VERSION "2013091101"
//#define CODE_VERSION "2017021101"
#define CODE_VERSION "2024070901"

// Project functions

//--------------------------------------------------------------
#ifdef FEATURE_ELEVATION_CONTROL
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

/*
 * elevation.c
 *
 *  Created on: Feb 26, 2017
 *      Author: jeff
 */
#ifdef FEATURE_ELEVATION_CONTROL
void service_rotation_elevation()
{
	static byte el_direction_change_flag = 0;
	static byte el_initial_slow_down_voltage = 0;

	if (el_state == INITIALIZE_NORMAL_UP)
	{
	  update_el_variable_outputs(normal_el_speed_voltage);
	  rotator(ACTIVATE,UP);
	  el_state = NORMAL_UP;
	}
	if (el_state == INITIALIZE_NORMAL_DOWN)
	{
	  update_el_variable_outputs(normal_el_speed_voltage);
	  rotator(ACTIVATE,DOWN);
	  el_state = NORMAL_DOWN;
	}
	if (el_state == INITIALIZE_SLOW_START_UP)
	{
	  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
	  rotator(ACTIVATE,UP);
	  el_slowstart_start_time = millis();
	  el_last_step_time = 0;
	  el_slow_start_step = 0;
	  el_state = SLOW_START_UP;
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_UP -> SLOW_START_UP"));}
	  #endif //DEBUG_SERVICE_ROTATION
	}

	if (el_state == INITIALIZE_SLOW_START_DOWN)
	{
	  update_el_variable_outputs(EL_SLOW_START_STARTING_PWM);
	  rotator(ACTIVATE,DOWN);
	  el_slowstart_start_time = millis();
	  el_last_step_time = 0;
	  el_slow_start_step = 0;
	  el_state = SLOW_START_DOWN;
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.println(F("service_rotation: INITIALIZE_SLOW_START_DOWN -> SLOW_START_DOWN"));}
	  #endif //DEBUG_SERVICE_ROTATION
	}

	if (el_state == INITIALIZE_TIMED_SLOW_DOWN_UP)
	{
	  el_direction_change_flag = 0;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_UP;
	}

	if (el_state == INITIALIZE_TIMED_SLOW_DOWN_DOWN)
	{
	  el_direction_change_flag = 0;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_DOWN;
	}

	if (el_state == INITIALIZE_DIR_CHANGE_TO_UP)
	{
	  el_direction_change_flag = 1;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_DOWN;
	}

	if (el_state == INITIALIZE_DIR_CHANGE_TO_DOWN)
	{
	  el_direction_change_flag = 1;
	  el_timed_slow_down_start_time = millis();
	  el_last_step_time = millis();
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  el_state = TIMED_SLOW_DOWN_UP;
	}

	// slow start-------------------------------------------------------------------------------------------------
	if ((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))
	{
	  if ((millis() - el_slowstart_start_time) >= EL_SLOW_START_UP_TIME)
	  {  // is it time to end slow start?
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.print(F("service_rotation: NORMAL_"));}
		#endif //DEBUG_SERVICE_ROTATION
		if (el_state == SLOW_START_UP)
		{
		  el_state = NORMAL_UP;
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {Serial.println(F("UP"));}
		  #endif //DEBUG_SERVICE_ROTATION
		} else
		{
		  el_state = NORMAL_DOWN;
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {Serial.println(F("DOWN"));}
		  #endif //DEBUG_SERVICE_ROTATION
		}
		update_el_variable_outputs(normal_el_speed_voltage);
	  } else
	  {  // it's not time to end slow start yet, but let's check if it's time to step up the speed voltage
		if (((millis() - el_last_step_time) > (EL_SLOW_START_UP_TIME/EL_SLOW_START_STEPS)) && (normal_el_speed_voltage > EL_SLOW_START_STARTING_PWM))
		{
		  #ifdef DEBUG_SERVICE_ROTATION
		  if (debug_mode) {
			Serial.print(F("service_rotation: step up: "));
			Serial.print(el_slow_start_step);
			Serial.print(F(" pwm: "));
			Serial.println((int)(EL_SLOW_START_STARTING_PWM+((normal_el_speed_voltage-EL_SLOW_START_STARTING_PWM)*((float)el_slow_start_step/(float)(EL_SLOW_START_STEPS-1)))));
		  }
		  #endif //DEBUG_SERVICE_ROTATION
		  update_el_variable_outputs((EL_SLOW_START_STARTING_PWM+((normal_el_speed_voltage-EL_SLOW_START_STARTING_PWM)*((float)el_slow_start_step/(float)(EL_SLOW_START_STEPS-1)))));
		  el_last_step_time = millis();
		  el_slow_start_step++;
		}
	  }
	} //((el_state == SLOW_START_UP) || (el_state == SLOW_START_DOWN))


	// timed slow down ------------------------------------------------------------------------------------------------------
	if (((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN)) && ((millis() - el_last_step_time) >= (TIMED_SLOW_DOWN_TIME/EL_SLOW_DOWN_STEPS)))
	{
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode)
	  {
		Serial.print(F("service_rotation: TIMED_SLOW_DOWN step down: "));
		Serial.print(el_slow_down_step);
		Serial.print(F(" pwm: "));
		Serial.println((int)(normal_el_speed_voltage*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS)));
	  }
	  #endif //DEBUG_SERVICE_ROTATION
	  update_el_variable_outputs((int)(normal_el_speed_voltage*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS)));
	  el_last_step_time = millis();
	  el_slow_down_step--;

	  if (el_slow_down_step == 0)
	  { // is it time to exit timed slow down?
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("service_rotation: TIMED_SLOW_DOWN->IDLE"));}
		#endif //DEBUG_SERVICE_ROTATION
		rotator(DEACTIVATE,UP);
		rotator(DEACTIVATE,DOWN);
		if (el_direction_change_flag)
		{
		  if (el_state == TIMED_SLOW_DOWN_UP)
		  {
			rotator(ACTIVATE,DOWN);
			if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_DOWN;} else {el_state = NORMAL_DOWN;};
			el_direction_change_flag = 0;
		  }
		  if (el_state == TIMED_SLOW_DOWN_DOWN)
		  {
			rotator(ACTIVATE,UP);
			if (el_slowstart_active) {el_state = INITIALIZE_SLOW_START_UP;} else {el_state = NORMAL_UP;};
			el_direction_change_flag = 0;
		  }
		} else
		{
		  el_state = IDLE;
		  el_request_queue_state = NONE;
		}
	  }
	}  //((el_state == TIMED_SLOW_DOWN_UP) || (el_state == TIMED_SLOW_DOWN_DOWN))

	// slow down ---------------------------------------------------------------------------------------------------------------
	if ((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))
	{
	  // is it time to do another step down?
	  if (abs((target_elevation - elevation)/HEADING_MULTIPLIER) <= (((float)SLOW_DOWN_BEFORE_TARGET_EL*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))))
	  {
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode)
		{
		  Serial.print(F("service_rotation: step down: "));
		  Serial.print(el_slow_down_step);
		  Serial.print(F(" pwm: "));
		  Serial.println((int)(EL_SLOW_DOWN_PWM_STOP+((el_initial_slow_down_voltage-EL_SLOW_DOWN_PWM_STOP)*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))));
		}
		#endif //DEBUG_SERVICE_ROTATION
		update_el_variable_outputs((EL_SLOW_DOWN_PWM_STOP+((el_initial_slow_down_voltage-EL_SLOW_DOWN_PWM_STOP)*((float)el_slow_down_step/(float)EL_SLOW_DOWN_STEPS))));
		el_slow_down_step--;
	  }
	}  //((el_state == SLOW_DOWN_UP) || (el_state == SLOW_DOWN_DOWN))

	// normal -------------------------------------------------------------------------------------------------------------------
	// if slow down is enabled, see if we're ready to go into slowdown
	if (((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == NORMAL_DOWN) || (el_state == SLOW_START_DOWN)) &&
	(el_request_queue_state == IN_PROGRESS_TO_TARGET) && el_slowdown_active && (abs((target_elevation - elevation)/HEADING_MULTIPLIER) <= SLOW_DOWN_BEFORE_TARGET_EL))
	{
	  #ifdef DEBUG_SERVICE_ROTATION
	  if (debug_mode) {Serial.print(F("service_rotation: SLOW_DOWN_"));}
	  #endif //DEBUG_SERVICE_ROTATION
	  el_slow_down_step = EL_SLOW_DOWN_STEPS-1;
	  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP))
	  {
		el_state = SLOW_DOWN_UP;
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("UP"));}
		#endif //DEBUG_SERVICE_ROTATION
	  } else
	  {
		el_state = SLOW_DOWN_DOWN;
		#ifdef DEBUG_SERVICE_ROTATION
		if (debug_mode) {Serial.println(F("DOWN"));}
		#endif //DEBUG_SERVICE_ROTATION
	  }
	  if (EL_SLOW_DOWN_PWM_START < current_el_speed_voltage)
	  {
		update_el_variable_outputs(EL_SLOW_DOWN_PWM_START);
		el_initial_slow_down_voltage = EL_SLOW_DOWN_PWM_START;
	  } else
	  {
		el_initial_slow_down_voltage = current_el_speed_voltage;
	  }
	}

	// check rotation target --------------------------------------------------------------------------------------------------------
	if ((el_state != IDLE) && (el_request_queue_state == IN_PROGRESS_TO_TARGET) )
	{
	  if ((el_state == NORMAL_UP) || (el_state == SLOW_START_UP) || (el_state == SLOW_DOWN_UP))
	  {
		if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		{
		  delay(50);
		  read_elevation();
		  if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) || ((elevation > target_elevation) && ((elevation - target_elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		  {
			rotator(DEACTIVATE,UP);
			rotator(DEACTIVATE,DOWN);
			el_state = IDLE;
			el_request_queue_state = NONE;
			#ifdef DEBUG_SERVICE_ROTATION
			if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
			#endif //DEBUG_SERVICE_ROTATION
		  }
		}
	  } else
	  {
		if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) ||
			((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		{
		  delay(50);
		  read_elevation();
		  if ((abs(elevation - target_elevation) < (ELEVATION_TOLERANCE*HEADING_MULTIPLIER)) ||
			  ((elevation < target_elevation) && ((target_elevation - elevation) < ((ELEVATION_TOLERANCE+5)*HEADING_MULTIPLIER))))
		  {
			rotator(DEACTIVATE,UP);
			rotator(DEACTIVATE,DOWN);
			el_state = IDLE;
			el_request_queue_state = NONE;
			#ifdef DEBUG_SERVICE_ROTATION
			if (debug_mode) {Serial.println(F("service_rotation: IDLE"));}
			#endif //DEBUG_SERVICE_ROTATION
		  }
		}
	  }
	}
} // service_rotator_elevation()
#endif //FEATURE_ELEVATION_CONTROL

