#include <Arduino.h>
#include "EEPROM.h"

#include "rotator_features.h"
#include "rotator_pins.h"
#include "settings.h"
#include "macros.h"
#include "serial_command_processing.h"
#include "global_variables.h"
#include "eeprom_local.h"
#include "utilities_local.h"

//--------------------------------------------------------------
void print_help()
{
  // The H command

  #ifdef OPTION_SERIAL_HELP_TEXT
  #ifdef FEATURE_YAESU_EMULATION
  Serial.println(F("R Rotate Azimuth Clockwise"));
  Serial.println(F("L Rotate Azimuth Counter Clockwise"));
  Serial.println(F("A Stop"));
  Serial.println(F("C Report Azimuth in Degrees"));
  Serial.println(F("M### Rotate to ### degrees"));
  Serial.println(F("MTTT XXX XXX XXX ... Timed Interval Direction Setting  (TTT = Step value in seconds, XXX = Azimuth in degrees)"));
  Serial.println(F("T Start Timed Interval Tracking"));
  Serial.println(F("N Report Total Number of M Timed Interval Azimuths"));
  Serial.println(F("X1 Horizontal Rotation Low Speed"));
  Serial.println(F("X2 Horizontal Rotation Middle 1 Speed"));
  Serial.println(F("X3 Horizontal Rotation Middle 2 Speed"));
  Serial.println(F("X4 Horizontal Rotation High Speed"));
  Serial.println(F("S Stop"));
  Serial.println(F("O Offset Calibration"));
  Serial.println(F("F Full Scale Calibration"));
  #ifdef FEATURE_ELEVATION_CONTROL
  Serial.println(F("U Rotate Elevation Up"));
  Serial.println(F("D Rotate Elevation Down"));
  Serial.println(F("E Stop Elevation Rotation"));
  Serial.println(F("B Report Elevation in Degrees"));
  Serial.println(F("Wxxx yyy Rotate Azimuth to xxx Degrees and Elevation to yyy Degrees\r\r"));
  Serial.println(F("O2 Elevation Offset Calibration (0 degrees)"));
  Serial.println(F("F2 Elevation Full Scale Calibration (180 degrees (or maximum))"));
  #endif //FEATURE_ELEVATION_CONTROL
  #endif //FEATURE_YAESU_EMULATION  
  #endif //OPTION_SERIAL_HELP_TEXT
}

//--------------------------------------------------------------
// Write the speed_voltage to the PWM control pin
// depends on which pin is configured
void update_az_variable_outputs(byte speed_voltage)
{
  #ifdef DEBUG_VARIABLE_OUTPUTS
  if (debug_mode) 
  {
    Serial.print(F("update_az_variable_outputs: speed_voltage: "));
    Serial.print(speed_voltage);
  }
  #endif //DEBUG_VARIABLE_OUTPUTS

  if (((az_state == SLOW_START_CW)       || 
       (az_state == NORMAL_CW)           || 
       (az_state == SLOW_DOWN_CW)        || 
       (az_state == TIMED_SLOW_DOWN_CW)) && 
       (rotate_cw_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_cw_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_cw_pwm, speed_voltage);
  }
  
  if (((az_state == SLOW_START_CCW)       ||
	   (az_state == NORMAL_CCW)           ||
	   (az_state == SLOW_DOWN_CCW)        ||
	   (az_state == TIMED_SLOW_DOWN_CCW)) &&
	   (rotate_ccw_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_ccw_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_ccw_pwm, speed_voltage);
  }
  
  if (((az_state == SLOW_START_CW)        || 
       (az_state == NORMAL_CW)            || 
       (az_state == SLOW_DOWN_CW)         || 
       (az_state == TIMED_SLOW_DOWN_CW)   || 
       (az_state == SLOW_START_CCW)       || 
       (az_state == NORMAL_CCW)           || 
       (az_state == SLOW_DOWN_CCW)        || 
       (az_state == TIMED_SLOW_DOWN_CCW)) && 
       (rotate_cw_ccw_pwm))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_cw_ccw_pwm"));}
    #endif //DEBUG_VARIABLE_OUTPUTS
    analogWrite(rotate_cw_ccw_pwm, speed_voltage);
  }  
  
  if (((az_state == SLOW_START_CW)       || 
       (az_state == NORMAL_CW)           || 
       (az_state == SLOW_DOWN_CW)        || 
       (az_state == TIMED_SLOW_DOWN_CW)) && 
       (rotate_cw_freq))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_cw_freq"));}  
    #endif //DEBUG_VARIABLE_OUTPUTS
    tone(rotate_cw_freq,map(speed_voltage,0,255,AZ_VARIABLE_FREQ_OUTPUT_LOW,AZ_VARIABLE_FREQ_OUTPUT_HIGH));
  }
  
  if (((az_state == SLOW_START_CCW)       || 
       (az_state == NORMAL_CCW)           || 
       (az_state == SLOW_DOWN_CCW)        || 
       (az_state == TIMED_SLOW_DOWN_CCW)) && 
       (rotate_ccw_freq))
  {
    #ifdef DEBUG_VARIABLE_OUTPUTS
    if (debug_mode) {Serial.print(F("\trotate_ccw_freq"));}  
    #endif //DEBUG_VARIABLE_OUTPUTS
    tone(rotate_ccw_freq,map(speed_voltage,0,255,AZ_VARIABLE_FREQ_OUTPUT_LOW,AZ_VARIABLE_FREQ_OUTPUT_HIGH));
  }  
  
  if (azimuth_speed_voltage) 
  {
    analogWrite(azimuth_speed_voltage, speed_voltage);
  }
  
  #ifdef DEBUG_VARIABLE_OUTPUTS
  if (debug_mode) {Serial.println();}
  #endif //DEBUG_VARIABLE_OUTPUTS
  
  current_az_speed_voltage = speed_voltage;
}

//--------------------------------------------------------------
void read_azimuth()
{
  unsigned int previous_raw_azimuth = raw_azimuth;
  static unsigned long last_measurement_time = 0;

  #ifdef DEBUG_HEADING_READING_TIME
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;
  static float average_read_time = 0;
  #endif //DEBUG_HEADING_READING_TIME

  #ifndef FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
  if ((millis() - last_measurement_time) > AZIMUTH_MEASUREMENT_FREQUENCY_MS){
  #else
  if (1)
  {
  #endif
    
    #ifdef FEATURE_AZ_POSITION_POTENTIOMETER
    // read analog input and convert it to degrees; this gets funky because of 450 degree rotation
    // Bearings:  180-------359-0--------270
    // Voltage:    0----------------------5
    // ADC:        0--------------------1023

    analog_az = analogRead(rotator_analog_az);
    raw_azimuth = (map(  analog_az,
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
    
    #ifdef FEATURE_AZ_POSITION_HMC5883L
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

    #ifdef FEATURE_AZ_POSITION_LSM303
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
    
    last_measurement_time = millis();
  }
}

//--------------------------------------------------------------
void check_for_dirty_configuration()
{  
  static unsigned long last_config_write_time = 0;
  
  if ((configuration_dirty) && ((millis() - last_config_write_time) > (EEPROM_WRITE_DIRTY_CONFIG_TIME*1000))){
    write_settings_to_eeprom();
    last_config_write_time = millis(); 
  } 
}

//--------------------------------------------------------------
void check_brake_release() 
{
  static byte in_az_brake_release_delay = 0;
  static unsigned long az_brake_delay_start_time = 0;
  #ifdef FEATURE_ELEVATION_CONTROL
  static byte in_el_brake_release_delay = 0;
  static unsigned long el_brake_delay_start_time = 0;
  #endif //FEATURE_ELEVATION_CONTROL
 
  if ((az_state == IDLE) && (brake_az_engaged)) 
  {
    if (in_az_brake_release_delay) 
    {
      if ((millis() - az_brake_delay_start_time) > AZ_BRAKE_DELAY) 
      {
        brake_release(AZ, false);
        in_az_brake_release_delay = 0; 
      }    
    } else 
    {
      az_brake_delay_start_time = millis();
      in_az_brake_release_delay = 1;
    }
  } 
 
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((el_state == IDLE) && (brake_el_engaged)) 
  {
    if (in_el_brake_release_delay) 
    {
      if ((millis() - el_brake_delay_start_time) > EL_BRAKE_DELAY) 
      {
        brake_release(EL, false);
        in_el_brake_release_delay = 0; 
      }    
    } else 
    {
      el_brake_delay_start_time = millis();
      in_el_brake_release_delay = 1;
    }  
  } 
  #endif //FEATURE_ELEVATION_CONTROL  
}

//--------------------------------------------------------------
// Brake release on means to apply power to release the brake
// default condition is braked, meaning brake release off
// BRAKE_RELEASE_{on,off} defined according to controller polarity
void brake_release(byte az_or_el, boolean brake_release)
{ 
  if (az_or_el == AZ) 
  {
    if (brake_az) 
    {
      if (brake_release) 
      {
        digitalWrite(brake_az, BRAKE_RELEASE_ON);
        brake_az_engaged = 1;
        #ifdef DEBUG_BRAKE
        if (debug_mode) 
        {
          Serial.println(F("brake_release: brake_az BRAKE_RELEASE_ON"));
        }
        #endif //DEBUG_BRAKE
      } else 
      {
        digitalWrite(brake_az,BRAKE_RELEASE_OFF);
        brake_az_engaged = 0;      
        #ifdef DEBUG_BRAKE
        if (debug_mode) 
        {
          Serial.println(F("brake_release: brake_az BRAKE_RELEASE_OFF"));
        }  
        #endif //DEBUG_BRAKE  
     } 
   }
  } else 
  {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (brake_el) 
    {
      digitalWrite(brake_el,HIGH);
      brake_el_engaged = 1;
      #ifdef DEBUG_BRAKE
      if (debug_mode) 
      {
        Serial.println(F("brake_release: brake_el BRAKE_RELEASE_ON"));
      }   
      #endif //DEBUG_BRAKE    
    } else 
    {
      digitalWrite(brake_el,LOW);
      brake_el_engaged = 0;   
      #ifdef DEBUG_BRAKE
      if (debug_mode) 
      {
        Serial.println(F("brake_release: brake_el BRAKE_RELEASE_OFF"));
      }       
      #endif //DEBUG_BRAKE
    }
    #endif //FEATURE_ELEVATION_CONTROL
  }   
}
