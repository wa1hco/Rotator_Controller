// Arduino environment
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <MsTimer2.h>

// C++ functions
#include <math.h> 

// Project configuration
#include "dependencies.h"

// Project functions

#include "Service_Blink_LED.h"

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
    Serial.print(F(" (sensor: "));
    Serial.print(analog_az);
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
    
    #ifdef DEBUG_AZ_SPEED_POT
    if (normal_az_speed_voltage) 
    {
      Serial.print(F("\tAZ Speed Norm: "));
      Serial.print(normal_az_speed_voltage, DEC);
    }
    
    Serial.print(F(" Current: "));
    Serial.print(current_az_speed_voltage,DEC);
    
    if (az_speed_pot) 
    {
      Serial.print(F("\tAZ Speed Pot: "));
      Serial.print(analogRead(az_speed_pot));
    }    
    #endif

    #ifdef DEBUG_AZ_PRESET_POT
    if (az_preset_pot) 
    {
      Serial.print(F("\tAZ Preset Pot Analog: "));
      Serial.print(analogRead(az_preset_pot));
      Serial.print(F("\tAZ Preset Pot Setting: "));
      Serial.print(map(analogRead(az_preset_pot), AZ_PRESET_POT_FULL_CW, AZ_PRESET_POT_FULL_CCW, AZ_PRESET_POT_FULL_CW_MAP, AZ_PRESET_POT_FULL_CCW_MAP));
    } 
    #endif
    
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
