
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
#include "dependencies.h"
#include "EEPROM.h"

//--------------------------------------------------------------
void initialize_eeprom_with_defaults()
{
  #ifdef DEBUG_EEPROM
  if (debug_mode) 
  {
    Serial.println(F("initialize_eeprom_with_defaults: writing eeprom"));
  }
  #endif //DEBUG_EEPROM

  configuration.Raz_full_ccw                = RAZ_FULL_CCW; // sensor reading
  configuration.Raz_full_cw                 = RAZ_FULL_CW;  // sensor reading
  configuration.analog_el_0_degrees         = ANALOG_EL_0_DEGREES;
  configuration.analog_el_max_elevation     = ANALOG_EL_MAX_ELEVATION;   
  configuration.azimuth_starting_point      = AZIMUTH_STARTING_POINT_DEFAULT;
  configuration.azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;
  configuration.last_azimuth                = azimuth;            // current azimuth
  #ifdef FEATURE_ELEVATION_CONTROL
  configuration.last_elevation              = elevation; 
  #else    
  configuration.last_elevation              = 0;
  #endif
  write_settings_to_eeprom();
}

//--------------------------------------------------------------
void write_settings_to_eeprom()
{
  #ifdef DEBUG_EEPROM
  if (debug_mode) 
  {
    Serial.println(F("write_settings_to_eeprom: writing settings to eeprom\n"));
  }
  #endif //DEBUG_EEPROM
  
  configuration.magic_number = EEPROM_MAGIC_NUMBER;
  
  const byte* p = (const byte*)(const void*)&configuration;
  unsigned int i;
  int ee = 0;
  for (i = 0; i < sizeof(configuration); i++)
  {
    EEPROM.write(ee++, *p++);  
  }
  //EEPROM_writeAnything(0,configuration);
  configuration_dirty = 0;
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
      Serial.println(F("read_settings_from_eeprom: "));
      Serial.print("Raz_full_ccw           ");
      Serial.println(configuration.Raz_full_ccw,          DEC);
      Serial.print("Raz_full_cw            ");
      Serial.println(configuration.Raz_full_cw,           DEC);
      Serial.print("azimuth_starting_point       ");
      Serial.println(configuration.azimuth_starting_point,      DEC);
      Serial.print("azimuth_rotation_capability  ");
      Serial.println(configuration.azimuth_rotation_capability, DEC);
      Serial.print("last_azimuth:                ");
      Serial.println(configuration.last_azimuth,1);
      Serial.print("analog_el_0_degrees          ");
      Serial.println(configuration.analog_el_0_degrees,         DEC);
      Serial.print("analog_el_max_elevation      ");
      Serial.println(configuration.analog_el_max_elevation,     DEC);
      Serial.print("last_elevation               ");
      Serial.println(configuration.last_elevation,1);          
    }
    #endif //DEBUG_EEPROM
    
    #ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
    raw_azimuth = int(configuration.last_azimuth*HEADING_MULTIPLIER);
    // unwrap raw_azimuth
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