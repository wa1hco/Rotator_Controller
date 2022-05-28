#include <Arduino.h>
#include "EEPROM.h"

#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"

#include "global_variables.h"
#include "serial_command_processing.h"
#include "eeprom_local.h"

//--------------------------------------------------------------
void initialize_eeprom_with_defaults()
{
  #ifdef DEBUG_EEPROM
  if (debug_mode) 
  {
    Serial.println(F("initialize_eeprom_with_defaults: writing eeprom"));
  }
  #endif //DEBUG_EEPROM

  configuration.analog_az_full_ccw          = ANALOG_AZ_FULL_CCW;
  configuration.analog_az_full_cw           = ANALOG_AZ_FULL_CW;
  configuration.analog_el_0_degrees         = ANALOG_EL_0_DEGREES;
  configuration.analog_el_max_elevation     = ANALOG_EL_MAX_ELEVATION;   
  configuration.azimuth_starting_point      = AZIMUTH_STARTING_POINT_DEFAULT;
  configuration.azimuth_rotation_capability = AZIMUTH_ROTATION_CAPABILITY_DEFAULT;
  configuration.last_azimuth                = azimuth;
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
    Serial.print(F("write_settings_to_eeprom: writing settings to eeprom\n"));
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
