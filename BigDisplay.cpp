//--------------------------------------------------------------
void update_big_display()
{
  // update the LCD display
  static byte lcd_state_row_0 = 0;
  static byte lcd_state_row_1 = 0;
  
  String direction_string;

  static int last_azimuth = -1;
  
  char workstring[7];
  
  unsigned int target = 0;
  
  // row 0 ------------------------------------------------------------
  // direction_string
  // work_string
  // azimuth_direction(azimuth)

  if (((millis() - last_lcd_update) > LCD_UPDATE_TIME) || (push_lcd_update))
  {
    if ((lcd_state_row_0 == 0) && (lcd_state_row_1 == 0))
    {
      lcd.clear();
      lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0); // cursor to start of centered string
      lcd.print(direction_string); 
      lcd_state_row_0 = LCD_DIRECTION;
    }

    #ifndef FEATURE_ELEVATION_CONTROL
    #ifdef FEATURE_AZ_PRESET_ENCODER
    target = az_encoder_raw_degrees;
    // wrap, twice if necessary
    if (target > (359*LCD_HEADING_MULTIPLIER)) {target = target - (360*LCD_HEADING_MULTIPLIER);}
    if (target > (359*LCD_HEADING_MULTIPLIER)) {target = target - (360*LCD_HEADING_MULTIPLIER);}

    if (preset_encoders_state == ENCODER_AZ_PENDING) 
    {
      clear_display_row(0);                                                 // Show Target Deg on upper line
      direction_string = "Target ";
      dtostrf(target/LCD_HEADING_MULTIPLIER,1,LCD_DECIMAL_PLACES,workstring);
      direction_string.concat(workstring);
      direction_string.concat(char(223));
      lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
      lcd.print(direction_string); 
      
      lcd_state_row_0 = LCD_TARGET_AZ;
      #ifdef DEBUG_DISPLAY
      if (debug_mode) 
      {
        Serial.print(F("update_display: "));
        Serial.println(direction_string);
      }        
      #endif //DEBUG_DISPLAY
      
    } else 
    {   
    #endif //FEATURE_AZ_PRESET_ENCODER  
      if (az_state != IDLE) 
      {
        if (az_request_queue_state == IN_PROGRESS_TO_TARGET) 
        {
          clear_display_row(0);
          direction_string = "Rotating to ";
          dtostrf(target_azimuth/LCD_HEADING_MULTIPLIER,1,LCD_DECIMAL_PLACES,workstring);
          direction_string.concat(workstring);          
          //direction_string.concat(int(target_azimuth / LCD_HEADING_MULTIPLIER));
          direction_string.concat(char(223));
          lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
          lcd.print(direction_string);          
          lcd_state_row_0 = LCD_ROTATING_TO;
          
          #ifdef DEBUG_DISPLAY
          if (debug_mode) {
            Serial.print(F("update_display: "));
            Serial.println(direction_string);
          }  
          #endif //DEBUG_DISPLAY        
        } else 
        {
          if ((az_state == SLOW_START_CW) || (az_state == NORMAL_CW) || (az_state == SLOW_DOWN_CW) || (az_state == TIMED_SLOW_DOWN_CW)) 
          {
            if (lcd_state_row_0 != LCD_ROTATING_CW) 
            {
              clear_display_row(0);
              direction_string = "Rotating CW";
              lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
              lcd.print(direction_string);                             
              lcd_state_row_0 = LCD_ROTATING_CW;
              #ifdef DEBUG_DISPLAY
              if (debug_mode) 
              {
                Serial.print(F("update_display: "));
                Serial.println(direction_string);
              }     
              #endif //DEBUG_DISPLAY           
            }
          } else 
          {
            if (lcd_state_row_0 != LCD_ROTATING_CCW) 
            {
              clear_display_row(0);
              direction_string = "Rotating CCW";
              lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
              lcd.print(direction_string);                              
              lcd_state_row_0 = LCD_ROTATING_CCW;
              #ifdef DEBUG_DISPLAY
              if (debug_mode) 
              {
                Serial.print(F("update_display: "));
                Serial.println(direction_string);
              }     
              #endif //DEBUG_DISPLAY            
            }
          }
        }
      } else 
      { // az_state == IDLE
        if ((last_azimuth != azimuth) || (lcd_state_row_0 != LCD_DIRECTION))
        {
          direction_string = azimuth_direction(azimuth);
          if ((last_direction_string == direction_string) || (lcd_state_row_0 != LCD_DIRECTION)) 
          {
            clear_display_row(0);
            lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),0);
            lcd.print(direction_string);          
            lcd_state_row_0 = LCD_DIRECTION;
            #ifdef DEBUG_DISPLAY
            if (debug_mode) 
            {
              Serial.print(F("update_display: "));
              Serial.println(direction_string); 
            }       
            #endif //DEBUG_DISPLAY        
          } else 
          {
            lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2)-1,0);
            lcd.print(" ");
            lcd.print(direction_string);   
            lcd.print(" "); 
            #ifdef DEBUG_DISPLAY
            if (debug_mode) {
              Serial.print(F("update_display: "));
              Serial.println(direction_string); 
            }               
            #endif //DEBUG_DISPLAY
          }
        }
      } //(az_state != IDLE)
    #ifdef FEATURE_AZ_PRESET_ENCODER
    } //(preset_encoders_state == ENCODER_AZ_PENDING)
    #endif //FEATURE_AZ_PRESET_ENCODER
    #endif
   
    push_lcd_update = 0;
  }

  //     row 1 --------------------------------------------

  if ((millis()-last_lcd_update) > LCD_UPDATE_TIME) 
  {
    if (last_azimuth != azimuth) 
    {
      clear_display_row(1);
      direction_string = "Azimuth ";
      dtostrf(azimuth/LCD_HEADING_MULTIPLIER,1,LCD_DECIMAL_PLACES,workstring);
      direction_string.concat(workstring); 
      direction_string.concat(char(223));                  
      lcd.setCursor(((LCD_COLUMNS - direction_string.length())/2),1);
      lcd.print(direction_string);  
       
      printbigazimuth(azimuth);
       
      last_azimuth = azimuth;
      lcd_state_row_1 = LCD_HEADING;  
    }  
  }
  if ((millis() - last_lcd_update) > LCD_UPDATE_TIME) {last_lcd_update = millis();}
  last_direction_string = direction_string;
} // update_big_dispaly()
