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

void display_calibration_settings();

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void yaesu_serial_command()
{
  if (incoming_serial_byte == 10) { return; }  // ignore carriage returns
  if ((incoming_serial_byte != 13) && (serial0_buffer_index < COMMAND_BUFFER_SIZE)) 
  {               // if it's not a carriage return, add it to the buffer
    serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
    serial0_buffer_index++;
  } else 
  {                       // we got a carriage return, time to get to work on the command
    if ((serial0_buffer[0] > 96) && (serial0_buffer[0] < 123)) 
    {
      serial0_buffer[0] = serial0_buffer[0] - 32;
    }
    switch (serial0_buffer[0]) // look at the first character of the command
    {          
      case 'A':  // A - CW/CCW rotation stop
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: A cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_STOP,0);
        Serial.println();
        break;         
      case 'C': // C - return current azimuth                                  
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: C cmd"));}
        #endif //DEBUG_SERIAL
        #ifdef OPTION_DELAY_C_CMD_OUTPUT
        delay(400);
        #endif
        report_current_azimuth();
        break;
      case 'F': // F - full scale calibration
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: F cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_f_command();
        break;                      

      case 'H': print_help(); break;                     // H - print help (simulated Yaesu GS-232A help - not all commands are supported
      case 'L':  // L - manual left (CCW) rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: L cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_CCW,0);
        Serial.println();
        break;         
      case 'O':  // O - offset calibration
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: O cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_o_command();
        break;                      
      case 'R':  // R - manual right (CW) rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: R cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ,REQUEST_CW,0);
        Serial.println();
        break;        
      case 'S':         // S - all stop
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: S cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(AZ, REQUEST_STOP, 0);
        #ifdef FEATURE_ELEVATION_CONTROL
        submit_request(EL,REQUEST_STOP,0);
        #endif
        #ifdef FEATURE_TIMED_BUFFER
        clear_timed_buffer();
        #endif //FEATURE_TIMED_BUFFER
        Serial.println();
        break;
      case 'M': // M - auto azimuth rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: M cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_m_command();
        break;     
      #ifdef FEATURE_TIMED_BUFFER 
      case 'N': // N - number of loaded timed interval entries
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: N cmd"));}
        #endif //DEBUG_SERIAL
        Serial.println(timed_buffer_number_entries_loaded);
        break;     
      #endif //FEATURE_TIMED_BUFFER  
      #ifdef FEATURE_TIMED_BUFFER      
      case 'T': initiate_timed_buffer(); break;           // T - initiate timed tracking
      #endif //FEATURE_TIMED_BUFFER
      case 'X':  // X - azimuth speed change
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: X cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_x_command();
        break;                               
      #ifdef FEATURE_ELEVATION_CONTROL
      case 'U':  // U - manual up rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: U cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_UP,0);
        Serial.println();
        break;            
      case 'D':  // D - manual down rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: D cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_DOWN,0);
        Serial.println();
        break;          
      case 'E':  // E - stop elevation rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: E cmd"));}
        #endif //DEBUG_SERIAL
        submit_request(EL,REQUEST_STOP,0);
        Serial.println();
        break;          
      case 'B': report_current_elevation(); break;        // B - return current elevation   
      #endif
      #ifdef FEATURE_ELEVATION_CONTROL
      case 'W':  // W - auto elevation rotation
        #ifdef DEBUG_SERIAL
        if (debug_mode) {Serial.println(F("yaesu_serial_command: W cmd"));}
        #endif //DEBUG_SERIAL
        yaesu_w_command();
        break;   
      #endif    
      #ifdef OPTION_GS_232B_EMULATION
      case 'P': yaesu_p_command(); break;                 ged// P - switch between 360 and 450 degree mode
      case 'Z':                                           // Z - Starting point toggle
        if (configuration.azimuth_starting_point == 180) 
        {
          configuration.azimuth_starting_point = 0;
          Serial.println(F("N Center"));
        } else 
        {
          configuration.azimuth_starting_point = 180;
          Serial.println(F("S Center"));
        }
        write_settings_to_eeprom();
        break;
      #endif
      default: 
        Serial.println(F("?>"));
        #ifdef DEBUG_SERIAL
        if (debug_mode) {
          Serial.print(F("check_serial: serial0_buffer_index: "));
          Serial.println(serial0_buffer_index);
          for (int debug_x = 0; debug_x < serial0_buffer_index; debug_x++) 
          {
            Serial.print(F("check_serial: serial0_buffer["));
            Serial.print(debug_x);
            Serial.print(F("]: "));
            Serial.print(serial0_buffer[debug_x]);
            Serial.print(F(" "));
            Serial.write(serial0_buffer[debug_x]);
            Serial.println();
          }
        }
        #endif //DEBUG_SERIAL
    }
    clear_command_buffer();
  }  
}
#endif //FEATURE_YAESU_EMULATION

void check_serial(){
  if (Serial.available()) 
  {
    if (serial_led) 
    {
      digitalWrite(serial_led, HIGH);   // blink the LED just to say we got something
    }
    
    incoming_serial_byte = Serial.read();
    last_serial_receive_time = millis();
        
    if ((incoming_serial_byte == 92) && (serial0_buffer_index == 0)) 
    { 
      // do we have a backslash command?
      serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
      serial0_buffer_index++;
      backslash_command = 1;
      return;
    }
  
    if (backslash_command) 
    {
      if (incoming_serial_byte == 13) 
      {  // do we have a carriage return?
        switch(serial0_buffer[1])
        {
          case 'D':  // toggle debug mode                                                                  // \D - Debug, invert debug state
            if (debug_mode) 
            {
              debug_mode = 0;
              Serial.println(F("debug mode 0"));
            } 
            else 
            {
              debug_mode = 1;
              Serial.println(F("debug mode 1"));
            } 
            break;    

          case 'E' :  // initialize eeprom                                                              // \E - Initialize eeprom
            initialize_eeprom_with_defaults();
            Serial.println(F("Initialized eeprom, please reset..."));
            break;

          case 'C':   // display calibration                                                                 // \C - read calibration memory
            display_calibration_settings();
            break;
            
          case 'L':   // rotate 180 degrees                                                               // \L - rotate to long path
            if (azimuth < (180*HEADING_MULTIPLIER))
            {
              submit_request(AZ,REQUEST_AZIMUTH,(azimuth+(180*HEADING_MULTIPLIER)));
            } else 
            {
              submit_request(AZ,REQUEST_AZIMUTH,(azimuth-(180*HEADING_MULTIPLIER)));
            }
            Serial.print("command: L, azimuth ");
            Serial.print(azimuth);
            Serial.println();
            break;
              
          #ifdef FEATURE_HOST_REMOTE_PROTOCOL
          case 'R' :
            Serial.print(F("Remote port rx sniff o"));
            if (remote_port_rx_sniff)
            {
              remote_port_rx_sniff = 0;
              Serial.println("ff");
            } else 
            {
              remote_port_rx_sniff = 1;
              Serial.println("n");
            }
            break;
          case 'S':
            for (int x = 2;x < serial0_buffer_index;x++)
            {
              Serial1.write(serial0_buffer[x]); 
              if (remote_port_tx_sniff)
              {
                Serial.write(serial0_buffer[x]);              
              }
            }
            Serial1.write(13);
            if (remote_port_tx_sniff)
            {
              Serial.write(13);              
            }            
            break;
          case 'T' :
            Serial.print(F("Remote port tx sniff o"));
            if (remote_port_tx_sniff)
            {
              remote_port_tx_sniff = 0;
              Serial.println("ff");
            } else 
            {
              remote_port_tx_sniff = 1;
              Serial.println("n");
            }
            break;            
          case 'Z' :
            Serial.print(F("Suspend auto remote commands o"));
            if (suspend_remote_commands)
            {
              suspend_remote_commands = 0;
              Serial.println("ff");
            } else 
            {
              suspend_remote_commands = 1;
              Serial.println("n");
            }
            break;              
          #endif //FEATURE_HOST_REMOTE_PROTOCOL
          
          #ifdef FEATURE_ANCILLARY_PIN_CONTROL
          case 'N' :  // \Nxx - turn pin on; xx = pin number
            if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58) && (serial0_buffer_index == 4)){           
              byte pin_value = 0;
              if (toupper(serial0_buffer[2]) == 'A')
              {
                pin_value = get_analog_pin(serial0_buffer[3]-48);
              } else 
              {
                pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
              }
              pinMode(pin_value,OUTPUT);
              digitalWrite(pin_value,HIGH);
              Serial.println("OK");                          
            } else 
            {
              Serial.println(F("Error"));  
            }       
            break;
          case 'F' :  // \Fxx - turn pin off; xx = pin number
            if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58) && (serial0_buffer_index == 4)){           
              byte pin_value = 0;
              if (toupper(serial0_buffer[2]) == 'A')
              {
                pin_value = get_analog_pin(serial0_buffer[3]-48);
              } else 
              {
                pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
              }
              pinMode(pin_value,OUTPUT);
              digitalWrite(pin_value,LOW);
              Serial.println("OK");                          
            } else 
            {
              Serial.println(F("Error"));  
            }       
            break;         
        case 'P' :  // \Pxxyyy - turn on pin PWM; xx = pin number, yyy = PWM value (0-255)
          if (((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)  && (serial0_buffer_index == 7)){
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*100) + ((serial0_buffer[5]-48)*10) + (serial0_buffer[6]-48);
            if ((write_value >= 0) && (write_value < 256))
            {
              pinMode(pin_value,OUTPUT);
              analogWrite(pin_value,write_value);
              Serial.println("OK");
            } else 
            {
              Serial.println(F("Error"));               
            }
          } else 
          {
            Serial.println(F("Error")); 
          }
          break;         
          #endif //FEATURE_ANCILLARY_PIN_CONTROL 
          
          default: Serial.println(F("error"));        
        }
        clear_command_buffer();
        backslash_command = 0;
        
      } else // no, add the character to the buffer
      { 
        if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {incoming_serial_byte = incoming_serial_byte - 32;} //uppercase it
        if (incoming_serial_byte != 10) // add it to the buffer if it's not a line feed
        { 
          serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
          serial0_buffer_index++;
        }
      }
      
    } else 
    {       
      #ifdef FEATURE_YAESU_EMULATION
      yaesu_serial_command();
      #endif //FEATURE_YAESU_EMULATION
      
      #ifdef FEATURE_EASYCOM_EMULATION
      easycom_serial_commmand();
      #endif //FEATURE_EASYCOM_EMULATION
      
      #ifdef FEATURE_REMOTE_UNIT_SLAVE
      remote_unit_serial_command();
      #endif //FEATURE_REMOTE_UNIT_SLAVE
    }
    
    if (serial_led) 
    {
      digitalWrite(serial_led, LOW);
    }
  } //if (Serial.available())

  
  #ifdef OPTION_SERIAL1_SUPPORT
  if (Serial1.available()){
    incoming_serial_byte = Serial1.read();
    #ifdef FEATURE_REMOTE_UNIT_SLAVE 
    if (serial_read_event_flag[1])
    {
      Serial.print("EVS1");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    } 
    #endif //FEATURE_REMOTE_UNIT_SLAVE
    #ifdef FEATURE_HOST_REMOTE_PROTOCOL
    if (remote_port_rx_sniff) {Serial.write(incoming_serial_byte);}
    if ((incoming_serial_byte != 10) && (serial1_buffer_index < COMMAND_BUFFER_SIZE))
    {
      //incoming_serial_byte = toupper(incoming_serial_byte);
      serial1_buffer[serial1_buffer_index] = incoming_serial_byte;
      serial1_buffer_index++;
      if ((incoming_serial_byte == 13) || (serial1_buffer_index == COMMAND_BUFFER_SIZE))
      {
        serial1_buffer_carriage_return_flag = 1;
      } 
    }
    serial1_last_receive_time = millis();
    #endif //FEATURE_HOST_REMOTE_PROTOCOL
  }
  #endif //OPTION_SERIAL1_SUPPORT

  #ifdef OPTION_SERIAL2_SUPPORT
  if (Serial2.available())
  {
    incoming_serial_byte = Serial2.read(); 
    #ifdef FEATURE_REMOTE_UNIT_SLAVE   
    if (serial_read_event_flag[2]
    ){
      Serial.print("EVS1");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    }  
    #endif //FEATURE_REMOTE_UNIT_SLAVE
  }
  #endif //OPTION_SERIAL2_SUPPORT
  
  #ifdef OPTION_SERIAL3_SUPPORT
  if (Serial3.available())
  {
    incoming_serial_byte = Serial3.read();  
    #ifdef FEATURE_REMOTE_UNIT_SLAVE 
    if (serial_read_event_flag[3])
    {
      Serial.print("EVS3");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    }  
    #endif //FEATURE_REMOTE_UNIT_SLAVE
  }
  #endif //OPTION_SERIAL3_SUPPORT 
  
  #ifdef OPTION_SERIAL4_SUPPORT
  if (Serial4.available())
  {
    incoming_serial_byte = Serial4.read(); 
    #ifdef FEATURE_REMOTE_UNIT_SLAVE   
    if (serial_read_event_flag[4])
    {
      Serial.print("EVS4");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    }  
    #endif //FEATURE_REMOTE_UNIT_SLAVE
  }
  #endif //OPTION_SERIAL4_SUPPORT  
}

  #ifdef FEATURE_ELEVATION_CONTROL
  if (axis == EL) 
  {
    #ifdef DEBUG_SUBMIT_REQUEST
    if (debug_mode) {Serial.print(F("EL "));Serial.print(request);Serial.print(F(" "));Serial.println(parm);}
    #endif //DEBUG_SUBMIT_REQUEST
    el_request = request;
    el_request_parm = parm;
    el_request_queue_state = IN_QUEUE;
  }   
  #endif //FEATURE_ELEVATION_CONTROL 

//--------------------------------------------------------------
void clear_command_buffer()
{
  serial0_buffer_index = 0;
  serial0_buffer[0] = 0;
}

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void yaesu_x_command() 
{  
  if (serial0_buffer_index > 1) 
  {
    switch (serial0_buffer[1]) 
    {
      case '4':
        normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;        
        update_az_variable_outputs(PWM_SPEED_VOLTAGE_X4);
        #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
        normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;        
        update_el_variable_outputs(PWM_SPEED_VOLTAGE_X4);        
        #endif
        Serial.print(F("Speed X4\r\n")); 
        break; 
      case '3':
        normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X3;  
        update_az_variable_outputs(PWM_SPEED_VOLTAGE_X3);
        #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
        normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X3;        
        update_el_variable_outputs(PWM_SPEED_VOLTAGE_X3);        
        #endif        
        Serial.print(F("Speed X3\r\n"));          
        break; 
      case '2':
        normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X2;  
        update_az_variable_outputs(PWM_SPEED_VOLTAGE_X2);  
        #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
        normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X2;        
        update_el_variable_outputs(PWM_SPEED_VOLTAGE_X2);        
        #endif        
        Serial.print(F("Speed X2\r\n"));          
        break; 
      case '1':
        normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X1;
        update_az_variable_outputs(PWM_SPEED_VOLTAGE_X1); 
        #if defined(FEATURE_ELEVATION_CONTROL) && defined(OPTION_EL_SPEED_FOLLOWS_AZ_SPEED)
        normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X1;        
        update_el_variable_outputs(PWM_SPEED_VOLTAGE_X1);        
        #endif        
        Serial.print(F("Speed X1\r\n"));          
        break; 
      default: Serial.println(F("?>")); break;            
    }     
  } else 
  {
      Serial.println(F("?>"));  
  }  
}
#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
#ifdef OPTION_GS_232B_EMULATION
void yaesu_p_command()
{
  if ((serial0_buffer[1] == '3') && (serial0_buffer_index > 2)) 
  {  // P36 command
    configuration.azimuth_rotation_capability = 360;
    Serial.print(F("Mode 360 degree\r\n"));
    write_settings_to_eeprom();  
  } else 
  {
    if ((serial0_buffer[1] == '4') && (serial0_buffer_index > 2)) 
    { // P45 command
      configuration.azimuth_rotation_capability = 450;
      Serial.print(F("Mode 450 degree\r\n"));
      write_settings_to_eeprom();
    } else {
      Serial.println(F("?>"));  
    }
  }
}
#endif //OPTION_GS_232B_EMULATION
#endif //FEATURE_YAESU_EMULATION
//--------------------------------------------------------------

#ifdef FEATURE_YAESU_EMULATION
void yaesu_o_command() // fully ccw, Offset command
{

  #ifdef FEATURE_ELEVATION_CONTROL
  if ((serial0_buffer[1] == '2') && (serial0_buffer_index > 1)) 
  {     // did we get the O2 command?
    yaesu_o2_command();
    return;
  }
  #endif

  clear_serial_buffer();

  Serial.println(F("Rotate to full CCW and send keystroke..."));
  get_keystroke();
  read_azimuth(); // NOP if using interrupts
  configuration.Raz_full_ccw = Raz; // azimuth before mapping
  write_settings_to_eeprom();
  read_settings_from_eeprom();

  Serial.print("O command:");
  Serial.print(" Raz ");
  Serial.print(Raz);
  Serial.print(", config.Raz_full_ccw ");
  Serial.print(configuration.Raz_full_ccw);
  Serial.println();
}
#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void print_wrote_to_memory()
{
  #ifdef DEBUG_EEPROM
  Serial.println(F("Wrote to memory"));
  #endif
}

#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void clear_serial_buffer()
{
  delay(200);
  while (Serial.available()) {incoming_serial_byte = Serial.read();}  
}

#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
#ifdef FEATURE_YAESU_EMULATION
void yaesu_f_command() // fully cw, Fullscale command
{
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((serial0_buffer[1] == '2') && (serial0_buffer_index > 1)) 
  {     // did we get the F2 command?
    yaesu_f2_command();
    return;
  }
  #endif

  clear_serial_buffer();

  Serial.println(F("Rotate to full CW and send keystroke..."));
  get_keystroke();
  read_azimuth(); // NOP is using interrupts
  configuration.Raz_full_cw = Raz; // azimuth before mapping
  write_settings_to_eeprom();
  read_settings_from_eeprom();

  Serial.print("F command:");
  Serial.print(" Raz ");
  Serial.print(Raz);
  Serial.print(", config.Raz_full_cw ");
  Serial.print(configuration.Raz_full_cw);
  Serial.println();
}
#endif //FEATURE_YAESU_EMULATION

//--------------------------------------------------------------
void yaesu_m_command()
{
  int parsed_azimuth = 0;
  
  // parse out M command
  if (serial0_buffer_index > 4) 
  {  // if there are more than 4 characters in the command buffer, we got a timed interval command
    #ifdef FEATURE_TIMED_BUFFER
    yaesu_az_load_timed_intervals();
    #else
    Serial.println(F("Feature not activated ?>"));
    #endif //FEATURE_TIMED_BUFFER
    return;
  } else 
  {                         // if there are four characters, this is just a single direction setting
    if (serial0_buffer_index == 4){
      parsed_azimuth = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
      #ifdef FEATURE_TIMED_BUFFER
      clear_timed_buffer();
      #endif //FEATURE_TIMED_BUFFER
      if ((parsed_azimuth > -1) && (parsed_azimuth <= (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability))) 
      {    
        submit_request(AZ,REQUEST_AZIMUTH,(parsed_azimuth*HEADING_MULTIPLIER));
        return;
      }
    }
  }
  Serial.println(F("?>"));

}//--------------------------------------------------------------
#if defined(FEATURE_YAESU_EMULATION) && defined(FEATURE_ELEVATION_CONTROL)
void yaesu_o2_command()
{
  clear_serial_buffer();
  Serial.println(F("Elevate to 0 degrees and send keystroke..."));
  get_keystroke();
  read_elevation();
  configuration.analog_el_0_degrees = analog_el;
  write_settings_to_eeprom();
  print_wrote_to_memory();
}
#endif //defined(FEATURE_YAESU_EMULATION) && defined(FEATURE_ELEVATION_CONTROL)

//--------------------------------------------------------------
#if defined(FEATURE_YAESU_EMULATION) && defined(FEATURE_ELEVATION_CONTROL)
void yaesu_f2_command()
{
  clear_serial_buffer();
  Serial.println(F("Elevate to 180 (or max elevation) and send keystroke..."));
  get_keystroke();
  read_elevation();
  configuration.analog_el_max_elevation = analog_el;
  write_settings_to_eeprom();
  print_wrote_to_memory();
}
#endif //defined(FEATURE_YAESU_EMULATION) && defined(FEATURE_ELEVATION_CONTROL)


//--------------------------------------------------------------
void report_current_azimuth() 
{
  #ifdef FEATURE_YAESU_EMULATION
  // The C command that reports azimuth

  String azimuth_string;

  #ifndef OPTION_GS_232B_EMULATION
  Serial.print(F("+0"));
  #endif
  #ifdef OPTION_GS_232B_EMULATION
  Serial.print(F("AZ="));
  #endif
  //Serial.write("report_current_azimith: azimuth=");
  //Serial.println(azimuth);
  azimuth_string = String(int(azimuth/HEADING_MULTIPLIER), DEC);
  if (azimuth_string.length() == 1) {
    Serial.print(F("00"));
  } else 
  {
    if (azimuth_string.length() == 2) 
    {
      Serial.print(F("0"));
    }
  }
  Serial.print(azimuth_string);

  #ifdef FEATURE_ELEVATION_CONTROL
  #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
  if ((serial0_buffer[1] == '2') && (serial0_buffer_index > 1)) 
  {     // did we get the C2 command?
  #endif
    report_current_elevation();
  #ifndef OPTION_C_COMMAND_SENDS_AZ_AND_EL
  } else 
  {
    Serial.println();
  }
  #endif //OPTION_C_COMMAND_SENDS_AZ_AND_EL
  #endif //FEATURE_ELEVATION_CONTROL
  
  #ifndef FEATURE_ELEVATION_CONTROL
  if ((serial0_buffer[1] == '2') && (serial0_buffer_index > 1)) 
  {     // did we get the C2 command?
    #ifndef OPTION_GS_232B_EMULATION
    Serial.println(F("+0000"));    // return a dummy elevation since we don't have the elevation feature turned on
    #else
    Serial.println(F("EL=000"));
    #endif
  } else 
  {
    Serial.println();
  }
  #endif //FEATURE_ELEVATION_CONTROL
  #endif //FEATURE_YAESU_EMULATION
}

//--------------------------------------------------------------
void get_keystroke()
{
    while (Serial.available() == 0) {}
    while (Serial.available() > 0) 
    {
      incoming_serial_byte = Serial.read();
    }
}
//--------------------------------------------------------------
#ifdef FEATURE_EASYCOM_EMULATION
void easycom_serial_commmand()
{
  /* Easycom protocol implementation
   
  Implemented commands:
  
  Command		Meaning			Parameters
  -------		-------			----------
  AZ		        Azimuth			number - 1 decimal place
  EL		        Elevation		number - 1 decimal place  
  
  ML		        Move Left
  MR		        Move Right
  MU		        Move Up
  MD		        Move Down
  SA		        Stop azimuth moving
  SE		        Stop elevation moving
  
  VE		        Request Version
  
  Easycom has no way to report azimuth or elevation back to the client, or report errors  
  */
  
  float heading = -1;

  if ((incoming_serial_byte != 13) && (incoming_serial_byte != 10) && (incoming_serial_byte != 32) && (serial0_buffer_index < COMMAND_BUFFER_SIZE))
  { // if it's not a CR, LF, or space, add it to the buffer
    if ((incoming_serial_byte > 96) && (incoming_serial_byte < 123)) {incoming_serial_byte = incoming_serial_byte - 32;} //uppercase it
    serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
    serial0_buffer_index++;
  } else 
  {                       // time to get to work on the command
    if (serial0_buffer_index)
    {
      switch (serial0_buffer[0]) 
      {                  // look at the first character of the command
        case 'A':  //AZ
          if (serial0_buffer[1] == 'Z')
          {   // format is AZx.x or AZxx.x or AZxxx.x (why didn't they make it fixed length?)
            switch (serial0_buffer_index) 
            {
              #ifdef OPTION_EASYCOM_AZ_QUERY_COMMAND
              case 2:
                Serial.print("AZ");
                Serial.println(float(azimuth*HEADING_MULTIPLIER),1);
                clear_command_buffer();
                return;
                break;
              #endif //OPTION_EASYCOM_AZ_QUERY_COMMAND
              case 5: // format AZx.x
                heading = (serial0_buffer[2]-48) + ((serial0_buffer[4]-48)/10);
                break;
              case 6: // format AZxx.x 
                heading = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48) + ((serial0_buffer[5]-48)/10);
                break;
              case 7: // format AZxxx.x
                heading = ((serial0_buffer[2]-48)*100) + ((serial0_buffer[3]-48)*10) + (serial0_buffer[4]-48) + ((serial0_buffer[6]-48)/10);
                break;
              //default: Serial.println("?"); break;
            }
            if (((heading >= 0) && (heading < 451))  && (serial0_buffer[serial0_buffer_index-2] == '.'))
            {
              submit_request(AZ,REQUEST_AZIMUTH,(heading*HEADING_MULTIPLIER));
            } else 
            {
              Serial.println("?");
            }
          } else {
            Serial.println("?");
          }
          break;        
        #ifdef FEATURE_ELEVATION_CONTROL
        case 'E':  //EL
          if (serial0_buffer[1] == 'L') 
          {
            switch (serial0_buffer_index) 
            {
              #ifdef OPTION_EASYCOM_EL_QUERY_COMMAND
              case 2:
                Serial.print("EL");
                Serial.println(float(elevation*HEADING_MULTIPLIER),1);
                clear_command_buffer();
                return;
                break;              
              #endif //OPTION_EASYCOM_EL_QUERY_COMMAND
              case 5: // format ELx.x
                heading = (serial0_buffer[2]-48) + ((serial0_buffer[4]-48)/10);
                break;
              case 6: // format ELxx.x 
                heading = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48) + ((serial0_buffer[5]-48)/10);
                break;
              case 7: // format ELxxx.x
                heading = ((serial0_buffer[2]-48)*100) + ((serial0_buffer[3]-48)*10) + (serial0_buffer[4]-48) + ((serial0_buffer[6]-48)/10);
                break;
              //default: Serial.println("?"); break;
            }
            if (((heading >= 0) && (heading < 181)) && (serial0_buffer[serial0_buffer_index-2] == '.')){
              submit_request(EL,REQUEST_ELEVATION,(heading*HEADING_MULTIPLIER));
            } else 
            {
              Serial.println("?");
            }
          } else 
          {
            Serial.println(F("?"));
          }
          break;
        #endif //#FEATURE_ELEVATION_CONTROL
        case 'S':  // SA or SE - stop azimuth, stop elevation
          switch (serial0_buffer[1]) 
          {
            case 'A':
              submit_request(AZ,REQUEST_STOP,0);
              break;
            #ifdef FEATURE_ELEVATION_CONTROL
            case 'E':
              submit_request(EL,REQUEST_STOP,0);
              break; 
            #endif //FEATURE_ELEVATION_CONTROL
            default: Serial.println("?"); break;
          }
          break;
        case 'M':  // ML, MR, MU, MD - move left, right, up, down
          switch (serial0_buffer[1])
          {
            case 'L': // ML - move left
              submit_request(AZ,REQUEST_CCW,0);
              break;
            case 'R': // MR - move right
              submit_request(AZ,REQUEST_CW,0);
              break;
            #ifdef FEATURE_ELEVATION_CONTROL
            case 'U': // MU - move up
              submit_request(EL,REQUEST_UP,0);
              break;
            case 'D': // MD - move down
              submit_request(EL,REQUEST_DOWN,0);
              break;
            #endif //FEATURE_ELEVATION_CONTROL
            default: Serial.println(F("?")); break;
          }
          break;
        case 'V': // VE - version query
          if (serial0_buffer[1] == 'E') {Serial.println(F("VE002"));} // not sure what to send back, sending 002 because this is easycom version 2?
          break;    
        default: Serial.println("?"); break;
      }
    } 
    clear_command_buffer();
  } 
}
#endif //FEATURE_EASYCOM_EMULATION
