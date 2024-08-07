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


   ***************************************************************************************************************    

    ***** https://github.com/k3ng/k3ng_rotator_controller/wiki *****


    All copyrights are the property of their respective owners

     ASCII Art Schematic
 
                               +----------------Yaesu Pin 1
                               |
                               N
            D6---{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 2
                               |
                               N
            D7---{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
            A0-------------+---------------------Yaesu Pin 4
                           |
                        [0.01uF]
                           |
                          GND
 
            D10---{4.7K}---+---------------------Yaesu Pin 3
                           |
                         [10uF]
                           |
                          GND
                               
                               
            Not Connected------------------------Yaesu Pin 6
 
            GND----------------------------------Yaesu Pin 5

    Alternatively Yaesu Pin 3 can be connected to Pin 6 if variable speed functionality (X commands) are not desired.  This will feed +5V directly
    into the speed voltage pin and set the unit for maximum speed all the time

    Yaesu Azimuth Only Rotator Controller Connector Pins
 
       6 || 5
      4      3
        2  1
    1 - ground to rotate L
    2 - ground to rotate R
    3 - speed voltage (input); 4.5V = max speed
    4 - analog azimuth voltage (output); 0V = full CCW, ~4.9V = full CW
    5 - ground
    6 - +5V or so

    Yaesu Az/El Rotator Control Connector Pins
 
        7 | | 6
       3   8   1
        5     4
           2
 
    1 - 2 - 4.5 VDC corresponding to 0 to 180 degrees elevation
    2 - Connect to Pin 8 to rotate right (clockwise)
    3 - Connect to Pin 8 to rotate Down
    4 - Connect to Pin 8 to rotate left (counterclockwise)
    5 - Connect to Pin 8 to rotate Up
    6 - 2 - 4.5 VDC corresponding to 0 to 450 degrees rotation
    7 - 13 - 6 VDC at up to 200 mA
    8 - Common ground

     ASCII Art Schematic
 
                               +----------------Yaesu Pin 4
                               |
                               N
            D6--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 2
                               |
                               N
            D7--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
                               +----------------Yaesu Pin 5
                               |
                               N
            D8--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
                               +----------------Yaesu Pin 3
                               |
                               N
            D9--{1000 ohms}---P      2N2222
                               N    or similar
                               |
                              GND
 
            A0-----------------------------------Yaesu Pin 6
 
            A1-----------------------------------Yaesu Pin 1
 
            NC-----------------------------------Yaesu Pin 7
 
            GND----------------------------------Yaesu Pin 8

    Quick Start
 
    In order to test and calibrate your unit, connect the Serial Monitor to the COM port set for 9600 and carriage return
    All command letters must be uppercase.
    The backslash d (\d) command toggles debug mode which will periodically display key parameters.
 
    To test basic operation, send commands using Serial Monitor:
    Rotate left(CCW): L 
    Rotate right(CW): R
    Stop rotation: A or S commands
    Read the current azimuth: C
    Go to an azimuth automatically: M command (examples: M180 = go to 180 degrees, M010 = go to 10 degrees
 
    To calibrate the unit, send the O command and rotate to 180 degrees / full CCW and send a carriage return, then
    send the F command and rotate to 270 degrees / full CW and send a carriage return (assuming a 450 degrees rotation rotator).
    If you are operating a 360 degree rotation rotator, for the F command rotate to 180 degrees full CW, not 270.
 
    ( CW means clockwise (or LEFT on Yaesu rotators) and CCW means counter clockwise (or RIGHT on Yaesu rotators))
 
    To use this code with AZ/EL rotators, uncomment the FEATURE_ELEVATION_CONTROL line below
 
    It does properly handle the 450 degree rotation capability of the Yaesu rotators.
 
    This code has been successfully interfaced with non-Yaesu rotators. Email me if you have a rotator you would like to interface this to.
 
    With the addition of a reasonable capacity DC power supply and two relays, this unit could entirely replace a control unit if desired.

    9/12/11 W3SA JJE added code to correct elevation display which was not following A1 input (map function was not working using the variables)
    Added code to keep azimuth and elevation updated if changed from the rotor control unit.
    Added code to handle flipped azimuth of antenna when elevation passes 90 degrees.
    Changed LCD display to display Direction, Azimuth and Elevation of antenna(s) on first line of display and actual Rotor azimuth and elevation on the second line
    Then when the elevation has passed 90 degrees you would get:
        NNE A 15 E 75
 		RTR A 195 E 115
    Otherwise it would be
        NNE A 15 E 75
 		RTR A 15 E 75
 
 
    01/02/13 M0CKE added code for a cheap rotary encoder input for setting the azimuth direction, rotary encoder connected to pins 6 & 7 with the center to ground,
    degrees are set by turning the encoder cw or ccw with 1 click being 1 degree if turned slowly or 1 click being 10 degree by turning quickly, this replaces the 
    preset pot and can be enabled by uncommenting "#define FEATURE_AZ_PRESET_ENCODER" below.
*/

// Project configuration
#include "dependencies.h"

FIR<float, 31> lpf_top;
FIR<float, 31> lpf_bot;

// define external functions
void check_serial();
void read_headings();
void az_check_operation_timeout();
void check_overlap();
void output_debug();
void profile_loop_time();
void TimedService();
void ReadAzimuthCDE();
void check_az_manual_rotate_limit();

#ifdef FEATURE_ROTATION_INDICATOR_PIN
void service_rotation_indicator_pin();
#endif //FEATURE_ROTATION_INDICATOR_PIN  

// Enter at TimeInterval specified in global
// callback from interrupt
// ReadAzimuthCDE() to read ADCs
// Sets DisplayFlag when it's time to update Display
// Entered at TIME_BETWEEN_AZ_ADC_READ msec
void TimedService() 
{
  // read azimuth on every interrupt
  ReadAzimuthCDE();

  // read buttons on every interrupt
  // if button pressed increment press time value, else clear press time value
  if (digitalRead(button_cw_pin) == LOW) // might be first press or continued press
  {
    if (button_cw_press_time < 100000) // count up to 100 seconds, arbitrary limit
    {
      button_cw_press_time += TIME_BETWEEN_AZ_ADC_READ;
    }
  }
  else
  {
    button_cw_press_time = 0;
  }

  if (digitalRead(button_ccw_pin) == LOW) // might be first press or continued press
  {
    if (button_ccw_press_time < 100000) // count up to 100 seconds, arbitrary limit
    {
      button_ccw_press_time += TIME_BETWEEN_AZ_ADC_READ;
    }
  }
  else
  {
    button_ccw_press_time = 0; // reset on any bounce off
  }
} // TimedService()


/* -------------------------------------- subroutines -----------------------------------------------
   Where the real work happens...
*/
void read_headings()
{
  read_azimuth();
  
  #ifdef FEATURE_ELEVATION_CONTROL
  read_elevation();
  #endif 
} // read_headings()

//--------------------------------------------------------------
void profile_loop_time()
{
  #ifdef DEBUG_PROFILE_LOOP_TIME 
  static unsigned long last_time = 0;
  static unsigned long last_print_time = 0;

  average_loop_time = (average_loop_time + (millis()-last_time))/2.0;
  last_time = millis();
    
  if (debug_mode)
  {
    if ((millis()-last_print_time) > 1000)
    {
      Serial.print(F("avg loop time: "));
      Serial.println(average_loop_time,2);
      last_print_time = millis();
    }
  }
  #endif //DEBUG_PROFILE_LOOP_TIME  
} // profile_loop_time()

//--------------------------------------------------------------
void check_az_manual_rotate_limit() 
{
  if ((current_az_state() == ROTATING_CCW) && (raw_azimuth < (AZ_MANUAL_ROTATE_CCW_LIMIT*HEADING_MULTIPLIER))) 
  {
    submit_request(AZ, REQUEST_KILL, 0);       
    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_az_manual_rotate_limit: stopping, "));
      Serial.print("raw_azimuth ");
      Serial.print(raw_azimuth);
      Serial.print(" <= CCW Limit ");
      Serial.println(AZ_MANUAL_ROTATE_CCW_LIMIT);
    } 
    #endif //DEBUG_AZ_MANUAL_ROTATE_LIMITS
  }
  if ((current_az_state() == ROTATING_CW) && (raw_azimuth > (AZ_MANUAL_ROTATE_CW_LIMIT*HEADING_MULTIPLIER))) 
  {
    submit_request(AZ, REQUEST_KILL, 0);   

    #ifdef DEBUG_AZ_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_az_manual_rotate_limit: stopping, "));
      Serial.print("raw_azimuth ");
      Serial.print(raw_azimuth);
      Serial.print(" >- CW Limit ");
      Serial.println(AZ_MANUAL_ROTATE_CW_LIMIT);
    } 
    #endif //DEBUG_AZ_MANUAL_ROTATE_LIMITS
  }
} // check_az_manual_rotate_limit() 

//--------------------------------------------------------------
#if defined(OPTION_EL_MANUAL_ROTATE_LIMITS) && defined(FEATURE_ELEVATION_CONTROL)
void check_el_manual_rotate_limit() 
{
  if ((current_el_state() == ROTATING_DOWN) && (elevation <= (EL_MANUAL_ROTATE_DOWN_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_DOWN_LIMIT of "));
      Serial.println(EL_MANUAL_ROTATE_DOWN_LIMIT);
    } 
    #endif //DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL,REQUEST_KILL,0);       
  }
  if ((current_el_state() == ROTATING_UP) && (elevation >= (EL_MANUAL_ROTATE_UP_LIMIT*HEADING_MULTIPLIER))) 
  {
    #ifdef DEBUG_EL_MANUAL_ROTATE_LIMITS
    if (debug_mode) 
    {
      Serial.print(F("check_el_manual_rotate_limit: stopping - hit EL_MANUAL_ROTATE_UP_LIMIT of "));
      Serial.println(EL_MANUAL_ROTATE_UP_LIMIT);
    } 
    #endif //DEBUG_EL_MANUAL_ROTATE_LIMITS
    submit_request(EL,REQUEST_KILL,0);   
  }
} // check_el_manual_rotate_limit() 
#endif //#ifdef OPTION_EL_MANUAL_ROTATE_LIMITS


//--------------------------------------------------------------
void check_overlap()
{
  static byte overlap_led_status = 0;
  static unsigned long last_check_time;
  
  if ((overlap_led) && ((millis() - last_check_time) > 500)) 
  {
     //if ((Raz > (500*HEADING_MULTIPLIER)) && (azimuth > (RAZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (!overlap_led_status)) {
     if ((azimuth > (RAZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (!overlap_led_status)) 
     {
       digitalWrite(overlap_led, HIGH);
       overlap_led_status = 1;
       #ifdef DEBUG_OVERLAP
       if (debug_mode) 
       {
         Serial.print(F("check_overlap: in overlap\r\n"));
       }
       #endif //DEBUG_OVERLAP
     } else 
     {
       //if (((Raz < (500*HEADING_MULTIPLIER)) || (azimuth < (RAZ_OVERLAP_DEGREES*HEADING_MULTIPLIER))) && (overlap_led_status)) {
       if ((azimuth < (RAZ_OVERLAP_DEGREES*HEADING_MULTIPLIER)) && (overlap_led_status)) 
       {
         digitalWrite(overlap_led, LOW);
         overlap_led_status = 0;
         #ifdef DEBUG_OVERLAP
         if (debug_mode) 
         {
           Serial.print(F("check_overlap: overlap off\r\n"));
         }
         #endif //DEBUG_OVERLAP
       }        
     }
     last_check_time = millis();
  }
} // check_overlap

//--------------------------------------------------------------
#if defined(FEATURE_REMOTE_UNIT_SLAVE) || defined(FEATURE_ANCILLARY_PIN_CONTROL)
byte get_analog_pin(byte pin_number)
{  
  byte return_output = 0;
           
  switch(pin_number)
  {
    case 0: return_output = A0; break;
    case 1: return_output = A1; break;
    case 2: return_output = A2; break;
    case 3: return_output = A3; break;
    case 4: return_output = A4; break;
    case 5: return_output = A5; break; 
    case 6: return_output = A6; break;    
  }  
  return return_output;
} // get_analog_pin
#endif //FEATURE_REMOTE_UNIT_SLAVE

//--------------------------------------------------------------
#ifdef FEATURE_REMOTE_UNIT_SLAVE
void remote_unit_serial_command()
{
    if (serial_read_event_flag[0])
    {
      Serial.print("EVS0");
      Serial.write(incoming_serial_byte); 
      Serial.println();
    }
    
    if ((incoming_serial_byte != 10) && (serial0_buffer_index < COMMAND_BUFFER_SIZE))
    {
      //incoming_serial_byte = toupper(incoming_serial_byte);
      serial0_buffer[serial0_buffer_index] = incoming_serial_byte;
      serial0_buffer_index++;
      if ((incoming_serial_byte == 13) || (serial0_buffer_index == COMMAND_BUFFER_SIZE))
      {
        serial0_buffer_carriage_return_flag = 1;
      } 
    }
} // remote_unit_serial_command
#endif //FEATURE_REMOTE_UNIT_SLAVE 

//--------------------------------------------------------------
#ifdef FEATURE_REMOTE_UNIT_SLAVE  
void service_remote_unit_serial_buffer()
{
/*
  This implements a protocol for host unit to remote unit communications
  Remote Slave Unit Protocol Reference

    PG - ping
    AZ - read azimuth
    EL - read elevation
    DOxx - digital pin initialize as output; 
    DIxx - digital pin initialize as input
    DPxx - digital pin initialize as input with pullup
    DRxx - digital pin read
    DLxx - digital pin write low
    DHxx - digital pin write high
    DTxxyyyy - digital pin tone output
    NTxx - no tone
    ARxx - analog pin read
    AWxxyyy - analog pin write
    SWxy - serial write byte
    SDx - deactivate serial read event; x = port #
    SSxyyyyyy... - serial write sting; x = port #, yyyy = string of characters to send
    SAx - activate serial read event; x = port #
    RB - reboot

  Responses

    ER - report an error (remote to host only)
    EV - report an event (remote to host only)
    OK - report success (remote to host only)
    CS - report a cold start (remote to host only)

  Error Codes

    ER01 - Serial port buffer timeout
    ER02 - Command syntax error

  Events

    EVSxy - Serial port read event; x = serial port number, y = byte returned
*/
  
  String command_string;
  byte command_good = 0;
  
  if (serial0_buffer_carriage_return_flag) 
  {
    
    // TODO: if checksumming is turned on, parse out the checksum, validate it, return an error if invalid, chop it off if valid and continue
    
    if (serial0_buffer_index < 3)
    {
      Serial.println(F("ER02"));  // we don't have enough characters - syntax error
    } else 
    {
      command_string = String(char(toupper(serial0_buffer[0]))) + String(char(toupper(serial0_buffer[1])));
      
      #ifdef DEBUG_SERVICE_SERIAL_BUFFER
      Serial.print(F("serial_serial_buffer: command_string: "));
      Serial.print(command_string);
      Serial.print(F("$ serial0_buffer_index: "));
      Serial.println(serial0_buffer_index);
      #endif //DEBUG_SERVICE_SERIAL_BUFFER
  
      if ((command_string == "SS") && (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
      {  // this is a variable length command
        command_good = 1;
        for (byte x = 3;x < serial0_buffer_index;x++){
          switch(serial0_buffer[2]-48){
            case 0: Serial.write(serial0_buffer[x]); break;
            #ifdef OPTION_SERIAL1_SUPPORT
            case 1: Serial1.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT
            #ifdef OPTION_SERIAL2_SUPPORT
            case 2: Serial2.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT
            #ifdef OPTION_SERIAL3_SUPPORT
            case 3: Serial3.write(serial0_buffer[x]); break;
            #endif //OPTION_SERIAL1_SUPPORT            
          }
        }
      }
      if (serial0_buffer_index == 3) 
      {
        if (command_string == "PG") {Serial.println(F("PG"));command_good = 1;}   // PG - ping
        if (command_string == "RB") {wdt_enable(WDTO_30MS); while(1) {}}         // RB - reboot
        if (command_string == "AZ") 
        {
          Serial.print(F("AZ"));
          if (raw_azimuth < 1000) {Serial.print("0");}
          if (raw_azimuth < 100) {Serial.print("0");}
          if (raw_azimuth < 10) {Serial.print("0");}
          Serial.println(raw_azimuth);
          command_good = 1;
        }
        #ifdef FEATURE_ELEVATION_CONTROL
        if (command_string == "EL") 
        {
          Serial.print(F("EL"));
          if (elevation >= 0) 
          {
            Serial.print("+");
          } else 
          {
            Serial.print("-");
          }
          if (abs(elevation) < 1000) {Serial.print("0");}
          if (abs(elevation) < 100) {Serial.print("0");}
          if (abs(elevation) < 10) {Serial.print("0");}
          Serial.println(abs(elevation));
          command_good = 1;      
        }  
        #endif //FEATURE_ELEVATION_CONTROL
      } // end of three byte commands

      if (serial0_buffer_index == 4) 
      {
        if ((command_string == "SA") & (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
        {
          serial_read_event_flag[serial0_buffer[2]-48] = 1;
          command_good = 1;
          Serial.println("OK");
        }
        if ((command_string == "SD") & (serial0_buffer[2] > 47) && (serial0_buffer[2] < 53))
        {
          serial_read_event_flag[serial0_buffer[2]-48] = 0;
          command_good = 1;
          Serial.println("OK");          
        }          
      }
  
      if (serial0_buffer_index == 5) 
      {
        if (command_string == "SW"){ // Serial Write command
          switch (serial0_buffer[2])
          {
            case '0': Serial.write(serial0_buffer[3]); command_good = 1; break; 
            #ifdef OPTION_SERIAL1_SUPPORT
            case '1': Serial1.write(serial0_buffer[3]); command_good = 1; break;
            #endif //OPTION_SERIAL1_SUPPORT
          } 
        }
       
        if (command_string == "DO")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            #ifdef DEBUG_SERVICE_SERIAL_BUFFER
            Serial.print(F("service_serial_buffer: pin_value: "));
            Serial.println(pin_value);
            #endif //DEBUG_SERVICE_SERIAL_BUFFER
            Serial.println("OK");
            pinMode(pin_value,OUTPUT);
          }        
        }
        
        if (command_string == "DH")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            digitalWrite(pin_value,HIGH);
            Serial.println("OK");
          }         
        }   
       
        if (command_string == "DL")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            digitalWrite(pin_value,LOW);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DI")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            pinMode(pin_value,INPUT);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DP")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58)){
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            //pinMode(pin_value,INPUT_PULLUP);
            pinMode(pin_value,INPUT);
            digitalWrite(pin_value,HIGH);
            Serial.println("OK");
          }         
        }   
   
        if (command_string == "DR")
        {
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            byte pin_read = digitalRead(pin_value);
            Serial.print("DR");
            Serial.write(serial0_buffer[2]);
            Serial.write(serial0_buffer[3]);
            if (pin_read){
              Serial.println("1");
            } else 
            {
              Serial.println("0");
            }
          }         
        }    
        if (command_string == "AR"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int pin_read = analogRead(pin_value);
            Serial.print("AR");
            Serial.write(serial0_buffer[2]);
            Serial.write(serial0_buffer[3]);
            if (pin_read < 1000) {Serial.print("0");}
            if (pin_read < 100) {Serial.print("0");}
            if (pin_read < 10) {Serial.print("0");}
            Serial.println(pin_read);
          }         
        } 
        
        if (command_string == "NT"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            command_good = 1;
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            noTone(pin_value);
            Serial.println("OK");
          }         
        }    
        
      } //if (serial0_buffer_index == 5)
      
      if (serial0_buffer_index == 8) {
        if (command_string == "AW"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A'){
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*100) + ((serial0_buffer[5]-48)*10) + (serial0_buffer[6]-48);
            if ((write_value >= 0) && (write_value < 256)){
              analogWrite(pin_value,write_value);
              Serial.println("OK");
              command_good = 1;
            }
          }         
        }        
      }

      if (serial0_buffer_index == 9) 
      {
        if (command_string == "DT"){
          if ((((serial0_buffer[2] > 47) && (serial0_buffer[2] < 58)) || (toupper(serial0_buffer[2]) == 'A')) && (serial0_buffer[3] > 47) && (serial0_buffer[3] < 58))
          {
            byte pin_value = 0;
            if (toupper(serial0_buffer[2]) == 'A')
            {
              pin_value = get_analog_pin(serial0_buffer[3]-48);
            } else 
            {
              pin_value = ((serial0_buffer[2]-48)*10) + (serial0_buffer[3]-48);
            }
            int write_value = ((serial0_buffer[4]-48)*1000) + ((serial0_buffer[5]-48)*100) + ((serial0_buffer[6]-48)*10) + (serial0_buffer[7]-48);
            if ((write_value >= 0) && (write_value <= 9999))
            {
              tone(pin_value,write_value);
              Serial.println("OK");
              command_good = 1;
            }
          }         
        }        
      }      
      
 
      if (!command_good) {Serial.println(F("ER02"));}
    }
    serial0_buffer_carriage_return_flag = 0;
    serial0_buffer_index = 0; 
  } else 
  {
    if (((millis() - last_serial_receive_time) > REMOTE_BUFFER_TIMEOUT_MS) && serial0_buffer_index)
    {
      Serial.println(F("ER01"));
      serial0_buffer_index = 0; 
    }
  }
} // service_remote_unit_serial_buffer()
#endif //FEATURE_REMOTE_UNIT_SLAVE

//--------------------------------------------------------------
void az_check_operation_timeout()
{
  // check if the last executed rotation operation has been going on too long

  if (((millis() - az_last_rotate_initiation) > OPERATION_TIMEOUT) && (az_state != IDLE)) 
  {
    submit_request(AZ,REQUEST_KILL,0);
    #ifdef DEBUG_AZ_CHECK_OPERATION_TIMEOUT
    if (debug_mode) {Serial.println(F("az_check_operation_timeout: timeout reached, aborting rotation"));}
    #endif //DEBUG_AZ_CHECK_OPERATION_TIMEOUT
  }
} // az_check_operation_timeout()

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void clear_timed_buffer()
{
  timed_buffer_status = EMPTY;
  timed_buffer_number_entries_loaded = 0;
  timed_buffer_entry_pointer = 0;
} // clear_timed_buffer()
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void initiate_timed_buffer()
{
  if (timed_buffer_status == LOADED_AZIMUTHS) 
  {
    timed_buffer_status = RUNNING_AZIMUTHS;
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[1]);
    last_timed_buffer_action_time = millis();
    timed_buffer_entry_pointer = 2;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS"));}
    #endif //DEBUG_TIMED_BUFFER
  } else 
  {
    #ifdef FEATURE_ELEVATION_CONTROL
    if (timed_buffer_status == LOADED_AZIMUTHS_ELEVATIONS) 
    {
      timed_buffer_status = RUNNING_AZIMUTHS_ELEVATIONS;
      submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[1]);
      submit_request(EL,REQUEST_ELEVATION,timed_buffer_elevations[1]);
      last_timed_buffer_action_time = millis();
      timed_buffer_entry_pointer = 2;
      #ifdef DEBUG_TIMED_BUFFER
      if (debug_mode) {Serial.println(F("initiate_timed_buffer: changing state to RUNNING_AZIMUTHS_ELEVATIONS"));}
      #endif //DEBUG_TIMED_BUFFER
    } else 
    {
      Serial.println(">");  // error
    }
    #endif
  }
} // initiate_timed_buffer()
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void print_timed_buffer_empty_message()
{  
  #ifdef DEBUG_TIMED_BUFFER
  if (debug_mode) {Serial.println(F("check_timed_interval: completed timed buffer; changing state to EMPTY"));}
  #endif //DEBUG_TIMED_BUFFER
} // print_timed_buffer_empty_message()

#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void check_timed_interval()
{
  if ((timed_buffer_status == RUNNING_AZIMUTHS) && (((millis() - last_timed_buffer_action_time)/1000) > timed_buffer_interval_value_seconds)) 
  {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("check_timed_interval: executing next timed interval step - azimuths"));}
    #endif //DEBUG_TIMED_BUFFER
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[timed_buffer_entry_pointer-1]);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) 
    {
      clear_timed_buffer();
      print_timed_buffer_empty_message();
    }
  }
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((timed_buffer_status == RUNNING_AZIMUTHS_ELEVATIONS) && (((millis() - last_timed_buffer_action_time)/1000) > timed_buffer_interval_value_seconds)) 
  {
    timed_buffer_entry_pointer++;
    #ifdef DEBUG_TIMED_BUFFER
    if (debug_mode) {Serial.println(F("check_timed_interval: executing next timed interval step - az and el"));}
    #endif //DEBUG_TIMED_BUFFER
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[timed_buffer_entry_pointer-1]);
    submit_request(EL,REQUEST_ELEVATION,timed_buffer_elevations[timed_buffer_entry_pointer-1]);
    last_timed_buffer_action_time = millis();
    if (timed_buffer_entry_pointer == timed_buffer_number_entries_loaded) 
    {
      clear_timed_buffer();
      print_timed_buffer_empty_message();
    }
  }
  #endif
} // check_timed_interval()
#endif //FEATURE_TIMED_BUFFER

//--------------------------------------------------------------
#ifdef FEATURE_TIMED_BUFFER
void yaesu_az_load_timed_intervals()
{
  int parsed_value = 0;

  clear_timed_buffer();

  parsed_value = ((int(serial0_buffer[1])-48)*100) + ((int(serial0_buffer[2])-48)*10) + (int(serial0_buffer[3])-48);
  if ((parsed_value > 0) && (parsed_value < 1000)) {
    timed_buffer_interval_value_seconds = parsed_value;
    for (int x = 5; x < serial0_buffer_index; x = x + 4) {
      parsed_value = ((int(serial0_buffer[x])-48)*100) + ((int(serial0_buffer[x+1])-48)*10) + (int(serial0_buffer[x+2])-48);
      if ((parsed_value > -1) && (parsed_value < 361)) 
      {  // is it a valid azimuth?
        timed_buffer_azimuths[timed_buffer_number_entries_loaded] = parsed_value * HEADING_MULTIPLIER;
        timed_buffer_number_entries_loaded++;
        timed_buffer_status = LOADED_AZIMUTHS;
        if (timed_buffer_number_entries_loaded > TIMED_INTERVAL_ARRAY_SIZE) {   // is the array full?
          submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[0]);  // array is full, go to the first azimuth
          timed_buffer_entry_pointer = 1;
          return;
        }
      } else 
      {   // we hit an invalid bearing
        timed_buffer_status = EMPTY;
        timed_buffer_number_entries_loaded = 0;
        Serial.println(F("?>"));  // error
        return;
      }
    }
    submit_request(AZ,REQUEST_AZIMUTH,timed_buffer_azimuths[0]);   // go to the first azimuth
    timed_buffer_entry_pointer = 1;

  } else 
  {
    Serial.println(F("?>"));  // error
  }
} // yaesu_az_load_timed_intervals()
#endif //FEATURE_TIMED_BUFFER

//-----------------------------------------------------------------
// write to L298N motor control chip
// L298N is a dual H bridge motor controller
// IN1 defined as high side for positive rotation
// PWM on Teensy is 488.28 Hz
#ifdef OPTION_AZIMUTH_MOTOR_DIR_CONTROL
void rotator_speed(byte speed)
{
	if (speed > 0)
	{
		digitalWrite(IN1Pin,      1);
		digitalWrite(IN2Pin,      0);
		analogWrite( ENAPin,  speed);
	}
	if (speed < 0)
	{
		digitalWrite(IN1Pin,      1);
		digitalWrite(IN2Pin,      0);
		analogWrite( ENAPin, -speed);
	}
	if (speed == 0)
	{
		digitalWrite(IN1Pin,      0);
		digitalWrite(IN2Pin,      0);
		digitalWrite(ENAPin,      0);
	}
} // rotator_speed(byte speed)
#endif

//--------------------------------------------------------------
#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
void az_position_pulse_interrupt_handler()
{
  #ifdef DEBUG_POSITION_PULSE_INPUT
  //az_position_pule_interrupt_handler_flag++;
  az_pulse_counter++;
  #endif //DEBUG_POSITION_PULSE_INPUT
  
  if (current_az_state() == ROTATING_CW) 
  {
    az_position_pulse_input_azimuth += AZ_POSITION_PULSE_DEG_PER_PULSE;
    last_known_az_state = ROTATING_CW;
  } else 
  {
    if (current_az_state() == ROTATING_CCW) 
    {
      az_position_pulse_input_azimuth -= AZ_POSITION_PULSE_DEG_PER_PULSE;
      last_known_az_state = ROTATING_CCW;
    } else 
    {
      if (last_known_az_state == ROTATING_CW)
      {
        az_position_pulse_input_azimuth += AZ_POSITION_PULSE_DEG_PER_PULSE;
      } else 
      {
        if (last_known_az_state == ROTATING_CCW)
        {
          az_position_pulse_input_azimuth -= AZ_POSITION_PULSE_DEG_PER_PULSE;
        }
      }
      #ifdef DEBUG_POSITION_PULSE_INPUT
      az_pulse_counter_ambiguous++;
      #endif //DEBUG_POSITION_PULSE_INPUT
    }
  }
  
  #ifdef OPTION_AZ_POSITION_PULSE_HARD_LIMIT
  if (az_position_pulse_input_azimuth < configuration.azimuth_starting_point) 
  {
    az_position_pulse_input_azimuth = configuration.azimuth_starting_point;
  }
  if (az_position_pulse_input_azimuth > (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability)) 
  {
    az_position_pulse_input_azimuth = (configuration.azimuth_starting_point + configuration.azimuth_rotation_capability);
  }
  #else
  if (az_position_pulse_input_azimuth < 0)
  {
    az_position_pulse_input_azimuth += 360;
  }
  if (az_position_pulse_input_azimuth >= 360)
  {
    az_position_pulse_input_azimuth -= 360;
  }
  #endif //OPTION_AZ_POSITION_PULSE_HARD_LIMIT
} // az_position_pulse_interrupt_handler()
#endif //FEATURE_AZ_POSITION_PULSE_INPUT


//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
byte submit_remote_command(byte remote_command_to_send)
{ 
  if (remote_unit_command_submitted || suspend_remote_commands) 
  {
    return 0;
  } else 
  {
    switch(remote_command_to_send)
    {
      case REMOTE_UNIT_AZ_COMMAND:
        Serial1.println("AZ");
        if (remote_port_tx_sniff) {Serial.println("AZ");}
        remote_unit_command_submitted = REMOTE_UNIT_AZ_COMMAND;
        break;
      case REMOTE_UNIT_EL_COMMAND:
        Serial1.println("EL");
        if (remote_port_tx_sniff) {Serial.println("EL");}
        remote_unit_command_submitted = REMOTE_UNIT_EL_COMMAND;
        break;    
    }
    last_remote_unit_command_time = millis();   
    remote_unit_command_results_available = 0;
    return 1;
  }
} // submit_remote_command(byte remote_command_to_send)
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
byte is_ascii_number(byte char_in)
{ 
  if ((char_in > 47) && (char_in < 58)) 
  {
    return 1;
  } else 
  {
    return 0;
  }
} // is_ascii_number(byte char_in)
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_HOST_REMOTE_PROTOCOL
void service_remote_communications_incoming_serial_buffer()
{
  byte good_data = 0;
  
  if (serial1_buffer_carriage_return_flag)
  {  
    #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
    if (debug_mode)
    {
      Serial.print(F("service_remote_communications_incoming_serial_buffer: serial1_buffer_index: "));
      Serial.print(serial1_buffer_index);
      Serial.print(F(" buffer: "));
      for (int x = 0;x < serial1_buffer_index;x++){
        Serial.write(serial1_buffer[x]);
      }
      Serial.println("$");
    }
    #endif //DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
    
    if (remote_unit_command_submitted)  
    {  // this was a solicited response
      switch(remote_unit_command_submitted)
      {
        case REMOTE_UNIT_AZ_COMMAND:
          if ((serial1_buffer_index == 7) && (serial1_buffer[0] == 'A') && (serial1_buffer[1] == 'Z') && 
          (is_ascii_number(serial1_buffer[2])) && (is_ascii_number(serial1_buffer[3])) && (is_ascii_number(serial1_buffer[4])) && (is_ascii_number(serial1_buffer[5])))
          {
            remote_unit_command_result_float = ((serial1_buffer[2]-48)*100) + ((serial1_buffer[3]-48)*10) + (serial1_buffer[4]-48) + ((serial1_buffer[5]-48)/10.0);           
            good_data = 1;
          }    
        break;
        case REMOTE_UNIT_EL_COMMAND:
          if ((serial1_buffer_index == 8) && (serial1_buffer[0] == 'E') && (serial1_buffer[1] == 'L') && 
          (is_ascii_number(serial1_buffer[3])) && (is_ascii_number(serial1_buffer[4])) && (is_ascii_number(serial1_buffer[5])) && (is_ascii_number(serial1_buffer[6])))
          {
            remote_unit_command_result_float = ((serial1_buffer[3]-48)*100) + ((serial1_buffer[4]-48)*10) + (serial1_buffer[5]-48) + ((serial1_buffer[6]-48)/10.0);           
            if (serial1_buffer[2] == '+') 
            {
              good_data = 1;
            }
            if (serial1_buffer[2] == '-') 
            {
              remote_unit_command_result_float = remote_unit_command_result_float * -1.0;
              good_data = 1;
            }             
          }         
        break;    
      }    
      if (good_data) 
      {
        remote_unit_command_results_available = remote_unit_command_submitted;
        remote_unit_good_results++;
        
        #ifdef DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER
        if (debug_mode)
        {
          Serial.print(F("service_remote_communications_incoming_serial_buffer: remote_unit_command_results_available: "));
          Serial.print(remote_unit_command_results_available);
          Serial.print(F(" remote_unit_command_result_float: "));
          Serial.println(remote_unit_command_result_float);
        }
        #endif //DEBUG_SVC_REMOTE_COMM_INCOMING_BUFFER        
        
        
      } else 
      {
        remote_unit_command_results_available = 0;
        remote_unit_bad_results++;
      }
      remote_unit_command_submitted = 0;           
    } else 
    {  // this was an unsolicited message
      
    }
    serial1_buffer_carriage_return_flag = 0;
    serial1_buffer_index = 0;
  }
 
  // has a command timed out?
  if ((remote_unit_command_submitted) && ((millis() - last_remote_unit_command_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS))
  {
    remote_unit_command_timeouts++; 
    remote_unit_command_submitted = 0;
    serial1_buffer_index = 0;
  }
  
  // have characters been in the buffer for some time but no carriage return?
  if ((serial1_buffer_index) && (!remote_unit_command_submitted) && ((millis() - serial1_last_receive_time) > REMOTE_UNIT_COMMAND_TIMEOUT_MS)) 
  {
    serial1_buffer_index = 0;
    remote_unit_incoming_buffer_timeouts++;
  }
} // service_remote_communications_incoming_serial_buffer()
#endif //FEATURE_HOST_REMOTE_PROTOCOL

//--------------------------------------------------------------------------
#ifdef FEATURE_AZIMUTH_CORRECTION
float correct_azimuth(float azimuth_in)
{
  if (sizeof(azimuth_calibration_from) != sizeof(azimuth_calibration_to))
  {
    return azimuth_in;
  }
  for (unsigned int x = 0;x < (sizeof(azimuth_calibration_from)-2);x++)
  {
    if ((azimuth_in >= azimuth_calibration_from[x]) && (azimuth_in <= azimuth_calibration_from[x+1]))
    {
      return (map(azimuth_in*10,azimuth_calibration_from[x]*10,azimuth_calibration_from[x+1]*10,azimuth_calibration_to[x]*10,azimuth_calibration_to[x+1]*10))/10.0;
    }
  }
  return(azimuth_in);
} // correct_azimuth(float azimuth_in)
#endif //FEATURE_AZIMUTH_CORRECTION

//-------------------------------------------------------------------------- 
#ifdef FEATURE_ROTATION_INDICATOR_PIN
void service_rotation_indicator_pin()
{
  static byte rotation_indication_pin_state = 0;
  static unsigned long time_rotation_went_inactive = 0;
  
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((!rotation_indication_pin_state) && ((az_state != IDLE) || (el_state != IDLE)))
  {
  #else
  if ((!rotation_indication_pin_state) && ((az_state != IDLE)))
  {  
  #endif
    if (rotation_indication_pin)
    {
      digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_ACTIVE_STATE);
    }
    rotation_indication_pin_state = 1; 
    #ifdef DEBUG_ROTATION_INDICATION_PIN
    if (debug_mode){Serial.println(F("service_rotation_indicator_pin: active"));}
    #endif
  }
  
  #ifdef FEATURE_ELEVATION_CONTROL
  if ((rotation_indication_pin_state) && (az_state == IDLE) && (el_state == IDLE)){
  #else
  if ((rotation_indication_pin_state) && (az_state == IDLE))
  {  
  #endif
    if (time_rotation_went_inactive == 0)
    {
      time_rotation_went_inactive = millis();
    } else 
    {
      if ((millis() - time_rotation_went_inactive) >= ((ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS * 1000)+(ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES * 60 * 1000)))
      {
        if (rotation_indication_pin)
        {
          digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_INACTIVE_STATE);
        }
        rotation_indication_pin_state = 0;   
        time_rotation_went_inactive = 0; 
        #ifdef DEBUG_ROTATION_INDICATION_PIN
        if (debug_mode){Serial.println(F("service_rotation_indicator_pin: inactive"));}
        #endif    
      }
    }
  } 
} // service_rotation_indicator_pin()
#endif //FEATURE_ROTATION_INDICATOR_PIN  