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
*/

// Arduino environment
#include <Arduino.h>

#include "dependencies.h"

#include "Input.h"
#include <MsTimer2.h>

extern FIR<float, 31> fir_top;
extern FIR<float, 31> fir_bot;

// define functions in this file
void initialize_peripherals();
void initialize_interrupts();
void initialize_serial();
void initialize_pins();

// define external functions
void read_headings();
void TimedService();

void setup() 
{
  delay(500);
  initialize_serial();
  initialize_peripherals();
  read_settings_from_eeprom(); 
  initialize_pins();

  read_headings();
  #ifdef FEATURE_YAESU_EMULATION
  report_current_azimuth();      // Yaesu - report the azimuth right off the bat without a C command; the Arduino doesn't wake up quick enough
                                 // to get first C command from HRD and if HRD doesn't see anything it doesn't connect
  #endif //FEATURE_YAESU_EMULATION                                 

  #ifdef FEATURE_TIMED_BUFFER 
  timed_buffer_status = EMPTY;
  #endif //FEATURE_TIMED_BUFFER 
  
  #ifdef FEATURE_LCD_DISPLAY
  initialize_lcd_display();
  #endif

  #ifdef FEATURE_MAX7221_DISPLAY
  initialize_MAX7221_display();
  #endif

  initialize_rotary_encoders(); 
  initialize_interrupts();

  /*
  FIR filter designed with
  http://t-filter.appspot.com

  sampling frequency: 200 Hz

  * 0 Hz - 20 Hz
    gain = 1
    desired ripple = 3 dB
    actual ripple = 1.1761649821089057 dB

  * 50 Hz - 100 Hz
    gain = 0
    desired attenuation = -60 dB
    actual attenuation = -66.03868970041013 dB

  */
  #ifdef FEATURE_FIR_FILTER
  #define FILTER_TAP_NUM 15
  static float filter_taps[FILTER_TAP_NUM] = {
    -0.004942353838242621,
    -0.018993776250465885,
    -0.034906800521284226,
    -0.02689862294067124,
     0.03329097458407915,
     0.14355647714952716,
     0.25568358157357024,
     0.3034216074312329,
     0.25568358157357024,
     0.14355647714952716,
     0.03329097458407915,
    -0.02689862294067124,
    -0.034906800521284226,
    -0.018993776250465885,
    -0.004942353838242621
  };

  fir_top.setFilterCoeffs(filter_taps);
  fir_bot.setFilterCoeffs(filter_taps);
  #endif // FIR filter

  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TIME_BETWEEN_AZ_ADC_READ, TimedService); // interval, function call
  // interrupts enabled after this point
  MsTimer2::start(); 
} //setup()

//--------------------------------------------------------------
void initialize_peripherals()
{  
  #ifdef FEATURE_WIRE_SUPPORT 
  Wire.begin();
  #endif
  
  #ifdef FEATURE_AZ_POSITION_HMC5883L
  compass = HMC5883L();
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if (error != 0) 
  {
    Serial.print(F("setup: compass error:"));
    Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
  }
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) 
  {
    Serial.print(F("setup: compass error:"));
    Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
  }
  #endif //FEATURE_AZ_POSITION_HMC5883L
  
  #ifdef FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
  accel = ADXL345();
  accel.SetRange(2, true);
  accel.EnableMeasurements();
  #endif //FEATURE_EL_POSITION_ADXL345_USING_LOVE_ELECTRON_LIB
  
  #ifdef FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
  accel.begin();
  #endif //FEATURE_EL_POSITION_ADXL345_USING_ADAFRUIT_LIB
  
  #ifdef FEATURE_JOYSTICK_CONTROL
  pinMode(pin_joystick_x,INPUT);
  pinMode(pin_joystick_y,INPUT);  
  #endif //FEATURE_JOYSTICK_CONTROL
  
  #if defined(FEATURE_EL_POSITION_LSM303) || defined(FEATURE_AZ_POSITION_LSM303)
  if (!lsm.begin())
  {
    Serial.println(F("setup: LSM303 error"));
  }  
  #endif //FEATURE_EL_POSITION_LSM303 || FEATURE_AZ_POSITION_LSM303 
}

//--------------------------------------------------------------
void initialize_interrupts()
{ 
  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
  attachInterrupt(AZ_POSITION_PULSE_PIN_INTERRUPT, az_position_pulse_interrupt_handler, FALLING);
  #endif //FEATURE_AZ_POSITION_PULSE_INPUT
  
  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
  attachInterrupt(EL_POSITION_PULSE_PIN_INTERRUPT, el_position_pulse_interrupt_handler, FALLING);
  #endif //FEATURE_EL_POSITION_PULSE_INPUT 
}

//--------------------------------------------------------------
void initialize_pins()
{  
  if (button_ccw)
  {
    pinMode(button_ccw, INPUT_PULLUP);
  }
  if (button_cw)
  {
    pinMode(button_cw, INPUT_PULLUP);
  }
  
  if (serial_led) 
  {
    pinMode(serial_led, OUTPUT);
  } 
  if (overlap_led) 
  {
    pinMode(overlap_led, OUTPUT);
  }
  if (brake_az) 
  {
    // apparently, have to set output before setting output level
    // could lead to microsecond glitch on brake signal, not enough to pull relay
    pinMode(brake_az, OUTPUT);
    digitalWrite(brake_az, BRAKE_RELEASE_OFF);
  }

  #ifdef FEATURE_HCO_ADC
  pinMode(AzPositionTopPin, INPUT);
  pinMode(AzPositionBotPin, INPUT);
  #endif

  if (az_preset_pot) 
  {
    pinMode(az_preset_pot, INPUT);
  }

  if (az_speed_pot) 
  {
    pinMode(az_speed_pot, INPUT);
  }

  
  if (preset_start_button) 
  {
    pinMode(preset_start_button, INPUT);
    digitalWrite(preset_start_button, HIGH);
  }  

  if (button_stop) 
  {
    pinMode(button_stop, INPUT);
    digitalWrite(button_stop, HIGH);
  }   

  #ifdef MAX7221_CS_PIN
    pinMode(MAX7221_CS_PIN, OUTPUT);
    digitalWrite(MAX7221_CS_PIN, HIGH);
  #endif

  #ifdef FEATURE_ELEVATION_CONTROL
  if (brake_el) 
  {
    // apparently, have to set output before setting output level
    // could lead to microsecond glitch on brake signal, not enough to pull relay
    pinMode(brake_el, OUTPUT);
    digitalWrite(brake_el, BRAKE_RELEASE_OFF);
  }  
  #endif //FEATURE_ELEVATION_CONTROL

  // Rotation control pins, ensure pins
  
  if (rotate_cw)         {pinMode(rotate_cw,         OUTPUT);}
  if (rotate_ccw)        {pinMode(rotate_ccw,        OUTPUT);}
  if (rotate_cw_pwm)     {pinMode(rotate_cw_pwm,     OUTPUT);}
  if (rotate_ccw_pwm)    {pinMode(rotate_ccw_pwm,    OUTPUT);}
  if (rotate_cw_ccw_pwm) {pinMode(rotate_cw_ccw_pwm, OUTPUT);}  
  if (rotate_motor)      {pinMode(rotate_motor,      OUTPUT);}
  if (rotate_cw_freq)    {pinMode(rotate_cw_freq,    OUTPUT);}
  if (rotate_ccw_freq)   {pinMode(rotate_ccw_freq,   OUTPUT);}  
  if (rotate_h1)         {pinMode(rotate_h1,         OUTPUT);}
  if (rotate_h2)         {pinMode(rotate_h2,         OUTPUT);}

  rotator(DEACTIVATE,  CW);
  rotator(DEACTIVATE, CCW);

  #ifndef FEATURE_AZ_POSITION_HMC5883L
  pinMode(rotator_analog_az, INPUT);
  #endif
  
  normal_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  current_az_speed_voltage = PWM_SPEED_VOLTAGE_X4;

  // if azimuth_speed_voltage pin is configured, set it up for PWM output
  if (azimuth_speed_voltage) 
  { 
    analogWrite(azimuth_speed_voltage, PWM_SPEED_VOLTAGE_X4);
  }

  #ifdef FEATURE_ELEVATION_CONTROL
  pinMode(rotate_up, OUTPUT);
  pinMode(rotate_down, OUTPUT);
  if (rotate_up_or_down)  {pinMode(rotate_up_or_down,  OUTPUT);}
  if (rotate_up_pwm)      {pinMode(rotate_up_pwm,      OUTPUT);}
  if (rotate_down_pwm)    {pinMode(rotate_down_pwm,    OUTPUT);} 
  if (rotate_up_down_pwm) {pinMode(rotate_up_down_pwm, OUTPUT);}
  if (rotate_up_freq)     {pinMode(rotate_up_freq,     OUTPUT);}
  if (rotate_down_freq)   {pinMode(rotate_down_freq,   OUTPUT);}   
  rotator(DEACTIVATE,UP);
  rotator(DEACTIVATE,DOWN); 
  #ifdef FEATURE_EL_POSITION_POTENTIOMETER
  pinMode(rotator_analog_el, INPUT);
  #endif //FEATURE_EL_POSITION_POTENTIOMETER
  if (button_up) 
  {
    pinMode(button_up, INPUT);
    digitalWrite(button_up, HIGH);
  }
  if (button_down) 
  {
    pinMode(button_down, INPUT);
    digitalWrite(button_down, HIGH);
  }
  
  if (elevation_speed_voltage) 
  {                 // if elevation_speed_voltage pin is configured, set it up for PWM output
    analogWrite(elevation_speed_voltage, PWM_SPEED_VOLTAGE_X4);
    normal_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
    current_el_speed_voltage = PWM_SPEED_VOLTAGE_X4;
  }  
  
  read_elevation();
  #endif //FEATURE_ELEVATION_CONTROL
  
  #ifdef FEATURE_AZ_POSITION_PULSE_INPUT
  if (az_position_pulse_pin) 
  {
    pinMode(az_position_pulse_pin, INPUT);
    #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
    digitalWrite(az_position_pulse_pin, HIGH);
    #endif //OPTION_POSITION_PULSE_INPUT_PULLUPS    
  }
  #endif //FEATURE_AZ_POSITION_PULSE_INPUT
  
  
  #ifdef FEATURE_EL_POSITION_PULSE_INPUT
  if (el_position_pulse_pin) 
  {
    pinMode(el_position_pulse_pin, INPUT);
    #ifdef OPTION_POSITION_PULSE_INPUT_PULLUPS
    digitalWrite(el_position_pulse_pin, HIGH);
    #endif //OPTION_POSITION_PULSE_INPUT_PULLUPS
  }
  #endif //FEATURE_EL_POSITION_PULSE_INPUT  
  
  #ifdef FEATURE_PARK
  if (button_park)
  {
    pinMode(button_park, INPUT);
    digitalWrite(button_park, HIGH);
  }
  #endif //FEATURE_PARK
  
  #ifdef FEATURE_ROTATION_INDICATOR_PIN
  if (rotation_indication_pin)
  {
    pinMode(rotation_indication_pin, OUTPUT);
    digitalWrite(rotation_indication_pin,ROTATION_INDICATOR_PIN_INACTIVE_STATE);
  }
  #endif //FEATURE_ROTATION_INDICATOR_PIN
  
  if (blink_led) 
  {
    pinMode(blink_led,OUTPUT);
  }
}  

//--------------------------------------------------------------
void initialize_serial()
{  
  Serial.begin(SERIAL_BAUD_RATE);
 
  #ifdef FEATURE_REMOTE_UNIT_SLAVE
  Serial.print(F("CS"));
  Serial.println(CODE_VERSION);
  #endif //FEATURE_REMOTE_UNIT_SLAVE
  
  #ifdef OPTION_SERIAL1_SUPPORT
  Serial1.begin(SERIAL1_BAUD_RATE);
  #endif //OPTION_SERIAL1_SUPPORT
  
  #ifdef OPTION_SERIAL2_SUPPORT
  Serial1.begin(SERIAL2_BAUD_RATE);
  #endif //OPTION_SERIAL2_SUPPORT
  
  #ifdef OPTION_SERIAL2_SUPPORT
  Serial1.begin(SERIAL2_BAUD_RATE);
  #endif //OPTION_SERIAL2_SUPPORT   
}

