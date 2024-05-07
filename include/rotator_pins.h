// -------------------------------------   Pin Definitions ------------------------------------------

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

// You need to look at these and set them appropriately !
// Most pins can be disabled by setting them to 0 (zero).  If you're not using a pin or function, set it to 0.

// Teensy v3.2 Digital and Analog Pins, with Teensey Shield
//    Shield board with Teensy on left, USB down
//----------|-|-|-|-|-|-|-|-|-|-|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|
// Pins     |0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|A10|A11|A12|A13|A14|
// Digital  |0|1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|                   |
// Analog   | | | | | | | | | | |  |  |  |  |14|15|16|17|18|19|20|21|22|23|  |  |26|27|28|29|30|31|  |  |A10|A11|A12|A13|A14|
// PWM      | | | |3|4|5|6| | |9|10|  |  |  |  |  |  |  |  |  |20|21|22|23|  |25|  |  |  |  |  |  |32|  |   |   |   |   |   |
//--------- |-|-|-|-|-|-|-|-|-|-|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|
// Audio    | | | | | | |6|7| |9|10|11|12|13|14|15|  |  |18|19|  |  |22|23|  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |
// Serial   |R|T| | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |
// LCD      | | | | | | | | | | |  |  |  |  |  |  |RS|EN|  |  |  |  |  |  |D7|  |D6|  |D5|  |D4|  |  |  |   |   |   |   |   |
// LED      | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  | G| Y|  |  |  |  |  |  |  |  |  |  | R|  |   |   |   |   |   |
// Relays   | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |
// L298N    | | | |E|1|2| | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |   |   |
// Key      | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |KY|  |  |   |   |   |   |   |
// Pots     | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |PT1|   |   |   |   |
// Rotator  | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |pot|   |   |   |
// Buttons  | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   |BTN|   |   |
// ALC      | | | | | | | | | | |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |   |   |   |ALC|   |
//--------- |-|-|-|-|-|-|-|-|-|-|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|
// Unused   | | |2| | | | | |8| |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |25|  |27|  |29|  |  |  |33|   |   |   |   |A14|
//--------- |-|-|-|-|-|-|-|-|-|-|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|---|---|---|---|

#define SerialRxPin    0
#define SerialTxPin    1

// when using a quad relay board
#define   Relay1Pin    0 // ground to assert relay
#define   Relay2Pin    0 // ground to assert relay
#define   Relay3Pin    0 // ground to assert relay
#define   Relay4Pin    0 // ground to assert relay

// when using an L298N H-bridge PWM motor controller
#define   ENAPin       3 // ENA on L298n
#define   IN1Pin       4 // IN1 on L298n
#define   IN2Pin       5 // IN2 on L298n

#define   LEDYelPin   21
#define   LEDGrnPin   20
#define   LEDRedPin   32

#define    LCDRsPin   16
#define    LCDEnPin   17
#define    LCDD7Pin   24
#define    LCDD6Pin   26
#define    LCDD5Pin   28
#define    LCDD4Pin   30
#define   LCDDimPin    4

#define    KeyInPin   31

#define   PotTopPin  A11
#define   PotBotPin  A10
#define   ButtonPin  A12
#define    ALCInPin  A13
#define   RotatorAz  A14

/* azimuth pins --------------------- (use just the azimuth pins for an azimuth-only rotator) */
#define rotate_cw               0           // asserted to activate rotator R ( CW) rotation - pin 1 on Yaesu connector
#define rotate_ccw              0           // asserted to activate rotator L (CCW) rotation - pin 2 on Yaesu connector
#define rotate_cw_pwm           0           // optional - PWM CW output - set to 0 to disable (must be PWM capable pin)
#define rotate_ccw_pwm          0           // optional - PWM CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_ccw_pwm       ENAPin      // optional - PWM on CW and CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_freq          0           // optional - CW variable frequency output
#define rotate_ccw_freq         0           // optional - CCW variable frequency output
#define rotate_h1               IN1Pin      // L298 H Bridge In 1 Pin, High means rotate cw
#define rotate_h2               IN2Pin      // L298 H Bridge In 2 Pin, High means rotate ccw
#define button_cw               0           // normally open button to ground for manual CW rotation (schematic pin: A1)
#define button_ccw              0           // normally open button to ground for manual CCW rotation (schematic pin: A2)
#define serial_led              0           // LED blinks when command is received on serial port (set to 0 to disable)
#define rotator_analog_az       PotTopPin   // reads analog azimuth voltage from rotator - pin 4 on Yaesu connector
#define azimuth_speed_voltage   0           // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_cw_pwm and rotate_ccw_pwm)
#define overlap_led             LEDYelPin   // line goes high when azimuth rotator is in overlap (> 360 rotators)
#define brake_az                0           // goes high to disengage azimuth brake (set to 0 to disable)
#define az_speed_pot            0           // connect to wiper of 1K to 10K potentiometer for speed control (set to 0 to disable)
#define az_preset_pot           PotBotPin   // connect to wiper of 1K to 10K potentiometer for preset control (set to 0 to disable)
#define preset_start_button     0           // connect to momentary switch (ground on button press) for preset start (set to 0 to disable or for preset automatic start)
#define button_stop             0           // connect to momentary switch (ground on button press) for preset stop (set to 0 to disable or for preset automatic start)
#define rotation_indication_pin 0
#define blink_led               1

/*----------- elevation pins --------------*/
#ifdef FEATURE_ELEVATION_CONTROL
#define rotate_up               0           // goes high to activate rotator elevation up
#define rotate_down             0           // goes high to activate rotator elevation down
#define rotate_up_or_down       0           // goes high when elevation up or down is activated
#define rotate_up_pwm           0           // optional - PWM UP output - set to 0 to disable (must be PWM capable pin)
#define rotate_down_pwm         0           // optional - PWM DOWN output - set to 0 to disable (must be PWM capable pin)
#define rotate_up_down_pwm      0           // optional - PWM on both UP and DOWN (must be PWM capable pin)
#define rotate_up_freq          0           // optional - UP variable frequency output
#define rotate_down_freq        0           // optional - UP variable frequency output
#define rotator_analog_el       0           // reads analog elevation voltage from rotator
#define button_up               0           // normally open button to ground for manual up elevation
#define button_down             0           // normally open button to ground for manual down rotation
#define brake_el                0           // goes high to disengage elevation brake (set to 0 to disable)
#define elevation_speed_voltage 0           // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_up_pwm and rotate_down_pwm)
#endif //FEATURE_ELEVATION_CONTROL

// rotary encoder pins and options
#ifdef FEATURE_AZ_PRESET_ENCODER 
#define az_rotary_preset_pin1   0           // CW Encoder Pin
#define az_rotary_preset_pin2   0           // CCW Encoder Pin
#endif //FEATURE_AZ_PRESET_ENCODER

#ifdef FEATURE_EL_PRESET_ENCODER 
#define el_rotary_preset_pin1   0           // UP Encoder Pin
#define el_rotary_preset_pin2   0           // DOWN Encoder Pin
#endif //FEATURE_EL_PRESET_ENCODER

#ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
#define az_rotary_position_pin1 0           // CW Encoder Pin
#define az_rotary_position_pin2 0           // CCW Encoder Pin
#endif //FEATURE_AZ_POSITION_ROTARY_ENCODER

#ifdef FEATURE_EL_POSITION_ROTARY_ENCODER
#define el_rotary_position_pin1         0   // CW Encoder Pin
#define el_rotary_position_pin2         0   // CCW Encoder Pin
#endif //FEATURE_EL_POSITION_ROTARY_ENCODER

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
#define az_position_pulse_pin           0   // must be an interrupt capable pin!
#define AZ_POSITION_PULSE_PIN_INTERRUPT 0   // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1
#endif                                                // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
#define el_position_pulse_pin           0   // must be an interrupt capable pin!
#define EL_POSITION_PULSE_PIN_INTERRUPT 0   // Uno: pin 2 = interrupt 0, pin 3 = interrupt 1
#endif                                      // read http://arduino.cc/en/Reference/AttachInterrupt for details on hardware and interrupts

#ifdef FEATURE_PARK
#define button_park 0
#endif

//classic 4 bit LCD pins
#define lcd_4_bit_rs_pin                LCDRsPin
#define lcd_4_bit_enable_pin            LCDEnPin
#define lcd_4_bit_d4_pin                LCDD4Pin 
#define lcd_4_bit_d5_pin                LCDD5Pin
#define lcd_4_bit_d6_pin                LCDD6Pin
#define lcd_4_bit_d7_pin                LCDD7Pin


#ifdef FEATURE_JOYSTICK_CONTROL
#define pin_joystick_x 0
#define pin_joystick_y 0
#endif //FEATURE_JOYSTICK_CONTROL

#endif // ifndef ROTATOR_PINS_H_


