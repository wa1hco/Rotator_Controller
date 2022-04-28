// -------------------------------------   Pin Definitions ------------------------------------------

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

// You need to look at these and set them appropriately !
// Most pins can be disabled by setting them to 0 (zero).  If you're not using a pin or function, set it to 0.

// Teensy v4.0 Digital and Analog Pins, front side only

//    Teensy 4.0 Pin Definitions, USB up, upperleft, counter clockwise
//----------|-|-|-|-|-|-|-|-|---|--|--|--|--|--|--|--|---|---|---|---|--|--|--|--|
// Pins     |0|1|2|3|4|5|6|7|8  |9 |10|11|12|13|14|15|16 |17 |18 |19 |20|21|22|23|
// Digital  |0|1|2|3|4|5|6|7|8  |9 |10|11|12|13|14|15|16 |17 |18 |19 |20|21|22|23|
// Analog   | | | | | | | | |   |  |  |  |  |  |A0|A1|A2 |A3 |A4 |A5 |A6|A7|A8|A9|
// PWM      |0|1|2|3|4|5|6|7|8  |9 |10|11|12|13|14|15|   |   |18 |19 |  |  |22|23|
// I2C      | | | | | | | | |   |  |  |  |  |  |  |  |CL1|DA1|DA0|CL0|  |  |  |  |
// Serial   |R|T| | | | | |R|T  |  |  |  |  |  |T3|R3|R4 |T4 |   |   |T5|R5|  |  |
// CAN      |R|T| | | | | | |   |  |  |T |  |R |  |  |   |   |   |   |  |  |T |R |
//--------- |-|-|-|-|-|-|-|-|---|--|--|--|--|--|--|--|---|---|---|---|--|--|--|--|

// HCO Rotator custom board use of pins
// Pins     |0|1|2|3|4|5|6|7|8  |9 |10|11|12|13|14|15|16 |17 |18 |19 |20|21|22|23|
//--------- |-|-|-|-|-|-|-|-|---|--|--|--|--|--|--|--|---|---|---|-- |--|--|--|--|
// Analog   | | | | | | | | |   |  |  |  |  |  |A0|A1|   |   |   |   |  |  |  |  |
// I2C      | | | | | | | | |   |  |  |  |  |  |  |  |   |   |DA0|CL0|  |  |  |  |
// Buttons  | | | | | | | | |CCW|CW|  |  |  |  |  |  |   |   |   |   |  |  |  |  |
// SPI      | | | | | | | | |   |  |CS|SI|SO|CL|  |  |   |   |   |   |  |  |  |  |
// Relays   | | | | | | | | |   |  |  |  |  |  |  |  |BR |LR |   |   |MV|  |  |  |
//--------- |-|-|-|-|-|-|-|-|---|--|--|--|--|--|--|--|---|---|---|---|--|--|--|--|
// Unused   |0|1|2|3|4|5|6|7|   |  |  |  |  |  |  |  |   |   |   |   |  |21|22|23|
//--------- |-|-|-|-|-|-|-|-|---|--|--|--|--|--|--|--|---|---|---|-- |--|--|--|--|

#ifdef FEATURE_MAX6959_DISPLAY
#define I2CClockPin    19 // SCL0, I2c, when using MAX6959
#define I2CDataPin     18 // SDA0, I2c, when using MAX6959
#endif

#ifdef FEATURE_MAX7221_DISPLAY
#define MAX7221_CS_PIN 10 // MAX7221 SPI bus chip select pin
#endif

#ifdef FEATURE_HCO_BUTTONS
#define BUTTON_CCW     8
#define BUTTON_CW      9
#endif

// when using a quad relay board
#define BrakeAzPin     16         // high to release brake
#define DirectionPin   17         // high to rotate left
#define MotorPin       20         // high to run motor

#define PositionPosPin 14         // A0, grounded wiper
#define PositionNegPin 15         // A1, grounded wiper

#define TeensyLED      13

// when using motor and direction relays or...
// when using an L298N H-bridge PWM motor controller
#define ENAPin         20         // ENA on L298n, Motor control pin
#define IN1Pin         17         // IN1 on L298n, Direction controlpin
#define IN2Pin         0          // IN2 on L298n


// azimuth pins --------------------- (use just the azimuth pins for an azimuth-only rotator)
// if change made here, also change initialize_pins()
#define serial_led              TeensyLED       // LED blinks when command is received on serial port (set to 0 to disable)
#define overlap_led             0               // line goes high when azimuth rotator is in overlap (> 360 rotators)
#define brake_az                BrakeAzPin      // goes high to disengage azimuth brake (set to 0 to disable)
#define az_speed_pot            0               // connect to wiper of 1K to 10K potentiometer for speed control (set to 0 to disable)
#define az_preset_pot           0               // connect to wiper of 1K to 10K potentiometer for preset control (set to 0 to disable)
#define preset_start_button     0               // connect to momentary switch (ground on button press) for preset start (set to 0 to disable or for preset automatic start)
#define button_stop             0               // connect to momentary switch (ground on button press) for preset stop (set to 0 to disable or for preset automatic start)

#define rotate_cw               0               // asserted to activate rotator R ( CW) rotation - pin 1 on Yaesu connector
#define rotate_ccw              0               // asserted to activate rotator L (CCW) rotation - pin 2 on Yaesu connector
#define rotate_cw_pwm           0               // optional - PWM CW output  - set to 0 to disable (must be PWM capable pin)
#define rotate_ccw_pwm          0               // optional - PWM CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_cw_ccw_pwm       0               // optional - PWM on CW and CCW output - set to 0 to disable (must be PWM capable pin)
#define rotate_motor            ENAPin          // optional - motor control pin
#define rotate_cw_freq          0               // optional - CW variable frequency output
#define rotate_ccw_freq         0               // optional - CCW variable frequency output

#define rotate_h1               IN1Pin          // L298 H Bridge In 1 Pin, High means rotate cw
#define rotate_h2               0               // L298 H Bridge In 2 Pin, High means rotate ccw
#define button_cw               BUTTON_CW       // normally open button to ground for manual CW rotation (schematic pin: A1)
#define button_ccw              BUTTON_CCW      // normally open button to ground for manual CCW rotation (schematic pin: A2)
#define azimuth_speed_voltage   0               // optional - PWM output for speed control voltage feed into rotator (on continually unlike rotate_cw_pwm and rotate_ccw_pwm)
#define blink_led               0

#define rotator_analog_az       PositionPosPin  // reads analog azimuth voltage from rotator - pin 4 on Yaesu connector
#define rotation_indication_pin 0


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
#define az_rotary_preset_pin1           0   // CW Encoder Pin
#define az_rotary_preset_pin2           0   // CCW Encoder Pin
#endif //FEATURE_AZ_PRESET_ENCODER

#ifdef FEATURE_EL_PRESET_ENCODER 
#define el_rotary_preset_pin1           0  // UP Encoder Pin
#define el_rotary_preset_pin2           0  // DOWN Encoder Pin
#endif //FEATURE_EL_PRESET_ENCODER

#ifdef FEATURE_AZ_POSITION_ROTARY_ENCODER
#define az_rotary_position_pin1         0  // CW Encoder Pin
#define az_rotary_position_pin2         0  // CCW Encoder Pin
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

#define    LCDRsPin   99
#define    LCDEnPin   99
#define    LCDD7Pin   99
#define    LCDD6Pin   99
#define    LCDD5Pin   99
#define    LCDD4Pin   99
#define   LCDDimPin   99

#define    KeyInPin   99

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


