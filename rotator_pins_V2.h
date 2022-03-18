// -------------------------------------   Pin Definitions ------------------------------------------

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

// You need to look at these and set them appropriately !
// Most pins can be disabled by setting them to 0 (zero).  If you're not using a pin or function, set it to 0.

// Upper row of pins, Shield board with Teensy on left, USB down
//----------|--|--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||
// Pins     |43|44|95|GN|28|25|24|23|15|17||16|15| 5| 1| 7| 6| 3| 2||64|63|13|12|46|45|44|43||
// Digital  |21|20|  |  |13|12|11|10| 9| 8|| 7| 6| 5| 4| 3| 2| 1| 0||14|15|16|17|18|19|20|21||
// PWM      |  |  |  |  |13|12|11|10| 0| 8|| 7| 6| 5| 4| 3| 2| 1| 0||14|15|16|17|18|19|20|21||
// PCINT    |  |  |  |  | 7| 6| 5| 4|  |  ||  | 6|  |  |  |  | 3| 8||10| 9|  |  |  |  |  |  ||
//--------- |--|--|--|--|--|--|--|--|- |- ||--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||
// LED      |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |L3|L2||L1|L0|  |  |  |  |  |  ||
// Relays   |  |  |  |  |  |OU|IN|An|  |  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// Key      |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// Bias     |  |  |  |  |  |  |  |  |  |BI||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// Flow     |  |  |  |  |  |  |  |  |FL|  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// Pots     |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// Buttons  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
// ALC      |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||
//--------- |--|--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||
// Unused   |43|44|95|GN|13|  |  |  |  |  ||16|15| 5| 1| 7| 6|  |  ||  |  |13|12|46|45|44|43||
//--------- |--|--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||

// 
// Lower row of pins, Shield board with Teensy on left, USB down
//----------|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|---|---|---|---|---|---||
// Pins     |NC|RF|30|3V|5V|GN|GN|VI||97|96|95|94|93|92|91|90||89|88| 87| 86| 86| 84| 83| 82||
// Digital  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Analog   |  |  |  |  |  |  |  |  ||A0|A1|A2|A3|A4|A5|A6|A7||A8|A9|A10|A11|A12|A13|A14|A15||
// PCINT    |  |  |  |  |  |  |  |  ||16|17|18|19|20|21|22|23||  |  |   |   |   |   |   |   ||
//--------- |--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|---|---|---|---|---|---||
// Temp     |  |  |  |  |  |  |  |  ||  |  |T6|T5|T4|T3|T2|T1||  |  |   |   |   |   |   |   ||
// LED      |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Power    |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||IN|RV| FW|   |   |   |   |   ||
// LED      |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Relays   |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Key      |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Bias     |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Pots     |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Buttons  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// ALC      |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Vd       |  |  |  |  |  |  |  |  ||  |VD|  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
// Id       |  |  |  |  |  |  |  |  ||ID|  |  |  |  |  |  |  ||  |  |   |   |   |   |   |   ||
//--------- |--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|---|---|---|---|---|---||
// Unused   |NC|RF|30|3V|5V|GN|GN|VI||  |  |  |  |  |  |  |  ||  |  |   | 86| 86| 84| 83| 82||
//--------- |--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--||--|--|---|---|---|---|---|---||

// 
// Pins along right edge, Shield board with Teensy on left, USB down
//                  Interior Row                                  ||               Exterior Row                          ||
//----------|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||
// Pins     |GN|20|22|36|38|40|42|52|50|54|56|55|60|72|74|76|78|5V||GN|19|21|35|37|39|41|51|70|53|55|57|59|71|73|75|77|5V||
// Digital  |  |52|50|48|46|44|42|40|38|36|34|32|30|28|26|24|22|  ||  |53|51|49|47|45|43|41|39|37|35|33|31|29|27|25|23|  ||
// Analog   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// PWM      |  |52|  |  |46|44|  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |45|  |  |  |  |  |  |  |  |  |  |  |  ||
// PCINT    |  | 1| 3|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  | 0| 2|  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
//--------- |--|--|--|--|--|--|--|--|- |--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||
// LCD      |  |  |  |D7|D6|D5|D4|E+|RS|  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// LED      |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// Relays   |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// Key      |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// Bias     |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// Pots     |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// Buttons  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
// ALC      |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  ||
//--------- |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|---|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||
// Unused   |  |20|22|  |  |  |  |  |  |54|56|55|60|72|74|76|78|  ||  |  |  |35|37|  |41|51|70|53|55|57|59|71|73|75|77|  ||
//--------- |--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--|--||

#define RF_IN_PIN
#define RF_FWD_PIN
#define RF_REV_PIN

#define VD_PIN
#define ID_PIN

#define TEMP_1_PIN
#define TEMP_2_PIN
#define TEMP_3_PIN
#define TEMP_4_PIN
#define TEMP_5_PIN
#define TEMP_6_PIN
#define FLOW_PIN

#define BIAS_PIN

#define RELAY_IN_PIN
#define RELAY_OUT_PIN
#define RELAY_ANT_PIN


#define SerialRxPin    0
#define SerialTxPin    1

#define    LCDRsPin   16
#define    LCDEnPin   17
#define    LCDD7Pin   24
#define    LCDD6Pin   26
#define    LCDD5Pin   28
#define    LCDD4Pin   30
#define   LCDDimPin    4

#define    KeyInPin   31

// when using a quad relay board
//#define   Relay1Pin    0 // ground to assert relay
//#define   Relay2Pin    0 // ground to assert relay
//#define   Relay3Pin    0 // ground to assert relay
//#define   Relay4Pin    0 // ground to assert relay

// when using an L298N H-bridge PWM motor controller
//#define   ENAPin       3 // ENA on L298n
//#define   IN1Pin       4 // IN1 on L298n
//#define   IN2Pin       5 // IN2 on L298n

//#define   LEDYelPin   21
//#define   LEDGrnPin   20
//#define   LEDRedPin   32

//#define   PotTopPin  A11
//#define   PotBotPin  A10
//#define   ButtonPin  A12
//#define    ALCInPin  A13
//#define   RotatorAz  A14

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
