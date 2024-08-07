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
 
 * settings.h
 *
 *  Created on: Feb 12, 2017
 *      Author: jeff
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

/* --------------------------- Settings ------------------------------------------------
You can tweak these, but read the online documentation!
*/

// analog voltage calibration - these are default values;
// you can either tweak these or set via the Yaesu O and F commands (and O2 and F2)....
#ifdef FEATURE_HCO_ADC
#define RAZ_FULL_CCW               0  // Ohms, uncalibrated Rtop, pot reading
#define RAZ_FULL_CW              500  // Ohms, uncalabrated Rtop, pot reading
#else
#define RAZ_FULL_CCW               0  
#define RAZ_FULL_CW              360  // max expected on 0 to 1023 scale
#endif

#define ANALOG_EL_0_DEGREES              2
#define ANALOG_EL_MAX_ELEVATION        180  // maximum elevation is normally 180 degrees unless change below for ELEVATION_MAXIMUM_DEGREES

#define RAZ_OVERLAP_DEGREES      360  // if overlap_led is enabled, turn on overlap led line if azimuth is greater than this setting
                                      // you must use raw azimuth (if the azimuth on the rotator crosses over to 0 degrees, add 360
                                      // for example, on a Yaesu 450 degree rotator with a starting point of 180 degrees, and an overlap LED
                                      // turning on when going CW and crossing 180, RAZ_OVERLAP_DEGREES should be set for 540 (180 + 360)

/* -------------------------- rotation settings ---------------------------------------*/

#define AZIMUTH_STARTING_POINT_DEFAULT        0 // the starting point in degrees of the azimuthal rotator
                                                // (the Yaesu GS-232B Emulation Z command will override this and write the setting to eeprom)
#define AZIMUTH_ROTATION_CAPABILITY_DEFAULT 360 // the default rotation capability of the rotator in degrees
                                                // (the Yaesu P36 and P45 commands will override this and write the setting to eeprom)
#define ELEVATION_MAXIMUM_DEGREES           180 // change this to set the maximum elevation in degrees

// PWM speed voltage settings
#define PWM_SPEED_VOLTAGE_X1            64
#define PWM_SPEED_VOLTAGE_X2           128
#define PWM_SPEED_VOLTAGE_X3           191
#define PWM_SPEED_VOLTAGE_X4           255

//AZ
#define AZ_SLOWSTART_DEFAULT             0    // 0 = off ; 1 = on
#define AZ_SLOWDOWN_DEFAULT              0    // 0 = off ; 1 = on
#define AZ_SLOW_START_UP_TIME         2000    // if slow start is enabled, the unit will ramp up speed for this many milliseconds
#define AZ_SLOW_START_STARTING_PWM       1    // PWM starting value for slow start
#define AZ_SLOW_START_STEPS             20

#define SLOW_DOWN_BEFORE_TARGET_AZ      10.0  // if slow down is enabled, slowdown will be activated within this many degrees of target azimuth
#define AZ_SLOW_DOWN_PWM_START         200    // starting PWM value for slow down
#define	AZ_SLOW_DOWN_PWM_STOP           20    // ending PWM value for slow down
#define AZ_SLOW_DOWN_STEPS              20

//EL
#define EL_SLOWSTART_DEFAULT             0    // 0 = off ; 1 = on
#define EL_SLOWDOWN_DEFAULT              0    // 0 = off ; 1 = on
#define EL_SLOW_START_UP_TIME         2000    // if slow start is enabled, the unit will ramp up speed for this many milliseconds
#define EL_SLOW_START_STARTING_PWM       1    // PWM starting value for slow start
#define EL_SLOW_START_STEPS             20

#define SLOW_DOWN_BEFORE_TARGET_EL      10.0  // if slow down is enabled, slowdown will be activated within this many degrees of target azimuth
#define EL_SLOW_DOWN_PWM_START         200    // starting PWM value for slow down
#define	EL_SLOW_DOWN_PWM_STOP           20    // ending PWM value for slow down
#define EL_SLOW_DOWN_STEPS              20

#define TIMED_SLOW_DOWN_TIME          2000

//Variable frequency output settings
#define AZ_VARIABLE_FREQ_OUTPUT_LOW      1    // Frequency in hertz of minimum speed
#define AZ_VARIABLE_FREQ_OUTPUT_HIGH    60    // Frequency in hertz of maximum speed
#define EL_VARIABLE_FREQ_OUTPUT_LOW      1    // Frequency in hertz of minimum speed
#define EL_VARIABLE_FREQ_OUTPUT_HIGH    60    // Frequency in hertz of maximum speed

// Settings for AZ ROTATE_LIMITS
#define AZ_MANUAL_ROTATE_CCW_LIMIT       0    // if using a rotator that starts at 180 degrees, set this to something like 185
#define AZ_MANUAL_ROTATE_CW_LIMIT      360    // add 360 to this if you go past 0 degrees (i.e. 180 CW after 0 degrees = 540)

// Settings for OPTION_EL_MANUAL_ROTATE_LIMITS
#define EL_MANUAL_ROTATE_DOWN_LIMIT     -1
#define EL_MANUAL_ROTATE_UP_LIMIT      181

// Speed pot settings
#define SPEED_POT_LOW                    0
#define SPEED_POT_HIGH                1023
#define SPEED_POT_LOW_MAP                1
#define SPEED_POT_HIGH_MAP             255

// Azimuth preset pot settings
#define AZ_PRESET_CHECK_INTERVAL       250     // 250 ms
#define AZ_PRESET_RATE                   3     // turning defined as 3 adc counts per 250 msec
#define AZ_PRESET_STOP_COUNT             6
#define AZ_PRESET_POT_FULL_CCW           0
#define AZ_PRESET_POT_FULL_CW         1023
#define AZ_PRESET_POT_FULL_CCW_MAP       0     // azimuth pot fully clockwise degrees
#define AZ_PRESET_POT_FULL_CW_MAP      360     // azimuth pot fully counter-clockwise degrees

#define ENCODER_PRESET_TIMEOUT        5000

// various code settings
#define AZIMUTH_TOLERANCE                3.0   // rotator will stop within X degrees when doing autorotation
#define ELEVATION_TOLERANCE              1.0
#define OPERATION_TIMEOUT            60000     // timeout for any rotation operation in mS ; 60 seconds is usually enough unless you have the speed turned down
#define TIMED_INTERVAL_ARRAY_SIZE       20
#define SERIAL_BAUD_RATE            115200
#define SERIAL1_BAUD_RATE             9600     // 9600
#define SERIAL2_BAUD_RATE             9600     // 9600
#define SERIAL3_BAUD_RATE             9600     // 9600
#define DISPLAY_UPDATE_INTERVAL        200     // msec, display update interval, LCD or 7 segment
#define AZ_BRAKE_DELAY                1000     // msec
#define EL_BRAKE_DELAY                1000     // msec
#define TIME_BETWEEN_AZ_ADC_READ         5     // msec, 200 Hz
#define BUTTON_BOUNCE_DELAY            100     // msec
#define BUTTON_LONG_PRESS             2000     // msec, 2 seconds

#define EEPROM_MAGIC_NUMBER 100
#define EEPROM_WRITE_DIRTY_CONFIG_TIME  30  //time in seconds

#ifdef FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#define HEADING_MULTIPLIER              10
#define LCD_HEADING_MULTIPLIER          10.0
#define LCD_DECIMAL_PLACES               1
#else //FEATURE_ONE_DECIMAL_PLACE_HEADINGS
#define HEADING_MULTIPLIER               1
#define DECIMAL_PLACES                   0
#endif //FEATURE_ONE_DECIMAL_PLACE_HEADINGS

#define AZ_POSITION_ROTARY_ENCODER_DEG_PER_PULSE 0.5
#define EL_POSITION_ROTARY_ENCODER_DEG_PER_PULSE 0.5

#define AZ_POSITION_PULSE_DEG_PER_PULSE    0.5
#define EL_POSITION_PULSE_DEG_PER_PULSE    0.5

#define PARK_AZIMUTH                       180
#define PARK_ELEVATION                     0.0

#define COMMAND_BUFFER_SIZE               50

#define REMOTE_BUFFER_TIMEOUT_MS         250
#define REMOTE_UNIT_COMMAND_TIMEOUT_MS  2000
#define AZ_REMOTE_UNIT_QUERY_TIME_MS     150  // how often we query the remote remote for azimuth
#define EL_REMOTE_UNIT_QUERY_TIME_MS     150  // how often we query the remote remote for elevation

// Define polarity, depends on rotator interface
//   Values for wa1hco "do everything" box
//     Arduino digital pin, optoisolator pull down to light, optoisolator pulls down relay coil
#define ROTATE_PIN_INACTIVE_VALUE        LOW
#define ROTATE_PIN_ACTIVE_VALUE          HIGH
#define BRAKE_RELEASE_OFF                LOW  // means braked, used inside *break_release*() control functions
#define BRAKE_RELEASE_ON                 HIGH // means brake released, used inside *break_release*() control functions

// not used if HCO ADC
#define AZIMUTH_SMOOTHING_FACTOR         0 // value = 0 to 99.9
#define ELEVATION_SMOOTHING_FACTOR       ((float)0.00) // value = 0 to 99.9

#define AZIMUTH_MEASUREMENT_FREQUENCY_MS   0  // this does not apply if using FEATURE_AZ_POSITION_GET_FROM_REMOTE_UNIT
#define ELEVATION_MEASUREMENT_FREQUENCY_MS 0  // this does not apply if using FEATURE_EL_POSITION_GET_FROM_REMOTE_UNIT

#define JOYSTICK_WAIT_TIME_MS            100

#define ROTATION_INDICATOR_PIN_ACTIVE_STATE HIGH
#define ROTATION_INDICATOR_PIN_INACTIVE_STATE LOW
#define ROTATION_INDICATOR_PIN_TIME_DELAY_SECONDS 0
#define ROTATION_INDICATOR_PIN_TIME_DELAY_MINUTES 0

#endif /* SETTINGS_H_ */
