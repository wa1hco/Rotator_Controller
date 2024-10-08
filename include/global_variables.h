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
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "dependencies.h"

// -------------------- global variables definitions-----------------------------
// azimuth globals
extern float         Raz;            // az reading from sensor, prior to mapping
extern float         azimuth;        // az after wrapping to 0-360, used for display and control
extern float         raw_azimuth;    // working variable, may be unwrapped
extern int           target_azimuth;
extern int           target_raw_azimuth;
extern char          az_request;
extern int           az_request_parm;
extern char          az_request_queue_state;
extern char          az_state;
extern unsigned long az_last_rotate_initiation;
extern bool          isAzButtonPressed; 
extern char          brake_az_engaged;
extern unsigned long az_slowstart_start_time;
extern          char az_slow_start_step;
extern unsigned long az_last_step_time;
extern          char az_slow_down_step;
extern unsigned long az_timed_slow_down_start_time;
extern char          normal_az_speed_voltage;
extern char          current_az_speed_voltage;
extern char          az_slowstart_active;
extern char          az_slowdown_active;

// needed when FEATURE_HCO_BUTTONS
extern int           button_cw_press_time;
extern int           button_ccw_press_time;
extern bool          is_CW_cal_mode;
extern bool          is_CCW_cal_mode;


// needed when FEATURE_HCO_ADC
extern int           IsrTime;

extern char          debug_mode;
extern unsigned long last_debug_output_time;

struct config_t
{
  char magic_number;
  int Raz_full_ccw;
  int Raz_full_cw;
  int analog_el_0_degrees;
  int analog_el_max_elevation;
  int azimuth_starting_point;
  int azimuth_rotation_capability;
  float last_azimuth;
  float last_elevation;
};
extern struct config_t configuration;
extern char configuration_dirty;

extern char serial0_buffer[COMMAND_BUFFER_SIZE];
extern int  serial0_buffer_index;
extern char incoming_serial_byte;
extern unsigned long last_serial_receive_time;

extern char backslash_command;

#ifdef FEATURE_TIMED_BUFFER
extern int timed_buffer_azimuths[TIMED_INTERVAL_ARRAY_SIZE];
extern int timed_buffer_number_entries_loaded;
extern int timed_buffer_entry_pointer;
extern int timed_buffer_interval_value_seconds;
extern unsigned long last_timed_buffer_action_time;
extern char timed_buffer_status;
#endif //FEATURE_TIMED_BUFFER

extern char push_lcd_update;

#ifdef FEATURE_ELEVATION_CONTROL
extern int elevation;
extern int target_elevation;
extern char brake_el_engaged;
extern char el_request;
extern int el_request_parm;
extern char el_request_queue_state;
extern char el_slowstart_active;
extern char el_slowdown_active;
extern unsigned long el_slowstart_start_time;
extern char el_slow_start_step;
extern unsigned long el_last_step_time;
extern char el_slow_down_step;
extern unsigned long el_timed_slow_down_start_time;
extern char normal_el_speed_voltage;
extern char current_el_speed_voltage;

extern int display_elevation;   // Variable added to handle elevation beyond 90 degrees.  /// W3SA
extern char el_state;
extern int analog_el;
unsigned long el_last_rotate_initiation = 0;

#ifdef FEATURE_TIMED_BUFFER
int timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
#endif //FEATURE_TIMED_BUFFER

char elevation_button_was_pushed = 0;
#endif //FEATURE_ELEVATION_CONTROL

//#ifdef FEATURE_ROTARY_ENCODER_SUPPORT

#ifdef OPTION_ENCODER_HALF_STEP_MODE      // Use the half-step state table (emits a code at 00 and 11)
extern const unsigned char ttable[6][4];
#else                                     // Use the full-step state table (emits a code at 00 only)
extern const unsigned char ttable[7][4];
#endif //OPTION_ENCODER_HALF_STEP_MODE

#ifdef FEATURE_AZ_PRESET_POT
extern bool        is_display_preset;
extern int         azimuth_preset;
#endif

#ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder State Tables
extern int az_encoder_raw_degrees;
#define ENCODER_IDLE 0
#define ENCODER_AZ_PENDING 1
#define ENCODER_EL_PENDING 2
#define ENCODER_AZ_EL_PENDING 3
extern volatile unsigned char az_encoder_state;

#ifdef FEATURE_EL_PRESET_ENCODER
extern volatile unsigned char el_encoder_state;
extern int el_encoder_degrees = 0;
#endif //FEATURE_EL_PRESET_ENCODER

extern char preset_encoders_state;
#endif //FEATURE_AZ_PRESET_ENCODER

//#endif //FEATURE_ROTARY_ENCODER_SUPPORT

#ifdef DEBUG_PROFILE_LOOP_TIME
extern float average_loop_time;
#endif //DEBUG_PROFILE_LOOP_TIME

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
extern volatile float az_position_pulse_input_azimuth = 0;
extern volatile char last_known_az_state = 0;
#endif //FEATURE_AZ_POSITION_PULSE_INPUT

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
extern volatile float el_position_pulse_input_elevation = 0;
extern volatile char last_known_el_state = 0;
#endif //FEATURE_EL_POSITION_PULSE_INPUT

#ifdef FEATURE_REMOTE_UNIT_SLAVE
extern char serial_read_event_flag[] = {0,0,0,0,0};
extern char serial0_buffer_carriage_return_flag = 0;
#endif

#ifdef FEATURE_HOST_REMOTE_PROTOCOL
extern char serial1_buffer[COMMAND_BUFFER_SIZE];
extern int serial1_buffer_index = 0;
extern char serial1_buffer_carriage_return_flag = 0;
extern unsigned long serial1_last_receive_time = 0;
extern char remote_unit_command_submitted = 0;
extern unsigned long last_remote_unit_command_time = 0;
extern unsigned int remote_unit_command_timeouts = 0;
extern unsigned int remote_unit_bad_results = 0;
extern unsigned int remote_unit_good_results = 0;
extern unsigned int remote_unit_incoming_buffer_timeouts = 0;
extern char remote_unit_command_results_available = 0;
extern float remote_unit_command_result_float = 0;
extern char remote_port_rx_sniff = 0;
extern char remote_port_tx_sniff = 0;
extern char suspend_remote_commands = 0;
#endif

#ifdef DEBUG_POSITION_PULSE_INPUT
//extern unsigned int az_position_pule_interrupt_handler_flag = 0;
//extern unsigned int el_position_pule_interrupt_handler_flag = 0;
extern volatile unsigned long az_pulse_counter = 0;
extern volatile unsigned long el_pulse_counter = 0;
extern volatile unsigned int az_pulse_counter_ambiguous = 0;
extern volatile unsigned int el_pulse_counter_ambiguous = 0;
#endif //DEBUG_POSITION_PULSE_INPUT

/*
  Azimuth and Elevation calibration tables - use with FEATURE_AZIMUTH_CORRECTION and/or FEATURE_ELEVATION_CORRECTION
  You must have the same number of entries in the _from and _to arrays!
*/
#ifdef FEATURE_AZIMUTH_CORRECTION
externfloat azimuth_calibration_from[]    = {180, 630};    /* these are in "raw" degrees, i.e. when going east past 360 degrees, add 360 degrees*/
extern float azimuth_calibration_to[]      = {180, 630};
#endif //FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
extern float elevation_calibration_from[]  = {-180, 0, 180};
extern float elevation_calibration_to[]    = { 180, 0, 180};
#endif //FEATURE_ELEVATION_CORRECTION

extern char incoming_serial_char;
extern char serial0_buffer[COMMAND_BUFFER_SIZE];
extern int serial0_buffer_index;
extern unsigned long last_serial_receive_time;
extern char backslash_command;
extern char configuration_dirty;

//----------------------------------------Public functions--------------------------------------
void initialize_global_variables();

#endif // GLOBAL_VARIABLES_H_