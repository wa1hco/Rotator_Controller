//
// check_serial.h
//
// Created on: Mar 17, 2021
//      Author: jeff, wa1hco
//
#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include <Arduino.h>
#include <avr/pgmspace.h>

#include "rotator_features.h"
#include "rotator_pins_HCO_board.h"
#include "settings.h"
#include "macros.h"

// -------------------- global variables definitions-----------------------------

// azimuth globals
extern int azimuth;
extern float AzFiltered;
extern int target_azimuth;
extern int target_raw_azimuth;
extern byte az_request;
extern int  az_request_parm;
extern byte az_request_queue_state;

extern byte az_state;
extern unsigned long az_last_rotate_initiation;
extern bool isAzButtonPressed;
extern byte MAX6959_buttons_debounced;
extern byte brake_az_engaged;
extern byte brake_el_engaged;

extern unsigned long az_slowstart_start_time;
extern          byte az_slow_start_step;
extern unsigned long az_last_step_time;
extern          byte az_slow_down_step;
extern unsigned long az_timed_slow_down_start_time;

extern byte normal_az_speed_voltage;
extern byte current_az_speed_voltage;
extern byte az_slowstart_active;
extern byte az_slowdown_active;

extern int IsrTime;

extern unsigned long last_debug_output_time;

struct config_t
{
  byte magic_number;
  int analog_az_full_ccw;
  int analog_az_full_cw;
  int analog_el_0_degrees;
  int analog_el_max_elevation;
  int azimuth_starting_point;
  int azimuth_rotation_capability;
  float last_azimuth;
  float last_elevation;
};

extern struct config_t configuration;


#ifdef FEATURE_TIMED_BUFFER
int timed_buffer_azimuths[TIMED_INTERVAL_ARRAY_SIZE];
int timed_buffer_number_entries_loaded = 0;
int timed_buffer_entry_pointer = 0;
int timed_buffer_interval_value_seconds = 0;
unsigned long last_timed_buffer_action_time = 0;
byte timed_buffer_status = 0;
#endif //FEATURE_TIMED_BUFFER

extern byte push_lcd_update;

#ifdef FEATURE_ELEVATION_CONTROL
int elevation = 0;
int target_elevation = 0;

byte el_request = 0;
int el_request_parm = 0;
byte el_request_queue_state = NONE;
byte el_slowstart_active = EL_SLOWSTART_DEFAULT;
byte el_slowdown_active = EL_SLOWDOWN_DEFAULT;
unsigned long el_slowstart_start_time = 0;
byte el_slow_start_step = 0;
unsigned long el_last_step_time = 0;
byte el_slow_down_step = 0;
unsigned long el_timed_slow_down_start_time = 0;
byte normal_el_speed_voltage = 0;
byte current_el_speed_voltage = 0;

int display_elevation = 0;   // Variable added to handle elevation beyond 90 degrees.  /// W3SA
byte el_state = IDLE;
int analog_el = 0;

unsigned long el_last_rotate_initiation = 0;
#ifdef FEATURE_TIMED_BUFFER
int timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
#endif //FEATURE_TIMED_BUFFER
byte elevation_button_was_pushed = 0;
#endif

#ifdef FEATURE_ROTARY_ENCODER_SUPPORT
#ifdef OPTION_ENCODER_HALF_STEP_MODE      // Use the half-step state table (emits a code at 00 and 11)
const unsigned char ttable[6][4] =
{
  {0x03, 0x02, 0x01, 0x00}, {0x23, 0x00, 0x01, 0x00},
  {0x13, 0x02, 0x00, 0x00}, {0x03, 0x05, 0x04, 0x00},
  {0x03, 0x03, 0x04, 0x10}, {0x03, 0x05, 0x03, 0x20}
};
#else                                     // Use the full-step state table (emits a code at 00 only)
const unsigned char ttable[7][4] =
{
  {0x00, 0x02, 0x04,  0x00}, {0x03, 0x00, 0x01, 0x10},
  {0x03, 0x02, 0x00,  0x00}, {0x03, 0x02, 0x01, 0x00},
  {0x06, 0x00, 0x04,  0x00}, {0x06, 0x05, 0x00, 0x10},
  {0x06, 0x05, 0x04,  0x00},
};

#endif //OPTION_ENCODER_HALF_STEP_MODE
#ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder State Tables
int az_encoder_raw_degrees = 0;
#define ENCODER_IDLE 0
#define ENCODER_AZ_PENDING 1
#define ENCODER_EL_PENDING 2
#define ENCODER_AZ_EL_PENDING 3
volatile unsigned char az_encoder_state = 0;
#ifdef FEATURE_EL_PRESET_ENCODER
volatile unsigned char el_encoder_state = 0;
int el_encoder_degrees = 0;
#endif //FEATURE_EL_PRESET_ENCODER
byte preset_encoders_state = ENCODER_IDLE;
#endif //FEATURE_AZ_PRESET_ENCODER
#endif //FEATURE_ROTARY_ENCODER_SUPPORT

#ifdef DEBUG_PROFILE_LOOP_TIME
extern float average_loop_time;
#endif //DEBUG_PROFILE_LOOP_TIME

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
volatile float az_position_pulse_input_azimuth = 0;
volatile byte last_known_az_state = 0;
#endif //FEATURE_AZ_POSITION_PULSE_INPUT

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
volatile float el_position_pulse_input_elevation = 0;
volatile byte last_known_el_state = 0;
#endif //FEATURE_EL_POSITION_PULSE_INPUT

#ifdef FEATURE_REMOTE_UNIT_SLAVE
byte serial_read_event_flag[] = {0,0,0,0,0};
byte serial0_buffer_carriage_return_flag = 0;
#endif

#ifdef FEATURE_HOST_REMOTE_PROTOCOL
byte serial1_buffer[COMMAND_BUFFER_SIZE];
int serial1_buffer_index = 0;
byte serial1_buffer_carriage_return_flag = 0;
unsigned long serial1_last_receive_time = 0;
byte remote_unit_command_submitted = 0;
unsigned long last_remote_unit_command_time = 0;
unsigned int remote_unit_command_timeouts = 0;
unsigned int remote_unit_bad_results = 0;
unsigned int remote_unit_good_results = 0;
unsigned int remote_unit_incoming_buffer_timeouts = 0;
byte remote_unit_command_results_available = 0;
float remote_unit_command_result_float = 0;
byte remote_port_rx_sniff = 0;
byte remote_port_tx_sniff = 0;
byte suspend_remote_commands = 0;
#endif

#ifdef DEBUG_POSITION_PULSE_INPUT
//unsigned int az_position_pule_interrupt_handler_flag = 0;
//unsigned int el_position_pule_interrupt_handler_flag = 0;
volatile unsigned long az_pulse_counter = 0;
volatile unsigned long el_pulse_counter = 0;
volatile unsigned int az_pulse_counter_ambiguous = 0;
volatile unsigned int el_pulse_counter_ambiguous = 0;
#endif //DEBUG_POSITION_PULSE_INPUT

/*
  Azimuth and Elevation calibration tables - use with FEATURE_AZIMUTH_CORRECTION and/or FEATURE_ELEVATION_CORRECTION
  You must have the same number of entries in the _from and _to arrays!
*/
#ifdef FEATURE_AZIMUTH_CORRECTION
float azimuth_calibration_from[]    = {180, 630};    /* these are in "raw" degrees, i.e. when going east past 360 degrees, add 360 degrees*/
float azimuth_calibration_to[]      = {180, 630};
#endif //FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
float elevation_calibration_from[]  = {-180, 0, 180};
float elevation_calibration_to[]    = { 180, 0, 180};
#endif //FEATURE_ELEVATION_CORRECTION

extern byte incoming_serial_byte;
extern byte serial0_buffer[COMMAND_BUFFER_SIZE];
extern int serial0_buffer_index;
extern unsigned long last_serial_receive_time;
extern byte backslash_command;
extern byte debug_mode;

extern byte configuration_dirty;

#endif