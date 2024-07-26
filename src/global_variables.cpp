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

#include "global_variables.h"

// initialize global variables called from setup()
// azimuth globals
float         Raz                                 = 0.0;   // Ohms, Resistance of top of azimuth pot
float         raw_azimuth                         = 0.0;   // deg, azimuth, may be unwrapped
float         azimuth                             = 0.0;   // deg, wrapped to 0 to 360
int           target_azimuth                      = 0;
int           target_raw_azimuth                  = 0;
char          az_request                          = 0;
int           az_request_parm                     = 0;
char          az_request_queue_state              = NONE;
char          az_state                            = IDLE;
unsigned long az_last_rotate_initiation           = 0;
bool          isAzButtonPressed                   = false; // original az button variable
char          brake_az_engaged                    = 0;
unsigned long az_slowstart_start_time             = 0;
char          az_slow_start_step                  = 0;
unsigned long az_last_step_time                   = 0;
char          az_slow_down_step                   = 0;
unsigned long az_timed_slow_down_start_time       = 0;
char          normal_az_speed_voltage             = 0;
char          current_az_speed_voltage            = 0;
char          az_slowstart_active                 = AZ_SLOWSTART_DEFAULT;
char          az_slowdown_active                  = AZ_SLOWDOWN_DEFAULT;

#ifdef FEATURE_HCO_BUTTONS
int           button_cw_press_time                = 0; // msec pressed, 0 means not pressed
int           button_ccw_press_time               = 0; // msec pressed, 0 means not pressed
bool          is_CW_cal_mode               = false;
bool          is_CCW_cal_mode                  = false;
#endif

#ifdef FEATURE_HCO_ADC
int           IsrTime                             = 0;  // test variable, ISR execution time in msec
#endif

char          debug_mode                          = DEFAULT_DEBUG_STATE;
unsigned long last_debug_output_time              = 0;

#ifdef FEATURE_TIMED_BUFFER
int           timed_buffer_number_entries_loaded  = 0;
int           timed_buffer_entry_pointer          = 0;
int           timed_buffer_interval_value_seconds = 0;
unsigned long last_timed_buffer_action_time       = 0;
char          timed_buffer_status                 = 0;

#endif //FEATURE_TIMED_BUFFER

char          push_lcd_update                     = 0;

#ifdef FEATURE_ELEVATION_CONTROL
int           elevation                           = 0;
int           target_elevation                    = 0;
brake_el_engaged                    = 0;
char          el_request                          = 0;
int           el_request_parm                     = 0;
char          el_request_queue_state              = NONE;
char          el_slowstart_active                 = EL_SLOWSTART_DEFAULT;
char          el_slowdown_active                  = EL_SLOWDOWN_DEFAULT;
unsigned long el_slowstart_start_time             = 0;
char          el_slow_start_step                  = 0;
unsigned long el_last_step_time                   = 0;
char          el_slow_down_step                   = 0;
unsigned long el_timed_slow_down_start_time       = 0;
char          normal_el_speed_voltage             = 0;
char          current_el_speed_voltage            = 0;
int           display_elevation                   = 0;   // Variable added to handle elevation beyond 90 degrees.  /// W3SA
char          el_state                            = IDLE;
int           analog_el                           = 0;
unsigned long el_last_rotate_initiation           = 0;

#ifdef FEATURE_TIMED_BUFFER
timed_buffer_elevations[TIMED_INTERVAL_ARRAY_SIZE];
#endif //FEATURE_TIMED_BUFFER

elevation_button_was_pushed        = 0;
#endif //FEATURE_ELEVATION_CONTROL

//#ifdef FEATURE_ROTARY_ENCODER_SUPPORT

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

#ifdef FEATURE_AZ_PRESET_POT
bool          is_display_preset                = false;
int           azimuth_preset;
#endif

#ifdef FEATURE_AZ_PRESET_ENCODER            // Rotary Encoder State Tables
int                    az_encoder_raw_degrees   = 0;
volatile unsigned char az_encoder_state         = 0;
#define ENCODER_IDLE 0
#define ENCODER_AZ_PENDING 1
#define ENCODER_EL_PENDING 2
#define ENCODER_AZ_EL_PENDING 3

#ifdef FEATURE_EL_PRESET_ENCODER
el_encoder_state = 0;
el_encoder_degrees = 0;
#endif //FEATURE_EL_PRESET_ENCODER

char                 preset_encoders_state     = ENCODER_IDLE;
#endif //FEATURE_AZ_PRESET_ENCODER

//#endif //FEATURE_ROTARY_ENCODER_SUPPORT

#ifdef DEBUG_PROFILE_LOOP_TIME
average_loop_time = 0;
#endif //DEBUG_PROFILE_LOOP_TIME

#ifdef FEATURE_AZ_POSITION_PULSE_INPUT
az_position_pulse_input_azimuth = 0;
last_known_az_state = 0;
#endif //FEATURE_AZ_POSITION_PULSE_INPUT

#ifdef FEATURE_EL_POSITION_PULSE_INPUT
el_position_pulse_input_elevation = 0;
last_known_el_state = 0;
#endif //FEATURE_EL_POSITION_PULSE_INPUT

#ifdef FEATURE_REMOTE_UNIT_SLAVE
serial_read_event_flag[] = {0,0,0,0,0};
serial0_buffer_carriage_return_flag = 0;
#endif

#ifdef FEATURE_HOST_REMOTE_PROTOCOL
serial1_buffer[COMMAND_BUFFER_SIZE];
serial1_buffer_index                    = 0;
serial1_buffer_carriage_return_flag     = 0;
serial1_last_receive_time               = 0;
remote_unit_command_submitted           = 0;
last_remote_unit_command_time           = 0;
remote_unit_command_timeouts            = 0;
remote_unit_bad_results                 = 0;
remote_unit_good_results                = 0;
remote_unit_incoming_buffer_timeouts    = 0;
remote_unit_command_results_available   = 0;
remote_unit_command_result_float        = 0;
remote_port_rx_sniff                    = 0;
remote_port_tx_sniff                    = 0;
suspend_remote_commands                 = 0;
#endif

#ifdef DEBUG_POSITION_PULSE_INPUT
//az_position_pule_interrupt_handler_flag = 0;
//el_position_pule_interrupt_handler_flag = 0;
az_pulse_counter                        = 0;
el_pulse_counter                        = 0;
az_pulse_counter_ambiguous              = 0;
el_pulse_counter_ambiguous              = 0;
#endif //DEBUG_POSITION_PULSE_INPUT

/*
Azimuth and Elevation calibration tables - use with FEATURE_AZIMUTH_CORRECTION and/or FEATURE_ELEVATION_CORRECTION
You must have the same number of entries in the _from and _to arrays!
*/
#ifdef FEATURE_AZIMUTH_CORRECTION
azimuth_calibration_from[]    = {180, 630};    /* these are in "raw" degrees, i.e. when going east past 360 degrees, add 360 degrees*/
azimuth_calibration_to[]      = {180, 630};
#endif //FEATURE_AZIMUTH_CORRECTION

#ifdef FEATURE_ELEVATION_CORRECTION
elevation_calibration_from[]  = {-180, 0, 180};
elevation_calibration_to[]    = { 180, 0, 180};
#endif //FEATURE_ELEVATION_CORRECTION

struct config_t configuration;
char            configuration_dirty   = 0;

char          serial0_buffer[COMMAND_BUFFER_SIZE];
int           serial0_buffer_index;
char          incoming_serial_byte;
unsigned long last_serial_receive_time;
char          backslash_command;