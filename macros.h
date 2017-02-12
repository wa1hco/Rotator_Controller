/*
 * macros.h
 *
 *  Created on: Feb 12, 2017
 *      Author: jeff
 */

#ifndef MACROS_H_
#define MACROS_H_

/*---------------------- macros - don't touch these unless you know what you are doing ---------------------*/
#define AZ 1
#define EL 2

#ifdef FEATURE_ROTARY_ENCODER_SUPPORT
#define DIR_CCW 0x10                      //  CW Encoder Code (Do not change)
#define DIR_CW  0x20                      // CCW Encoder Code (Do not change)
#endif //FEATURE_ROTARY_ENCODER_SUPPORT

//az_state
#define IDLE                             0
#define SLOW_START_CW                    1
#define SLOW_START_CCW                   2
#define NORMAL_CW                        3
#define NORMAL_CCW                       4
#define SLOW_DOWN_CW                     5
#define SLOW_DOWN_CCW                    6
#define INITIALIZE_SLOW_START_CW         7
#define INITIALIZE_SLOW_START_CCW        8
#define INITIALIZE_TIMED_SLOW_DOWN_CW    9
#define INITIALIZE_TIMED_SLOW_DOWN_CCW  10
#define TIMED_SLOW_DOWN_CW              11
#define TIMED_SLOW_DOWN_CCW             12
#define INITIALIZE_DIR_CHANGE_TO_CW     13
#define INITIALIZE_DIR_CHANGE_TO_CCW    14
#define INITIALIZE_NORMAL_CW            15
#define INITIALIZE_NORMAL_CCW           16

//el_state
#define SLOW_START_UP                    1
#define SLOW_START_DOWN                  2
#define NORMAL_UP                        3
#define NORMAL_DOWN                      4
#define SLOW_DOWN_DOWN                   5
#define SLOW_DOWN_UP                     6
#define INITIALIZE_SLOW_START_UP         7
#define INITIALIZE_SLOW_START_DOWN       8
#define INITIALIZE_TIMED_SLOW_DOWN_UP    9
#define INITIALIZE_TIMED_SLOW_DOWN_DOWN 10
#define TIMED_SLOW_DOWN_UP              11
#define TIMED_SLOW_DOWN_DOWN            12
#define INITIALIZE_DIR_CHANGE_TO_UP     13
#define INITIALIZE_DIR_CHANGE_TO_DOWN   14
#define INITIALIZE_NORMAL_UP            15
#define INITIALIZE_NORMAL_DOWN          16

//az_request & el_request
#define REQUEST_STOP                     0
#define REQUEST_AZIMUTH                  1
#define REQUEST_AZIMUTH_RAW              2
#define REQUEST_CW                       3
#define REQUEST_CCW                      4
#define REQUEST_UP                       1
#define REQUEST_DOWN                     2
#define REQUEST_ELEVATION                3
#define REQUEST_KILL                     5

#define DEACTIVATE                       0
#define ACTIVATE                         1

#define CW                               1
#define CCW                              2
#define STOP_AZ                          3
#define STOP_EL                          4
#define UP                               5
#define DOWN                             6
#define STOP                             7

//az_request_queue_state & el_request_queue_state
#define NONE                             0
#define IN_QUEUE                         1
#define IN_PROGRESS_TIMED                2
#define IN_PROGRESS_TO_TARGET            3

#define EMPTY 0
#define LOADED_AZIMUTHS                  1
#define RUNNING_AZIMUTHS                 2
#ifdef FEATURE_ELEVATION_CONTROL
#define LOADED_AZIMUTHS_ELEVATIONS       3
#define RUNNING_AZIMUTHS_ELEVATIONS      4
#endif //FEATURE_ELEVATION_CONTROL

#define LCD_UNDEF                        0
#define LCD_HEADING                      1
#define LCD_DIRECTION                    2
#define LCD_TARGET_AZ                    3
#define LCD_TARGET_EL                    4
#define LCD_TARGET_AZ_EL                 5
#define LCD_ROTATING_CW                  6
#define LCD_ROTATING_CCW                 7
#define LCD_ROTATING_TO                  8
#define LCD_ELEVATING_TO                 9
#define LCD_ELEVATING_UP                10
#define LCD_ELEVATING_DOWN              11
#define LCD_ROTATING_AZ_EL              12

#define NOT_DOING_ANYTHING               0
#define ROTATING_CW                      1
#define ROTATING_CCW                     2
#define ROTATING_UP                      3
#define ROTATING_DOWN                    4

#define REMOTE_UNIT_NO_COMMAND           0
#define REMOTE_UNIT_AZ_COMMAND           1
#define REMOTE_UNIT_EL_COMMAND           2
/* ------end of macros ------- */

#endif /* MACROS_H_ */
