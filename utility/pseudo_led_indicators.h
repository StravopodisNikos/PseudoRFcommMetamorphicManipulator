#ifndef pseudo_led_indicators_h
#define pseudo_led_indicators_h

#include <avr/pgmspace.h>

//                    INDICATOR NAME		   	RGB VALUE		   COLOR
const PROGMEM unsigned char pseudo_turn_off_led[]              = {0, 0, 0};
const PROGMEM unsigned char pseudo_critical_error_led[]        = {255, 0, 0};
const PROGMEM unsigned char pseudo_entered_p2pcsp[]            = {0  ,0, 255 };       // deep blue
const PROGMEM unsigned char pseudo_pinged_indicator[]          = {0, 255, 0  };       // deep green
const PROGMEM unsigned char pseudo_stepping_in_progress[]      = {128, 128, 0};       // Olive
const PROGMEM unsigned char pseudo_completed_move_indicator[]  = {128, 0, 128};       // Purple
const PROGMEM unsigned char pseudo_torque_off_indicator[]     	= {148, 0, 211};       // dark violet
const PROGMEM unsigned char pseudo_motors_homing_indicator[]   = {255, 140, 0};       // orange
const PROGMEM unsigned char pseudo_motors_moving_indicator[]  	= {135, 206, 235};     // skyblue
const PROGMEM unsigned char pseudo_updated_force_indicator[]  	= {241, 235, 0};       // yellow
const PROGMEM unsigned char pseudo_updated_current_indicator[] = {255, 160, 122};     // LightSalmon
const PROGMEM unsigned char pseudo_updated_imu_indicator[]  	= {210, 45, 180};      // violet light
const PROGMEM unsigned char pseudo_updated_pos_indicator[]  	= {240, 128, 128};     // LightCoral
const PROGMEM unsigned char pseudo_updated_vel_indicator[]  	= {255, 0, 255};       // Fuchsia
const PROGMEM unsigned char pseudo_saved_data_log_dicator[]  	= {32, 62, 142};       // blue light
const PROGMEM unsigned char pseudo_access_eeprom_indicator[]  	= {210, 45, 180};      // violet light
const PROGMEM unsigned char pseudo_synced_motion_success[]  	= {180, 180, 180};     // grey light

#endif
