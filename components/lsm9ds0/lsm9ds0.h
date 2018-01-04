/**
 * Driver for LSM9DS0 module: 3D accelerometer, 3D gyroscope, 3D magnetometer
 *
 * This driver is for the usage with the ESP8266 and FreeRTOS (esp-open-rtos)
 * [https://github.com/SuperHouse/esp-open-rtos]. It is also working with ESP32
 * and ESP-IDF [https://github.com/espressif/esp-idf.git] as well as Linux
 * based systems using a wrapper library for ESP8266 functions.
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2017 Gunar Schorcht (https://github.com/gschorcht)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __LSM9DS0_H__
#define __LSM9DS0_H__

// Uncomment one of the following defines to enable debug output
// #define LSM9DS0_DEBUG_LEVEL_1    // only error messages
// #define LSM9DS0_DEBUG_LEVEL_2    // debug and error messages

#ifdef LSM9DS0_DEBUG_LEVEL_1
#define L3GD20H_DEBUG_LEVEL_1  1
#define LSM303D_DEBUG_LEVEL_1  1
#endif

#ifdef LSM9DS0_DEBUG_LEVEL_2
#define L3GD20H_DEBUG_LEVEL_2  2
#define LSM303D_DEBUG_LEVEL_2  2
#endif

#include "lsm303d.h"
#include "l3gd20h.h"

#define LSM9DS0_I2C_AM_ADDRESS_1  LSM303D_I2C_ADDRESS_1 // SDO_XM pin is low
#define LSM9DS0_I2C_AM_ADDRESS_2  LSM303D_I2C_ADDRESS_2 // SDO_XM pin is high

#define LSM9DS0_I2C_G_ADDRESS_1   L3GD20H_I2C_ADDRESS_1 // SDO_G pin is low
#define LSM9DS0_I2C_G_ADDRESS_2   L3GD20H_I2C_ADDRESS_2 // SDO_G pin is high

/*
 * Mapping of LSM303D accelerometer and magnetometer sensor types
 */

// mapping of lsm303d_a_odr_t
#define lsm9ds0_a_power_down          lsm303d_a_power_down
#define lsm9ds0_a_odr_3_125           lsm303d_a_odr_3_125
#define lsm9ds0_a_odr_6_25            lsm303d_a_odr_6_25
#define lsm9ds0_a_odr_12_5            lsm303d_a_odr_12_5
#define lsm9ds0_a_odr_25              lsm303d_a_odr_25
#define lsm9ds0_a_odr_50              lsm303d_a_odr_50
#define lsm9ds0_a_odr_100             lsm303d_a_odr_100
#define lsm9ds0_a_odr_200             lsm303d_a_odr_200
#define lsm9ds0_a_odr_400             lsm303d_a_odr_400
#define lsm9ds0_a_odr_800             lsm303d_a_odr_800
#define lsm9ds0_a_odr_1600            lsm303d_a_odr_1600
#define lsm9ds0_a_odr_t               lsm303d_a_odr_t

// mapping of lsm303d_m_odr_t
#define lsm9ds0_m_odr_3_125           lsm303d_m_odr_3_125
#define lsm9ds0_m_odr_6_25            lsm303d_m_odr_6_25
#define lsm9ds0_m_odr_12_5            lsm303d_m_odr_12_5
#define lsm9ds0_m_odr_25              lsm303d_m_odr_25
#define lsm9ds0_m_odr_50              lsm303d_m_odr_50
#define lsm9ds0_m_odr_100             lsm303d_m_odr_100
#define lsm9ds0_m_do_not_use          lsm303d_m_do_not_use
#define lsm9ds0_m_low_power           lsm303d_m_low_power
#define lsm9ds0_m_odr_t               lsm303d_m_odr_t

// mapping of lsm303d_a_scale_t
#define lsm9ds0_a_scale_2_g           lsm303d_a_scale_2_g
#define lsm9ds0_a_scale_4_g           lsm303d_a_scale_4_g
#define lsm9ds0_a_scale_6_g           lsm303d_a_scale_6_g
#define lsm9ds0_a_scale_8_g           lsm303d_a_scale_8_g
#define lsm9ds0_a_scale_16_g          lsm303d_a_scale_16_g
#define lsm9ds0_a_scale_t             lsm303d_a_scale_t

// mapping of lsm303d_m_scale_t
#define lsm9ds0_m_scale_2_Gs          lsm303d_m_scale_2_Gs
#define lsm9ds0_m_scale_4_Gs          lsm303d_m_scale_4_Gs
#define lsm9ds0_m_scale_8_Gs          lsm303d_m_scale_8_Gs
#define lsm9ds0_m_scale_12_Gs         lsm303d_m_scale_12_Gs
#define lsm9ds0_m_scale_t             lsm303d_m_scale_t

// mapping of lsm303d_a_aaf_bw_t
#define lsm9ds0_a_aaf_bw_773          lsm303d_a_aaf_bw_773
#define lsm9ds0_a_aaf_bw_194          lsm303d_a_aaf_bw_194
#define lsm9ds0_a_aaf_bw_362          lsm303d_a_aaf_bw_362
#define lsm9ds0_a_aaf_bw_50           lsm303d_a_aaf_bw_50
#define lsm9ds0_a_aaf_bw_t            lsm303d_a_aaf_bw_t

// mapping of lsm303d_m_mode_t
#define lsm9ds0_m_continuous          lsm303d_m_continuous
#define lsm9ds0_m_single              lsm303d_m_single
#define lsm9ds0_m_power_down          lsm303d_m_power_down
#define lsm9ds0_m_mode_t              lsm303d_m_mode_t

// mapping of lsm303d_m_resolution_t
#define lsm9ds0_m_low_res             lsm303d_m_low_res
#define lsm9ds0_m_high_res            lsm303d_m_high_res
#define lsm9ds0_m_resolution_t        lsm303d_m_resolution_t

// mapping of lsm303d_fifo_mode_t (also valid for l3gd20h_fifo_mode_t)
#define lsm9ds0_bypass                lsm303d_bypass
#define lsm9ds0_fifo                  lsm303d_fifo
#define lsm9ds0_stream                lsm303d_stream
#define lsm9ds0_stream_to_fifo        lsm303d_stream_to_fifo
#define lsm9ds0_bypass_to_stream      lsm303d_bypass_to_stream
#define lsm9ds0_fifo_mode_t           lsm303d_fifo_mode_t

// mapping of lsm303d_int_signal_t
#define lsm9ds0_int_am_signal1        lsm303d_int1_signal
#define lsm9ds0_int_am_signal2        lsm303d_int2_signal
#define lsm9ds0_int_am_signal_t       lsm303d_int_signal_t

// mapping of lsm303d_int_signal_type_t (also valid for l3gd20h_signal_type_t)
#define lsm9ds0_int_push_pull         lsm303d_push_pull
#define lsm9ds0_int_open_drain        lsm303d_open_drain
#define lsm9ds0_int_signal_type_t     lsm303d_int_signal_type_t

// mapping of lsm303d_int_event_gen_t
#define lsm9ds0_int_a_event1_gen      lsm303d_int_event1_gen
#define lsm9ds0_int_a_event2_gen      lsm303d_int_event2_gen
#define lsm9ds0_int_a_event_gen_t     lsm303d_int_event_gen_t

// mapping of lsm303d_int_type_t
#define lsm9ds0_int_a_data_ready      lsm303d_int_a_data_ready
#define lsm9ds0_int_m_data_ready      lsm303d_int_m_data_ready
#define lsm9ds0_int_a_fifo_empty      lsm303d_int_fifo_empty
#define lsm9ds0_int_a_fifo_thresh     lsm303d_int_fifo_thresh
#define lsm9ds0_int_a_fifo_overrun    lsm303d_int_fifo_overrun
#define lsm9ds0_int_a_event1          lsm303d_int_event1
#define lsm9ds0_int_a_event2          lsm303d_int_event2
#define lsm9ds0_int_a_click           lsm303d_int_click
#define lsm9ds0_int_m_thresh          lsm303d_int_m_thresh
#define lsm9ds0_int_am_type_t         lsm303d_int_type_t

#define lsm9ds0_int_am_data_source_t  lsm303d_int_data_source_t

#define lsm9ds0_or                    lsm303d_or
#define lsm9ds0_and                   lsm303d_and
#define lsm9ds0_6d_movement           lsm303d_6d_movement
#define lsm9ds0_6d_position           lsm303d_6d_position
#define lsm9ds0_4d_movement           lsm303d_4d_movement
#define lsm9ds0_4d_position           lsm303d_4d_position

#define lsm9ds0_int_a_event_config_t  lsm303d_int_event_config_t
#define lsm9ds0_int_a_event_source_t  lsm303d_int_event_source_t
#define lsm9ds0_int_a_click_config_t  lsm303d_int_click_config_t
#define lsm9ds0_int_a_click_source_t  lsm303d_int_click_source_t
#define lsm9ds0_int_m_thresh_config_t lsm303d_int_m_thresh_config_t
#define lsm9ds0_int_m_thresh_source_t lsm303d_int_m_thresh_source_t

#define lsm9ds0_int_m_low_active      lsm303d_low_active
#define lsm9ds0_int_m_high_active     lsm303d_high_active

// mapping of lsm303d_hpf_mode_t (also valid for l3gd20h_hpf_mode_t)
#define lsm9ds0_hpf_normal            lsm303d_hpf_normal
#define lsm9ds0_hpf_reference         lsm303d_hpf_reference
#define lsm9ds0_hpf_normal_x          lsm303d_hpf_normal_x
#define lsm9ds0_hpf_autoreset         lsm303d_hpf_autoreset
#define lsm9ds0_hpf_mode_t            lsm303d_hpf_mode_t

#define lsm9ds0_raw_a_data_t          lsm303d_raw_a_data_t
#define lsm9ds0_raw_a_data_fifo_t     lsm303d_raw_a_data_fifo_t
#define lsm9ds0_float_a_data_t        lsm303d_float_a_data_t
#define lsm9ds0_float_a_data_fifo_t   lsm303d_float_a_data_fifo_t

#define lsm9ds0_raw_m_data_t          lsm303d_raw_m_data_t
#define lsm9ds0_float_m_data_t        lsm303d_float_m_data_t

#define lsm9ds0_am_sensor_t           lsm303d_sensor_t

/*
 * Mapping of L3GD20 gyroscope sensor types
 */

// mapping of l3gd20h_mode_t
#define lsm9ds0_g_power_down          l3gd20h_power_down
#define lsm9ds0_g_odr_95              l3gd20h_normal_odr_100
#define lsm9ds0_g_odr_190             l3gd20h_normal_odr_200
#define lsm9ds0_g_odr_380             l3gd20h_normal_odr_400
#define lsm9ds0_g_odr_760             l3gd20h_normal_odr_800
#define lsm9ds0_g_odr_t               l3gd20h_mode_t

// mapping of l3gd20h_scale_t
#define lsm9ds0_g_scale_245_dps       l3gd20h_scale_245_dps
#define lsm9ds0_g_scale_500_dps       l3gd20h_scale_500_dps
#define lsm9ds0_g_scale_2000_dps      l3gd20h_scale_2000_dps
#define lsm9ds0_g_scale_t             l3gd20h_scale_t

// mapping of l3gd20h_fifo_mode_t (corresponds to lsm303d_fifo_mode_t)
// #define lsm9ds0_bypass             l3gd20h_bypass
// #define lsm9ds0_fifo               l3gd20h_fifo
// #define lsm9ds0_stream             l3gd20h_stream
// #define lsm9ds0_stream_to_fifo     l3gd20h_stream_to_fifo
// #define lsm9ds0_bypass_to_stream   l3gd20h_bypass_to_stream
// #define lsm9ds0_fifo_mode_t        l3gd20h_fifo_mode_t

// mapping of l3gd20h_filter_t
#define lsm9ds0_g_no_filter           l3gd20h_no_filter
#define lsm9ds0_g_hpf_only            l3gd20h_hpf_only
#define lsm9ds0_g_lpf2_only           l3gd20h_lpf2_only
#define lsm9ds0_g_hpf_and_lpf2        l3gd20h_hpf_and_lpf2
#define lsm9ds0_g_filter_t            l3gd20h_filter_t

// mapping of l3gd20h_hpf_mode_t (corresponds to lsm303d_hpf_mode_t)
// #define lsm9ds0_hpf_normal         l3gd20h_hpf_normal
// #define lsm9ds0_hpf_reference      l3gd20h_hpf_reference
// #define lsm9ds0_hpf_normal_x       l3gd20h_hpf_normal_x
// #define lsm9ds0_hpf_autoreset      l3gd20h_hpf_autoreset
// #define lsm9ds0_hpf_mode_t         l3gd20h_hpf_mode_t

// mapping of l3gd20h_int_types_t
#define lsm9ds0_int_g_data_ready      l3gd20h_int_data_ready
#define lsm9ds0_int_g_fifo_thresh     l3gd20h_int_fifo_threshold
#define lsm9ds0_int_g_fifo_overrun    l3gd20h_int_fifo_overrun
#define lsm9ds0_int_g_fifo_empty      l3gd20h_int_fifo_empty
#define lsm9ds0_int_g_event           l3gd20h_int_event
#define lsm9ds0_int_g_types_t         l3gd20h_int_types_t

#define lsm9ds0_int_g_data_source_t   l3gd20h_int_data_source_t
#define lsm9ds0_int_g_event_config_t  l3gd20h_int_event_config_t
#define lsm9ds0_int_g_event_source_t  l3gd20h_int_event_source_t

#define lsm9ds0_int_g_high_active     l3gd20h_high_active
#define lsm9ds0_int_g_low_active      l3gd20h_low_active
#define lsm9ds0_int_g_signal_level_t  l3gd20h_signal_level_t

// mapping of l3gd20h_signal_type_t (corresponds to lsm303d_signal_type_t)
// #define lsm9ds0_int_push_pull      l3gd20h_push_pull
// #define lsm9ds0_int_open_drain     l3gd20h_open_drain
// #define lsm9ds0_int_signal_type_t  l3gd20h_signal_type_t

#define lsm9ds0_raw_g_data_t          l3gd20h_raw_data_t
#define lsm9ds0_raw_g_data_fifo_t     l3gd20h_raw_data_fifo_t
#define lsm9ds0_float_g_data_t        l3gd20h_float_data_t
#define lsm9ds0_float_g_data_fifo_t   l3gd20h_float_data_fifo_t

#define lsm9ds0_g_sensor_t            l3gd20h_sensor_t

/*
 * Mapping of LSM303D accelerometer and magnetometer sensor functions
 */

#define lsm9ds0_init_am_sensor          lsm303d_init_sensor
#define lsm9ds0_set_a_mode              lsm303d_set_a_mode
#define lsm9ds0_set_m_mode              lsm303d_set_m_mode
#define lsm9ds0_set_a_scale             lsm303d_set_a_scale
#define lsm9ds0_set_m_scale             lsm303d_set_m_scale
#define lsm9ds0_new_a_data              lsm303d_new_a_data
#define lsm9ds0_new_m_data              lsm303d_new_m_data
#define lsm9ds0_get_float_a_data        lsm303d_get_float_a_data
#define lsm9ds0_get_float_a_data_fifo   lsm303d_get_float_a_data_fifo
#define lsm9ds0_get_float_m_data        lsm303d_get_float_m_data
#define lsm9ds0_get_raw_a_data          lsm303d_get_raw_a_data
#define lsm9ds0_get_raw_a_data_fifo     lsm303d_get_raw_a_data_fifo
#define lsm9ds0_get_raw_m_data          lsm303d_get_raw_m_data
#define lsm9ds0_set_a_fifo_mode         lsm303d_set_fifo_mode
#define lsm9ds0_enable_int_am           lsm303d_enable_int
#define lsm9ds0_get_int_am_data_source  lsm303d_get_int_data_source
#define lsm9ds0_set_int_m_thresh_config lsm303d_set_int_m_thresh_config
#define lsm9ds0_get_int_m_thresh_config lsm303d_get_int_m_thresh_config
#define lsm9ds0_get_int_m_thresh_source lsm303d_get_int_m_thresh_source
#define lsm9ds0_set_int_a_event_config  lsm303d_set_int_event_config
#define lsm9ds0_get_int_a_event_config  lsm303d_get_int_event_config
#define lsm9ds0_get_int_a_event_source  lsm303d_get_int_event_source
#define lsm9ds0_set_int_a_click_config  lsm303d_set_int_click_config
#define lsm9ds0_get_int_a_click_config  lsm303d_get_int_click_config
#define lsm9ds0_get_int_a_click_source  lsm303d_get_int_click_source
#define lsm9ds0_config_int_am_signals   lsm303d_config_int_signals
#define lsm9ds0_config_a_hpf            lsm303d_config_a_hpf
#define lsm9ds0_set_a_hpf_ref           lsm303d_set_a_hpf_ref
#define lsm9ds0_get_a_hpf_ref           lsm303d_get_a_hpf_ref
#define lsm9ds0_set_m_offset            lsm303d_set_m_offset
#define lsm9ds0_get_m_offset            lsm303d_get_m_offsetconfig_int_

#define lsm9ds0_enable_temperature      lsm303d_enable_temperature
#define lsm9ds0_get_temperature         lsm303d_get_temperature

#define lsm9ds0_reg_am_write            lsm303d_reg_write
#define lsm9ds0_reg_am_read             lsm303d_reg_read

/*
 * Mapping of L3GD20 gyroscope sensor functions
 */

#define lsm9ds0_init_g_sensor           l3gd20h_init_sensor
#define lsm9ds0_set_g_mode              l3gd20h_set_mode
#define lsm9ds0_set_g_scale             l3gd20h_set_scale
#define lsm9ds0_set_g_fifo_mode         l3gd20h_set_fifo_mode
#define lsm9ds0_select_g_output_filter  l3gd20h_select_output_filter
#define lsm9ds0_new_g_data              l3gd20h_new_data
#define lsm9ds0_get_float_g_data        l3gd20h_get_float_data
#define lsm9ds0_get_float_g_data_fifo   l3gd20h_get_float_data_fifo
#define lsm9ds0_get_raw_g_data          l3gd20h_get_raw_data
#define lsm9ds0_get_raw_g_data_fifo     l3gd20h_get_raw_data_fifo
#define lsm9ds0_enable_int_g            l3gd20h_enable_int
#define lsm9ds0_set_int_g_event_config  l3gd20h_set_int_event_config
#define lsm9ds0_get_int_g_event_config  l3gd20h_get_int_event_config
#define lsm9ds0_get_int_g_event_source  l3gd20h_get_int_event_source
#define lsm9ds0_get_int_g_data_source   l3gd20h_get_int_data_source
#define lsm9ds0_config_int_g_signals    l3gd20h_config_int_signals
#define lsm9ds0_config_g_hpf            l3gd20h_config_hpf
#define lsm9ds0_set_g_hpf_ref           l3gd20h_set_hpf_ref
#define lsm9ds0_get_g_hpf_ref           l3gd20h_get_hpf_ref

#define lsm9ds0_reg_g_write             l3gd20h_reg_write
#define lsm9ds0_reg_g_read              l3gd20h_reg_read

#endif // __LSM9DS0_H__
