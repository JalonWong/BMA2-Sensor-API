/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bma2.h
* @date       2021-03-29
* @version    v0.3.0
*
*/

/*!
 * @defgroup bma2 BMA2
 */

#ifndef _BMA2_H
#define _BMA2_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bma2_defs.h"

/***************************************************************************/

/*!     BMA2 User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma2
 * \defgroup bma2ApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma2ApiInit
 * \page bma2_api_bma2_init bma2_init
 * \code
 * int8_t bma2_init(struct bma2_dev *dev);
 * \endcode
 * @details This API is the entry point, Call this API before using other APIs.
 * This API reads the chip-id of the sensor which is the first step to
 * verify the sensor
 *
 * @param[in,out] dev : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_init(struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiRegisters Registers
 * @brief Read / Write data to register address
 */

/*!
 * \ingroup bma2ApiRegisters
 * \page bma2_api_bma2_set_regs bma2_set_regs
 * \code
 * int8_t bma2_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bma2_dev *dev);
 * \endcode
 * @details This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address where the reg_data is to be written
 * @param[in] reg_data : Pointer to data buffer which is to be written
 *                       in the reg_addr of sensor
 * @param[in] length   : No of bytes of data to be written
 * @param[in] dev      : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiRegisters
 * \page bma2_api_bma2_get_regs bma2_get_regs
 * \code
 * int8_t bma2_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bma2_dev *dev);
 * \endcode
 * @details This API reads the data from the given register
 * address of the sensor
 *
 * @param[in] reg_addr  : Register address from where the data is to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data
 * @param[in] length    : No of bytes of data to be read
 * @param[in] dev       : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiSoftreset Soft Reset
 * @brief Performs soft-reset of the sensor
 */

/*!
 * \ingroup bma2ApiSoftreset
 * \page bma2_api_bma2_soft_reset bma2_soft_reset
 * \code
 * int8_t bma2_soft_reset(struct bma2_dev *dev);
 * \endcode
 * @details This API is used to perform soft-reset of the sensor
 * where all the registers are reset to their default values
 *
 * @param[in] dev       : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_soft_reset(struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiPowermode Power mode
 * @brief Power mode operations of the sensor
 */

/*!
 * \ingroup bma2ApiPowermode
 * \page bma2_api_bma2_set_power_mode bma2_set_power_mode
 * \code
 * int8_t bma2_set_power_mode(uint8_t power_mode, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the power mode of the sensor.
 *
 * @param[in] power_mode  : Macro to select power mode of the sensor
 * @param[in] dev         : Structure instance of bma2_dev
 *
 *@verbatim
 *       power_mode          |  Description
 * --------------------------|---------------
 * BMA2_NORMAL_MODE          | Normal mode
 * BMA2_DEEP_SUSPEND_MODE    | Deepsuspend mode
 * BMA2_LOW_POWER_MODE_1     | Low power mode 1
 * BMA2_SUSPEND_MODE         | Suspend mode
 * BMA2_LOW_POWER_MODE_2     | Low power mode 2
 * BMA2_STANDBY_MODE         | Standby mode
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_power_mode(uint8_t power_mode, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiPowermode
 * \page bma2_api_bma2_get_power_mode bma2_get_power_mode
 * \code
 * int8_t bma2_get_power_mode(uint8_t *power_mode, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the power mode of the sensor
 *
 * @param[out] power_mode  : Pointer to store power mode of the sensor
 * @param[in] dev          : Structure instance of bma2_dev
 *
 *@verbatim
 *       power_mode          |  Description
 * --------------------------|---------------
 * BMA2_NORMAL_MODE          | Normal mode
 * BMA2_DEEP_SUSPEND_MODE    | Deepsuspend mode
 * BMA2_LOW_POWER_MODE_1     | Low power mode 1
 * BMA2_SUSPEND_MODE         | Suspend mode
 * BMA2_LOW_POWER_MODE_2     | Low power mode 2
 * BMA2_STANDBY_MODE         | Standby mode
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_power_mode(uint8_t *power_mode, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiConfigurations Configurations
 * @brief Set / Get Sensor configurations
 */

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_accel_conf bma2_get_accel_conf
 * \code
 * int8_t bma2_get_accel_conf(struct bma2_acc_conf *accel_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the accelerometer configurations and store
 * them in the corresponding structure instance
 *
 * @param[in] accel_conf   : Structure instance of the configuration structure of accelometer
 * @param[in] dev          : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_accel_conf(struct bma2_acc_conf *accel_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_accel_conf bma2_set_accel_conf
 * \code
 * int8_t bma2_set_accel_conf(const struct bma2_acc_conf *accel_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set set the accelerometer configurations like
 * bandwidth, range, shadow_dis and data_high_bw)
 *
 * @param[in] accel_conf   : Structure instance of the configuration structure of accelometer
 * @param[in] dev          : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_accel_conf(const struct bma2_acc_conf *accel_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_slope_conf bma2_get_slope_conf
 * \code
 * int8_t bma2_get_slope_conf(struct bma2_slope_conf *slope_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the slope configurations
 *
 * @param[out] slope_conf  : Slope configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_slope_conf(struct bma2_slope_conf *slope_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_slope_conf bma2_set_slope_conf
 * \code
 * int8_t bma2_set_slope_conf(const struct bma2_slope_conf *slope_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the slope configurations
 *
 * @param[in] slope_conf  : Slope configuration structure instance
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_slope_conf(const struct bma2_slope_conf *slope_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_tap_conf bma2_get_tap_conf
 * \code
 * int8_t bma2_get_tap_conf(struct bma2_tap_conf *tap, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the tap configurations
 *
 * @param[out] tap         : Tap configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_tap_conf(struct bma2_tap_conf *tap, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_tap_conf bma2_set_tap_conf
 * \code
 * int8_t bma2_set_tap_conf(const struct bma2_tap_conf *tap, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the tap configurations
 *
 * @param[in] tap         : Tap configuration structure instance
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_tap_conf(const struct bma2_tap_conf *tap, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_orient_conf bma2_get_orient_conf
 * \code
 * int8_t bma2_get_orient_conf(struct bma2_orient_conf *orient, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the orient configurations
 *
 * @param[out] orient      : Orient configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_orient_conf(struct bma2_orient_conf *orient, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_orient_conf bma2_set_orient_conf
 * \code
 * int8_t bma2_set_orient_conf(const struct bma2_orient_conf *orient, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the orient configurations
 *
 * @param[in] orient      : Orient configuration structure instance
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_orient_conf(const struct bma2_orient_conf *orient, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_flat_conf bma2_get_flat_conf
 * \code
 * int8_t bma2_get_flat_conf(struct bma2_flat_conf *flat, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the flat configurations
 *
 * @param[out] flat        : Flat configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_flat_conf(struct bma2_flat_conf *flat, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_flat_conf bma2_set_flat_conf
 * \code
 * int8_t bma2_set_flat_conf(const struct bma2_flat_conf *flat, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the flat configurations
 *
 * @param[in] flat        : Flat configuration structure instance
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_flat_conf(const struct bma2_flat_conf *flat, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_low_g_conf bma2_get_low_g_conf
 * \code
 * int8_t bma2_get_low_g_conf(struct bma2_low_g_conf *low_g, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the low G configurations
 *
 * @param[out] low_g       : Low-G configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_low_g_conf(struct bma2_low_g_conf *low_g, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_low_g_conf bma2_set_low_g_conf
 * \code
 * int8_t bma2_set_low_g_conf(const struct bma2_low_g_conf *low_g, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the low G configurations
 *
 * @param[out] low_g       : Low-G configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_low_g_conf(const struct bma2_low_g_conf *low_g, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_high_g_conf bma2_get_high_g_conf
 * \code
 * int8_t bma2_get_high_g_conf(struct bma2_high_g_conf *high_g, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the high G configurations
 *
 * @param[out] high_g      : High-G configuration structure instance
 * @param[in] dev          : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_high_g_conf(struct bma2_high_g_conf *high_g, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_high_g_conf bma2_set_high_g_conf
 * \code
 * int8_t bma2_set_high_g_conf(const struct bma2_high_g_conf *high_g, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the high G configurations
 *
 * @param[in] high_g      : High-G configuration structure instance
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_high_g_conf(const struct bma2_high_g_conf *high_g, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_get_slo_no_mot_conf bma2_get_slo_no_mot_conf
 * \code
 * int8_t bma2_get_slo_no_mot_conf(struct bma2_slo_no_mot_conf *slo_no_mot, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the slow/no motion configurations
 *
 * @param[out] slo_no_mot    : Slow/no motion configuration structure instance
 * @param[in] dev            : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_slo_no_mot_conf(struct bma2_slo_no_mot_conf *slo_no_mot, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiConfigurations
 * \page bma2_api_bma2_set_slo_no_mot_conf bma2_set_slo_no_mot_conf
 * \code
 * int8_t bma2_set_slo_no_mot_conf(const struct bma2_slo_no_mot_conf *slo_no_mot, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the slow/no motion configurations
 *
 * @param[in] slo_no_mot      : Slow/no motion configuration structure instance
 * @param[in] dev             : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_slo_no_mot_conf(const struct bma2_slo_no_mot_conf *slo_no_mot, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiAccelData Accel Data
 * @brief Read accel data from the sensor
 */

/*!
 * \ingroup bma2ApiAccelData
 * \page bma2_api_bma2_get_accel_data bma2_get_accel_data
 * \code
 * int8_t bma2_get_accel_data(struct bma2_sensor_data *accel, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the accel data from the sensor
 * based on the variant specific resolution
 *
 * @param[out] accel    : Structure instance to store the accel data
 * @param[in] dev       : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 * @return BMA2_W_NO_NEW_AVAILABLE - Warning : No new data is available.
 */
int8_t bma2_get_accel_data(struct bma2_sensor_data *accel, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiTemperatureData Temerature Data
 * @brief Read temperature data from the sensor
 */

/*!
 * \ingroup bma2ApiTemperatureData
 * \page bma2_api_bma2_get_temperature_data bma2_get_temperature_data
 * \code
 * int8_t bma2_get_temperature_data(uint8_t *temp_raw_data, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the temperature raw data output.
 *
 * @param[in,out] temp_raw_data      : Variable to get temperature raw data.
 * @param[in] dev                    : Structure instance of bma2_dev.
 *
 * @note : The slope of temperature as per data-sheet is 0.5K/LSB And the center temperature is 23'C.
 * So the raw temperature data is conveted into deg C using the following formula.
 * temperature_data = (float)(((int8_t)temp_raw_data) * 0.5 + 23);
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_temperature_data(uint8_t *temp_raw_data, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiFifo Fifo
 * @brief Performs FIFO configurations
 */

/*!
 * \ingroup bma2ApiFifo
 * \page bma2_api_bma2_get_fifo_config bma2_get_fifo_config
 * \code
 * int8_t bma2_get_fifo_config(struct bma2_fifo_frame *fifo, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the fifo configurations like FIFO mode, axis, watermark level and frame count.
 *
 * @param[in/out] fifo   : Structure instance of bma2_fifo_frame
 * @param[in] dev        : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_fifo_config(struct bma2_fifo_frame *fifo, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiFifo
 * \page bma2_api_bma2_set_fifo_config bma2_set_fifo_config
 * \code
 * int8_t bma2_set_fifo_config(const struct bma2_fifo_frame *fifo, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the fifo configurations like FIFO mode, axis and watermark level.
 *
 * @param[in] fifo       : Structure instance of bma2_fifo_frame
 * @param[in] dev        : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_fifo_config(const struct bma2_fifo_frame *fifo, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiFifo
 * \page bma2_api_bma2_read_fifo_data bma2_read_fifo_data
 * \code
 * int8_t bma2_read_fifo_data(struct bma2_fifo_frame *fifo, struct bma2_dev *dev);
 *
 * \endcode
 * @details This API is used to read the FIFO data from the FIFO register
 *
 * @param[in/out] fifo      : Structure instance of bma2_fifo_frame
 * @param[in] dev           : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_read_fifo_data(struct bma2_fifo_frame *fifo, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiFifo
 * \page bma2_api_bma2_extract_accel bma2_extract_accel
 * \code
 * int8_t bma2_extract_accel(struct bma2_sensor_data *accel_data, uint16_t *acc_index, const struct bma2_fifo_frame *fifo);
 * \endcode
 * @details This API is used to extract the accel data from the fifo
 *
 * @param[out] accel_data   : Structure instance of bma2_sensor_data
 * @param[out] acc_index    : Variable to hold valid accel frames
 * @param[in/out] fifo      : Structure instance of bma2_fifo_frame
 */
int8_t bma2_extract_accel(struct bma2_sensor_data *accel_data, uint16_t *acc_index, const struct bma2_fifo_frame *fifo);

/**
 * \ingroup bma2
 * \defgroup bma2ApiInterrupt Interrupt
 * @brief Performs interrupt operations of the sensor
 */

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_enable_interrupt bma2_enable_interrupt
 * \code
 * int8_t bma2_enable_interrupt(uint32_t int_en, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to enable the various interrupts
 *
 * @param[in] int_en    : Macro to select interrupts to be enabled
 * @param[in] dev       : Structure instance of bma2_dev
 *
 *@verbatim
 *            int_en                 |  Description
 * ----------------------------------|---------------
 * BMA2_INT_EN_SLOPE_X_AXIS          | Enable slope x-axis interrupt
 * BMA2_INT_EN_SLOPE_Y_AXIS          | Enable slope y-axis interrupt
 * BMA2_INT_EN_SLOPE_Z_AXIS          | Enable slope z-axis interrupt
 * BMA2_INT_EN_DOUBLE_TAP            | Enable double tap interrupt
 * BMA2_INT_EN_SINGLE_TAP            | Enable single tap interrupt
 * BMA2_INT_EN_ORIENTATION           | Enable orientation interrupt
 * BMA2_INT_EN_FLAT                  | Enable flat interrupt
 * BMA2_INT_EN_HIGH_G_X_AXIS         | Enable high-g x-axis interrupt
 * BMA2_INT_EN_HIGH_G_Y_AXIS         | Enable high-g y-axis interrupt
 * BMA2_INT_EN_HIGH_G_Z_AXIS         | Enable high-g z-axis interrupt
 * BMA2_INT_EN_LOW_G                 | Enable low-g interrupt
 * BMA2_INT_EN_DATA_READY            | Enable data ready interrupt
 * BMA2_INT_EN_FIFO_FULL             | Enable fifo full interrupt
 * BMA2_INT_EN_FIFO_WM               | Enable fifo watermark interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS | Enable slow/no motion x-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS | Enable slow/no motion y-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS | Enable slow/no motion z-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_SEL    | Enable slow/no motion selection interrupt
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_enable_interrupt(uint32_t int_en, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_get_enabled_interrupts bma2_get_enabled_interrupts
 * \code
 * int8_t bma2_get_enabled_interrupts(uint32_t *int_en, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the various interrupts
 * which are enabled in the sensor
 *
 * @param[out] int_en     : Pointer to store interrupts enabled
 * @param[in] dev         : Structure instance of bma2_dev
 *
 *@verbatim
 *            int_en                 |  Description
 * ----------------------------------|---------------
 * BMA2_INT_EN_SLOPE_X_AXIS          | Enable slope x-axis interrupt
 * BMA2_INT_EN_SLOPE_Y_AXIS          | Enable slope y-axis interrupt
 * BMA2_INT_EN_SLOPE_Z_AXIS          | Enable slope z-axis interrupt
 * BMA2_INT_EN_DOUBLE_TAP            | Enable double tap interrupt
 * BMA2_INT_EN_SINGLE_TAP            | Enable single tap interrupt
 * BMA2_INT_EN_ORIENTATION           | Enable orientation interrupt
 * BMA2_INT_EN_FLAT                  | Enable flat interrupt
 * BMA2_INT_EN_HIGH_G_X_AXIS         | Enable high-g x-axis interrupt
 * BMA2_INT_EN_HIGH_G_Y_AXIS         | Enable high-g y-axis interrupt
 * BMA2_INT_EN_HIGH_G_Z_AXIS         | Enable high-g z-axis interrupt
 * BMA2_INT_EN_LOW_G                 | Enable low-g interrupt
 * BMA2_INT_EN_DATA_READY            | Enable data ready interrupt
 * BMA2_INT_EN_FIFO_FULL             | Enable fifo full interrupt
 * BMA2_INT_EN_FIFO_WM               | Enable fifo watermark interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS | Enable slow/no motion x-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS | Enable slow/no motion y-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS | Enable slow/no motion z-axis interrupt
 * BMA2_INT_EN_SLOW_NO_MOTION_SEL    | Enable slow/no motion selection interrupt
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_enabled_interrupts(uint32_t *int_en, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_get_int_status bma2_get_int_status
 * \code
 * int8_t bma2_get_int_status(struct bma2_int_status *int_status, struct bma2_dev *dev);
 *
 * \endcode
 * @details This API gets the interrupt status from the sensor.
 *
 * @param[out] int_status     : Structure instance of bma2_int_status structure
 * @param[in] dev             : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_int_status(struct bma2_int_status *int_status, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_set_int_out_ctrl bma2_set_int_out_ctrl
 * \code
 * int8_t bma2_set_int_out_ctrl(const struct bma2_int_pin *pin_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the interrupt pin configurations
 *
 * @param[in] pin_conf    : Interrupt pin configuration structure
 * @param[in] dev         : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_int_out_ctrl(const struct bma2_int_pin *pin_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_get_int_out_ctrl bma2_get_int_out_ctrl
 * \code
 * int8_t bma2_get_int_out_ctrl(struct bma2_int_pin *pin_conf, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the interrupt pin configurations
 *
 * @param[out] pin_conf       : Interrupt pin configuration structure
 * @param[in] dev             : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_int_out_ctrl(struct bma2_int_pin *pin_conf, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_set_int_mapping bma2_set_int_mapping
 * \code
 * int8_t bma2_set_int_mapping(uint8_t map, uint32_t int_map, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to map/unmap the interrupts to interrupt pins.
 *
 * @param[in] map        : Variable to select map/unmap interrupt
 * @param[in] int_map    : Macro that holds interrupt to be mapped/unmapped
 * @param[in] dev        : Structure instance of bma2_dev
 *
 *@verbatim
 *            map               |  Description
 * -----------------------------|---------------
 *         BMA2_INT_MAP         | Interrupt map
 *         BMA2_INT_UNMAP       | Interrupt unmap
 *
 *      int_map                 |  Description
 * -----------------------------|---------------
 * BMA2_INT1_MAP_LOW_G          | Map low-g interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_HIGH_G         | Map high-g interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SLOPE          | Map slope interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SLOW_NO_MOTION | Map slow/no motion interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_DOUBLE_TAP     | Map double tap interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SINGLE_TAP     | Map single tap interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_ORIENTATION    | Map orientation interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FLAT           | Map flat interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_DATA_READY     | Map data ready interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FIFO_WM        | Map fifo watermark interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FIFO_FULL      | Map fifo full interrupt to interrupt 1 pin
 * BMA2_INT2_MAP_FIFO_FULL      | Map fifo full interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_FIFO_WM        | Map fifo watermark interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_DATA_READY     | Map data ready interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_LOW_G          | Map low-g interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_HIGH_G         | Map high-g interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SLOPE          | Map slope interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SLOW_NO_MOTION | Map slow/no motion interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_DOUBLE_TAP     | Map double tap interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SINGLE_TAP     | Map single tap interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_ORIENTATION    | Map orientation interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_FLAT           | Map flat interrupt to interrupt 2 pin
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_int_mapping(uint8_t map, uint32_t int_map, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_get_int_mapping bma2_get_int_mapping
 * \code
 * int8_t bma2_get_int_mapping(uint32_t *int_map, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the mapped interrupts.
 *
 * @param[in] int_map        : Pointer to store interrupts mapped
 * @param[in] dev            : Structure instance of bma2_dev
 *
 *@verbatim
 *            map               |  Description
 * -----------------------------|---------------
 *      BMA2_INT_MAP            | Interrupt map
 *      BMA2_INT_UNMAP          | Interrupt unmap
 *
 *      int_map                 |  Description
 * -----------------------------|---------------
 * BMA2_INT1_MAP_LOW_G          | Map low-g interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_HIGH_G         | Map high-g interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SLOPE          | Map slope interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SLOW_NO_MOTION | Map slow/no motion interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_DOUBLE_TAP     | Map double tap interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_SINGLE_TAP     | Map single tap interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_ORIENTATION    | Map orientation interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FLAT           | Map flat interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_DATA_READY     | Map data ready interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FIFO_WM        | Map fifo watermark interrupt to interrupt 1 pin
 * BMA2_INT1_MAP_FIFO_FULL      | Map fifo full interrupt to interrupt 1 pin
 * BMA2_INT2_MAP_FIFO_FULL      | Map fifo full interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_FIFO_WM        | Map fifo watermark interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_DATA_READY     | Map data ready interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_LOW_G          | Map low-g interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_HIGH_G         | Map high-g interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SLOPE          | Map slope interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SLOW_NO_MOTION | Map slow/no motion interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_DOUBLE_TAP     | Map double tap interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_SINGLE_TAP     | Map single tap interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_ORIENTATION    | Map orientation interrupt to interrupt 2 pin
 * BMA2_INT2_MAP_FLAT           | Map flat interrupt to interrupt 2 pin
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_int_mapping(uint32_t *int_map, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_set_int_src bma2_set_int_src
 * \code
 * int8_t bma2_set_int_src(uint8_t filter, uint8_t int_src, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to select the interrupt source as filtered or unfiltered.
 *
 * @param[in] filter         : Variable that holds whether filtered/unfiltered source
 * @param[in] int_src        : Macro that holds the interrupt for which source is to be selected
 * @param[in] dev            : Structure instance of bma2_dev
 *
 *@verbatim
 *            filter           |  Description
 * ----------------------------|---------------
 * BMA2_INT_FILTERED_DATA      | Filtered data of interrupt
 * BMA2_INT_UNFILTERED_DATA    | Unfiltered data of interrupt
 *
 *      int_src                |  Description
 * ----------------------------|---------------
 * BMA2_INT_SRC_LOW_G          | Source for low-g interrupt
 * BMA2_INT_SRC_HIGH_G         | Source for high-g interrupt
 * BMA2_INT_SRC_SLOPE          | Source for slope interrupt
 * BMA2_INT_SRC_SLOW_NO_MOTION | Source for slow/no motion interrupt
 * BMA2_INT_SRC_TAP            | Source for tap interrupt
 * BMA2_INT_SRC_DATA           | Source for data interrupt
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_int_src(uint8_t filter, uint8_t int_src, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiInterrupt
 * \page bma2_api_bma2_int_rst_latch bma2_int_rst_latch
 * \code
 * int8_t bma2_int_rst_latch(uint8_t reset_cmd, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to reset the mode if it is in latched mode
 *
 * @param[in] reset_cmd       : Latched interrupt can be reset/kept as it is by this command
 * @param[in] dev             : Structure instance of bma2_dev
 *
 *@verbatim
 *  reset_cmd               |     Significance
 * -------------------------|---------------------------------
 *  BMA2_RESET_LATCHED_INT  | Clears all latched interrupts
 *  BMA2_LATCHED_INT_ACTIVE | Keeps latched interrupts active
 *@endverbatim
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_int_rst_latch(uint8_t reset_cmd, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiNVM NVM
 * @brief NVM progam API
 */

/*!
 * \ingroup bma2ApiNVM
 * \page bma2_api_bma2_nvm_prog bma2_nvm_prog
 * \code
 * int8_t bma2_nvm_prog(struct bma2_dev *dev);
 * \endcode
 * @details This API is used for NVM program
 *
 * @param[in] dev            : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_nvm_prog(struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiOffset Offset
 * @brief Offset API
 */

/*!
 * \ingroup bma2ApiOffset
 * \page bma2_api_bma2_get_offset_data bma2_get_offset_data
 * \code
 * int8_t bma2_get_offset_data(struct bma2_offset_data *offset_data, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to get the offset data
 *
 * @param[in] offset_data : Structure to store offset compensation value
 * @param[in] dev         : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_get_offset_data(struct bma2_offset_data *offset_data, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiOffset
 * \page bma2_api_bma2_set_offset_data bma2_set_offset_data
 * \code
 * int8_t bma2_set_offset_data(const struct bma2_offset_data *offset_data, struct bma2_dev *dev);
 * \endcode
 * @details This API is used to set the offset data
 *
 * @param[in] offset_data : Structure to store offset compensation value
 * @param[in] dev         : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @return 0 -> Success
 * @return < 0 -> Fail
 */
int8_t bma2_set_offset_data(const struct bma2_offset_data *offset_data, struct bma2_dev *dev);

/*!
 * \ingroup bma2ApiOffset
 * \page bma2_api_bma2_reset_offset bma2_reset_offset
 * \code
 * int8_t bma2_reset_offset(struct bma2_dev *dev);
 * \endcode
 * @details This API is used to reset the offset
 *
 * @param[in] dev            : Structure instance of bma2_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval >0 -> Warning
 * @retval <0 -> Fail
 */
int8_t bma2_reset_offset(struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiAccelSelftest Accel Self test
 * @brief Self test for accel
 */

/*!
 * \ingroup bma2ApiAccelSelftest
 * \page bma2_api_bma2_perform_accel_selftest bma2_perform_accel_selftest
 * \code
 * int8_t bma2_perform_accel_selftest(int8_t *result, struct bma2_dev *dev);
 * \endcode
 * @details This API checks whether the self test functionality of the sensor
 *  is working or not
 *
 *  @param[in] result : Pointer variable used to store the result of self test
 *  operation
 *
 *@verbatim
 * ------------------------------------------------------
 *  result   |  Description
 * ----------|-------------------------------------------
 *  0x00     | BMA2_SELFTEST_PASS
 *  0x01     | BMA2_SELFTEST_DIFF_X_AXIS_FAILED
 *  0x02     | BMA2_SELFTEST_DIFF_Y_AXIS_FAILED
 *  0x03     | BMA2_SELFTEST_DIFF_Z_AXIS_FAILED
 *  0x04     | BMA2_SELFTEST_DIFF_X_AND_Y_AXIS_FAILED
 *  0x05     | BMA2_SELFTEST_DIFF_X_AND_Z_AXIS_FAILED
 *  0x06     | BMA2_SELFTEST_DIFF_Y_AND_Z_AXIS_FAILED
 *  0x07     | BMA2_SELFTEST_DIFF_X_Y_AND_Z_AXIS_FAILED
 *  0x08     | BMA2_SELFTEST_FAIL
 * -------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma2_perform_accel_selftest(int8_t *result, struct bma2_dev *dev);

/**
 * \ingroup bma2
 * \defgroup bma2ApiComp Fast/Slow offset compensation
 * @brief Fast/Slow offset compensation
 */

/*!
 * \ingroup bma2ApiComp
 * \page bma2_api_bma2_fast_slow_offset_compensation bma2_fast_slow_offset_compensation
 * \code
 * int8_t bma2_fast_slow_offset_compensation(uint8_t cal_trigger, struct bma2_fast_slow_offset foc, struct bma2_dev *dev);
 * \endcode
 * @details This API performs fast offset compensation
 *
 *  @param[in] cal_trigger : Variable to trigger fast offset compensation
 *
 *@verbatim
 * ------------------------------------------------
 *      cal_trigger      |       Description
 * ----------------------|-------------------------
 *  BMA2_OFFSET_NONE     | Do not trigger FOC
 *  BMA2_OFFSET_X_AXIS   | Trigger FOC for X-Axis
 *  BMA2_OFFSET_Y_AXIS   | Trigger FOC for Y-Axis
 *  BMA2_OFFSET_Z_AXIS   | Trigger FOC for Z-Axis
 * ------------------------------------------------
 *@endverbatim
 *
 * @param[in] foc : Structure instance of bma2_fast_slow_offset
 * @param[in] dev : Structure instance of bma2_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma2_fast_slow_offset_compensation(const uint8_t *cal_trigger,
                                          const struct bma2_fast_slow_offset *foc,
                                          struct bma2_dev *dev);

/*! CPP guard */
#ifdef __cplusplus
}
#endif

#endif /* _BMA2_H */
