/*
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * The license is available at root folder
 *
 */

#include <stdio.h>
#include "bma2.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Meter per second squared data.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);

/******************************************************************************/
int main(void)
{
    int8_t rslt = 0;
    struct bma2_dev dev = { 0 };
    struct bma2_int_status int_status;
    uint16_t idx = 0;
    uint32_t int_en, int_map;
    struct bma2_sensor_data accel;
    uint8_t power_mode;
    float x = 0, y = 0, z = 0;
    struct bma2_acc_conf accel_conf;

    /* Interface reference is given as a parameter
     *         For I2C : BMA2_I2C_INTF
     *         For SPI : BMA2_SPI_INTF
     */
    rslt = bma2_interface_init(&dev, BMA2_I2C_INTF);
    bma2_error_codes_print_result("bma2_interface_init", rslt);

    rslt = bma2_init(&dev);
    bma2_error_codes_print_result("bma2_init", rslt);
    printf("Chip id : 0x%x\n", dev.chip_id);

    rslt = bma2_get_accel_conf(&accel_conf, &dev);
    bma2_error_codes_print_result("bma2_get_accel_conf", rslt);

    accel_conf.range = BMA2_ACC_RANGE_2G;

    rslt = bma2_set_accel_conf(&accel_conf, &dev);
    bma2_error_codes_print_result("bma2_set_accel_conf", rslt);

    rslt = bma2_set_power_mode(BMA2_NORMAL_MODE, &dev);
    bma2_error_codes_print_result("bma2_set_power_mode", rslt);

    rslt = bma2_get_power_mode(&power_mode, &dev);
    bma2_error_codes_print_result("bma2_get_power_mode", rslt);

    printf("Power mode set is : %d \n", power_mode);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_DATA_READY, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_interrupts_enabled", rslt);

    rslt = bma2_set_int_src(BMA2_INT_FILTERED_DATA, BMA2_INT_SRC_DATA, &dev);
    bma2_error_codes_print_result("bma2_set_int_src", rslt);

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_DATA_READY, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    rslt = bma2_get_int_mapping(&int_map, &dev);
    bma2_error_codes_print_result("bma2_get_int_mapping", rslt);

    printf("Accel data collected at 2G Range\n\n");

    while (idx < 50)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_1 & BMA2_INT_1_ASSERTED_DATA_READY)
        {
            rslt = bma2_get_accel_data(&accel, &dev);
            bma2_error_codes_print_result("bma2_get_accel_data", rslt);

            if (rslt == BMA2_OK)
            {
                printf("Accel[%d] : Raw_Accel_X : %d, Raw_Accel_Y : %d, Raw_Accel_Z : %d    ",
                       idx,
                       accel.x,
                       accel.y,
                       accel.z);

                /* Converting lsb to meter per second squared for respective bit resolution at 2G range */
                x = lsb_to_ms2(accel.x, 2, dev.resolution);
                y = lsb_to_ms2(accel.y, 2, dev.resolution);
                z = lsb_to_ms2(accel.z, 2, dev.resolution);

                /* Print the data in m/s2 */
                printf("Acc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);

                idx++;
            }
        }
    }

    bma2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to meters per seconds square.
 */
static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}
