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

/*! Sample count to display before/after performing offset compensation */
#define OFFSET_SAMPLE_COUNT  UINT8_C(100)

/******************************************************************************/

/*!
 *  @brief This function converts lsb to g for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity data.
 */
static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This internal API Performs FOC
 *
 *  @param[in] cal_trigger    : Axis for which FOC is to be performed
 *  @param[in] foc            : Structure instance of bma2_fast_slow_offset
 *  @param[in] dev            : Structure instance of bma2_dev
 *
 *  @return Result of API execution status
 *  @return 0 -> Success
 *  @return < 0 -> Fail
 */
static int8_t perform_foc_test(uint8_t cal_trigger, struct bma2_fast_slow_offset foc, struct bma2_dev *dev);

/******************************************************************************/
int main(void)
{
    int8_t rslt = 0;
    uint8_t power_mode;
    uint8_t cal_trigger;
    struct bma2_int_status int_status;
    uint16_t idx = 0;
    uint32_t int_en, int_map;
    struct bma2_sensor_data accel;
    float x = 0, y = 0, z = 0;
    struct bma2_offset_data offset_data;
    struct bma2_fast_slow_offset foc = { 0 };
    struct bma2_acc_conf accel_conf;

    struct bma2_dev dev = { 0 };

    /* Interface reference is given as a parameter
     *         For I2C : BMA2_I2C_INTF
     *         For SPI : BMA2_SPI_INTF
     */
    rslt = bma2_interface_init(&dev, BMA2_I2C_INTF);
    bma2_error_codes_print_result("bma2_interface_init", rslt);

    rslt = bma2_init(&dev);
    bma2_error_codes_print_result("bma2_init", rslt);
    printf("Chip id : 0x%x\n", dev.chip_id);

    rslt = bma2_set_power_mode(BMA2_NORMAL_MODE, &dev);
    bma2_error_codes_print_result("bma2_set_power_mode", rslt);

    rslt = bma2_get_power_mode(&power_mode, &dev);
    bma2_error_codes_print_result("bma2_get_power_mode", rslt);

    printf("Power mode set is : %d \n", power_mode);

    rslt = bma2_get_accel_conf(&accel_conf, &dev);
    bma2_error_codes_print_result("bma2_get_accel_conf", rslt);

    /*
     * NOTE : Fast offset compensation must be done at range 2G
     */
    accel_conf.range = BMA2_ACC_RANGE_2G;
    accel_conf.bw = BMA2_ACC_BW_1000_HZ;

    rslt = bma2_set_accel_conf(&accel_conf, &dev);
    bma2_error_codes_print_result("bma2_set_accel_conf", rslt);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_DATA_READY, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    rslt = bma2_set_int_src(BMA2_INT_UNFILTERED_DATA, BMA2_INT_SRC_DATA, &dev);
    bma2_error_codes_print_result("bma2_set_int_src", rslt);

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_DATA_READY, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    rslt = bma2_get_int_mapping(&int_map, &dev);
    bma2_error_codes_print_result("bma2_get_int_mapping", rslt);

    printf("\n\nAccel data collected at 2G Range (Before FOC)\n\n");

    while (idx < OFFSET_SAMPLE_COUNT)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_1 & BMA2_INT_1_ASSERTED_DATA_READY)
        {
            rslt = bma2_get_accel_data(&accel, &dev);
            bma2_error_codes_print_result("bma2_get_accel_data", rslt);

            if (rslt == BMA2_OK)
            {
                /* Converting lsb to g for respective bit resolution at 2G range */
                x = lsb_to_g(accel.x, 2, dev.resolution);
                y = lsb_to_g(accel.y, 2, dev.resolution);
                z = lsb_to_g(accel.z, 2, dev.resolution);

                printf(
                    "Accel[%d] : Raw_Accel_X : %d, Raw_Accel_Y : %d, Raw_Accel_Z : %d  Acc_G_X = %4.2f, Acc_G_Y = %4.2f, Acc_G_Z = %4.2f\n",
                    idx,
                    accel.x,
                    accel.y,
                    accel.z,
                    x,
                    y,
                    z);

                idx++;

                int_status.int_status_1 = 0;
            }
        }
    }

    printf("\nRead Offset data (Before FOC)\n\n");

    rslt = bma2_get_offset_data(&offset_data, &dev);
    bma2_error_codes_print_result("bma2_get_offset_data", rslt);

    printf("offset_x_data : %d\n", offset_data.offset_x_data);
    printf("offset_y_data : %d\n", offset_data.offset_y_data);
    printf("offset_z_data : %d\n", offset_data.offset_z_data);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for X-Axis with offset target 0g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_X_AXIS;
    foc.offset_target_x = BMA2_OFFSET_TARGET_0G_VAL1;

    rslt = perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for X-Axis with offset target +1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_X_AXIS;
    foc.offset_target_x = BMA2_OFFSET_TARGET_POS_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for X-Axis with offset target -1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_X_AXIS;
    foc.offset_target_x = BMA2_OFFSET_TARGET_NEG_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Y-Axis with offset target 0g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Y_AXIS;
    foc.offset_target_y = BMA2_OFFSET_TARGET_0G_VAL1;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Y-Axis with offset target +1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Y_AXIS;
    foc.offset_target_y = BMA2_OFFSET_TARGET_POS_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Y-Axis with offset target -1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Y_AXIS;
    foc.offset_target_y = BMA2_OFFSET_TARGET_NEG_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Z-Axis with offset target 0g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Z_AXIS;
    foc.offset_target_z = BMA2_OFFSET_TARGET_0G_VAL1;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Z-Axis with offset target +1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Z_AXIS;
    foc.offset_target_z = BMA2_OFFSET_TARGET_POS_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    printf("\n ******************************************************\n");
    printf("     Perform FOC for Z-Axis with offset target -1g\n");
    printf("******************************************************\n");

    cal_trigger = BMA2_OFFSET_Z_AXIS;
    foc.offset_target_z = BMA2_OFFSET_TARGET_NEG_1G;

    perform_foc_test(cal_trigger, foc, &dev);
    bma2_error_codes_print_result("perform_foc_test", rslt);

    bma2_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API Performs FOC
 */
static int8_t perform_foc_test(uint8_t cal_trigger, struct bma2_fast_slow_offset foc, struct bma2_dev *dev)
{
    int8_t rslt;
    uint8_t idx = 0;
    struct bma2_int_status int_status = { 0 };
    struct bma2_sensor_data accel;
    float x = 0, y = 0, z = 0;
    struct bma2_offset_data offset_data;

    rslt = bma2_fast_slow_offset_compensation(&cal_trigger, &foc, dev);
    bma2_error_codes_print_result("bma2_fast_slow_offset_compensation", rslt);

    printf("\n#\n# Accel data collected at 2G Range (After FOC)\n\n");

    while (idx < OFFSET_SAMPLE_COUNT)
    {
        rslt = bma2_get_int_status(&int_status, dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_1 & BMA2_INT_1_ASSERTED_DATA_READY)
        {
            rslt = bma2_get_accel_data(&accel, dev);
            bma2_error_codes_print_result("bma2_get_accel_data", rslt);

            if (rslt == BMA2_OK)
            {
                /* Converting lsb to g for respective bit resolution at 2G range */
                x = lsb_to_g(accel.x, 2, dev->resolution);
                y = lsb_to_g(accel.y, 2, dev->resolution);
                z = lsb_to_g(accel.z, 2, dev->resolution);

                printf(
                    "Accel[%d] : Raw_Accel_X : %d, Raw_Accel_Y : %d, Raw_Accel_Z : %d  Acc_G_X = %4.2f, Acc_G_Y = %4.2f, Acc_G_Z = %4.2f\n",
                    idx,
                    accel.x,
                    accel.y,
                    accel.z,
                    x,
                    y,
                    z);

                idx++;

                int_status.int_status_1 = 0;
            }
        }
    }

    printf("\nRead Offset data (After FOC)\n\n");

    rslt = bma2_get_offset_data(&offset_data, dev);
    bma2_error_codes_print_result("bma2_get_offset_data", rslt);

    printf("offset_x_data : %d\n", offset_data.offset_x_data);
    printf("offset_y_data : %d\n", offset_data.offset_y_data);
    printf("offset_z_data : %d\n", offset_data.offset_z_data);

    printf("\n\nReset Offset\n");
    rslt = bma2_reset_offset(dev);
    bma2_error_codes_print_result("bma2_reset_offset", rslt);

    printf("\nRead Offset data (After reset)\n\n");

    rslt = bma2_get_offset_data(&offset_data, dev);
    bma2_error_codes_print_result("bma2_get_offset_data", rslt);

    printf("offset_x_data : %d\n", offset_data.offset_x_data);
    printf("offset_y_data : %d\n", offset_data.offset_y_data);
    printf("offset_z_data : %d\n", offset_data.offset_z_data);

    printf("\n\nAccel data collected at 2G Range (After Offset Reset)\n\n");

    idx = 0;

    while (idx < OFFSET_SAMPLE_COUNT)
    {
        rslt = bma2_get_int_status(&int_status, dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_1 & BMA2_INT_1_ASSERTED_DATA_READY)
        {
            rslt = bma2_get_accel_data(&accel, dev);
            bma2_error_codes_print_result("bma2_get_accel_data", rslt);

            if (rslt == BMA2_OK)
            {
                /* Converting lsb to g for respective bit resolution at 2G range */
                x = lsb_to_g(accel.x, 2, dev->resolution);
                y = lsb_to_g(accel.y, 2, dev->resolution);
                z = lsb_to_g(accel.z, 2, dev->resolution);

                printf(
                    "Accel[%d] : Raw_Accel_X : %d, Raw_Accel_Y : %d, Raw_Accel_Z : %d  Acc_G_X = %4.2f, Acc_G_Y = %4.2f, Acc_G_Z = %4.2f\n",
                    idx,
                    accel.x,
                    accel.y,
                    accel.z,
                    x,
                    y,
                    z);

                idx++;

                int_status.int_status_1 = 0;
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to g.
 */
static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (val * g_range) / half_scale;
}
