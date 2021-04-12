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
int main(void)
{
    int8_t rslt = 0;
    struct bma2_dev dev = { 0 };
    uint32_t int_en;
    uint8_t power_mode;
    struct bma2_int_status int_status;
    struct bma2_int_pin pin_conf;
    struct bma2_slo_no_mot_conf no_mot_settin, no_mot_cur_settings;

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

    rslt = bma2_get_slo_no_mot_conf(&no_mot_settin, &dev);
    bma2_error_codes_print_result("bma2_get_slo_no_mot_conf", rslt);

    no_mot_settin.duration = 0x01;
    no_mot_settin.threshold = 0x64;

    rslt = bma2_set_slo_no_mot_conf(&no_mot_settin, &dev);
    bma2_error_codes_print_result("bma2_set_slo_no_mot_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("Slow/no-motion duration    : %d\n", no_mot_settin.duration);
    printf("Slow/no-motion threshold   : %d\n", no_mot_settin.threshold);

    rslt = bma2_get_slo_no_mot_conf(&no_mot_cur_settings, &dev);
    bma2_error_codes_print_result("bma2_get_slo_no_mot_conf", rslt);

    rslt = bma2_get_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_get_int_out_ctrl", rslt);

    pin_conf.latch_int = BMA2_NON_LATCHED;
    pin_conf.int1_lvl = BMA2_ACTIVE_HIGH;
    pin_conf.int1_od = BMA2_PUSH_PULL;

    rslt = bma2_set_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_set_int_out_ctrl", rslt);

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_SLOW_NO_MOTION, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    printf("\nEnable slow motion\n");

    int_en = BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS;

    rslt = bma2_enable_interrupt(int_en, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS)
    {
        printf("Slow motion x axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS)
    {
        printf("Slow motion y axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS)
    {
        printf("Slow motion z axes - enabled\n");
    }

    printf("Move the board slowly\n");

    while (1)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_SLOW_NO_MOTION)
        {
            printf("Slow motion test success\n");
            break;
        }
    }

    printf("\nEnable no motion\n");

    int_en = BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS | BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS |
             BMA2_INT_EN_SLOW_NO_MOTION_SEL;

    rslt = bma2_enable_interrupt(int_en, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_X_AXIS)
    {
        printf("No motion x axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_Y_AXIS)
    {
        printf("No motion y axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_SLOW_NO_MOTION_Z_AXIS)
    {
        printf("No motion z axes - enabled\n");
    }

    printf("Do not move the board\n");

    while (1)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_SLOW_NO_MOTION)
        {
            printf("No motion test success\n");
            break;
        }
    }

    bma2_coines_deinit();

    return rslt;
}
