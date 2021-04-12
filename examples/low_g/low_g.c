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
    struct bma2_low_g_conf low_g_setting, low_g_cur_set;

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

    rslt = bma2_get_low_g_conf(&low_g_setting, &dev);
    bma2_error_codes_print_result("bma2_get_low_g_conf", rslt);

    low_g_setting.duration = 0x01;
    low_g_setting.hysteresis = 0x02;
    low_g_setting.mode = BMA2_AXIS_SUMMING_MODE;
    low_g_setting.threshold = 0x30;

    rslt = bma2_set_low_g_conf(&low_g_setting, &dev);
    bma2_error_codes_print_result("bma2_set_low_g_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("Low g duration   : %d\n", low_g_setting.duration);
    printf("Low g hyst       : %d\n", low_g_setting.hysteresis);
    printf("Low g mode       : %d\n", low_g_setting.mode);
    printf("Low g threshold  : %d\n", low_g_setting.threshold);

    rslt = bma2_get_low_g_conf(&low_g_cur_set, &dev);
    bma2_error_codes_print_result("bma2_get_low_g_conf", rslt);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_LOW_G, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    rslt = bma2_get_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_get_int_out_ctrl", rslt);

    pin_conf.latch_int = BMA2_NON_LATCHED;
    pin_conf.int1_lvl = BMA2_ACTIVE_HIGH;
    pin_conf.int1_od = BMA2_PUSH_PULL;

    rslt = bma2_set_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_set_int_out_ctrl", rslt);

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_LOW_G, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    printf("Drop the board for low_g interrupt\n");

    while (1)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_LOW_G)
        {
            printf("Low-g interrupt detected\n");
            printf("Low_g test success\n");
            break;
        }
    }

    bma2_coines_deinit();

    return rslt;
}
