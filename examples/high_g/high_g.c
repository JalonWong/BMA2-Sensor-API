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
    int8_t count_x = 0, count_y = 0, count_z = 0;
    struct bma2_dev dev = { 0 };
    uint32_t int_en;
    uint8_t power_mode;
    struct bma2_int_status int_status;
    struct bma2_int_pin pin_conf;
    struct bma2_high_g_conf high_g_settin, cur_high_g_sett;

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

    rslt = bma2_get_high_g_conf(&high_g_settin, &dev);
    bma2_error_codes_print_result("bma2_get_high_g_conf", rslt);

    high_g_settin.duration = 0x64;
    high_g_settin.hysteresis = 0x03;
    high_g_settin.threshold = 0x90;

    rslt = bma2_set_high_g_conf(&high_g_settin, &dev);
    bma2_error_codes_print_result("bma2_set_high_g_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("High-g duration   : %d\n", high_g_settin.duration);
    printf("High-g threshold  : %d\n", high_g_settin.threshold);
    printf("High-g hysteresis : %d\n", high_g_settin.hysteresis);

    rslt = bma2_get_high_g_conf(&cur_high_g_sett, &dev);
    bma2_error_codes_print_result("bma2_get_high_g_conf", rslt);

    int_en = BMA2_INT_EN_HIGH_G_X_AXIS | BMA2_INT_EN_HIGH_G_Y_AXIS | BMA2_INT_EN_HIGH_G_Z_AXIS;

    rslt = bma2_enable_interrupt(int_en, &dev);
    bma2_error_codes_print_result("bma2_enable_interrupt", rslt);

    rslt = bma2_get_enabled_interrupts(&int_en, &dev);
    bma2_error_codes_print_result("bma2_get_enabled_interrupts", rslt);

    if (int_en & BMA2_INT_EN_HIGH_G_X_AXIS)
    {
        printf("High g x axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_HIGH_G_Y_AXIS)
    {
        printf("High g y axes - enabled\n");
    }

    if (int_en & BMA2_INT_EN_HIGH_G_Z_AXIS)
    {
        printf("High g z axes - enabled\n");
    }

    rslt = bma2_get_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_get_int_out_ctrl", rslt);

    pin_conf.latch_int = BMA2_NON_LATCHED;
    pin_conf.int1_lvl = BMA2_ACTIVE_HIGH;
    pin_conf.int1_od = BMA2_PUSH_PULL;

    rslt = bma2_set_int_out_ctrl(&pin_conf, &dev);
    bma2_error_codes_print_result("bma2_set_int_out_ctrl", rslt);

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_HIGH_G, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    printf("\nMove the board in reverse gravity for high-g \n\n");

    while (1)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_HIGH_G)
        {
            if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_X)
            {
                if (count_x == 0)
                {
                    if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_SIGN)
                    {
                        printf("High G interrupt triggered on X negative axis\n");
                    }
                    else
                    {
                        printf("High G interrupt triggered on X positive axis\n");
                    }

                    count_x = 1;
                }
            }

            if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_Y)
            {
                if (count_y == 0)
                {
                    if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_SIGN)
                    {
                        printf("High G interrupt triggered on Y negative axis\n");
                    }
                    else
                    {
                        printf("High G interrupt triggered on Y positive axis\n");
                    }

                    count_y = 1;
                }
            }

            if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_Z)
            {
                if (count_z == 0)
                {
                    if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_HIGH_SIGN)
                    {
                        printf("High G interrupt triggered on Z negative axis\n");
                    }
                    else
                    {
                        printf("High G interrupt triggered on Z positive axis\n");
                    }

                    count_z = 1;
                }
            }

            if ((count_x == 1) && (count_y == 1) && (count_z == 1))
            {
                printf("\nHigh-G interrupt testing done. Exiting !\n");
                break;
            }
        }
    }

    bma2_coines_deinit();

    return rslt;
}
