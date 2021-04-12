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
    int8_t rslt = 0, loop = 0;
    struct bma2_dev dev = { 0 };
    uint32_t int_en;
    uint8_t power_mode;
    struct bma2_int_status int_status;
    struct bma2_int_pin pin_conf;
    struct bma2_tap_conf tap_sett, cur_tap_set;

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

    rslt = bma2_get_tap_conf(&tap_sett, &dev);
    bma2_error_codes_print_result("bma2_get_tap_conf", rslt);

    tap_sett.duration = BMA2_500_MS_TAP_DURN;
    tap_sett.threshold = 0x05;
    tap_sett.quiet = BMA2_20_MS_TAP_QUIET_DURN;
    tap_sett.shock = BMA2_75_MS_TAP_SHOCK_DURN;

    rslt = bma2_set_tap_conf(&tap_sett, &dev);
    bma2_error_codes_print_result("bma2_set_tap_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("Tap duration    : %d\n", tap_sett.duration);
    printf("Tap threshold   : %d\n", tap_sett.threshold);
    printf("Tap quiet       : %d\n", tap_sett.quiet);
    printf("Tap sample      : %d\n", tap_sett.sample);
    printf("Tap shock       : %d\n", tap_sett.shock);

    rslt = bma2_get_tap_conf(&cur_tap_set, &dev);
    bma2_error_codes_print_result("bma2_get_tap_conf", rslt);

    int_en = BMA2_INT_EN_SINGLE_TAP | BMA2_INT_EN_DOUBLE_TAP;

    rslt = bma2_enable_interrupt(int_en, &dev);
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

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, (BMA2_INT1_MAP_SINGLE_TAP | BMA2_INT1_MAP_DOUBLE_TAP), &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    printf("\nTap the board\n");

    while (loop < 10)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_SINGLE_TAP)
        {
            printf("\nSingle tap interrupt asserted\n");

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_X)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Single tap interrupt triggered on X negative axis\n");
                }
                else
                {
                    printf("Single tap interrupt triggered on X positive axis\n");
                }
            }

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_Y)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Single tap interrupt triggered on Y negative axis\n");
                }
                else
                {
                    printf("Single tap interrupt triggered on Y positive axis\n");
                }
            }

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_Z)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Single tap interrupt triggered on Z negative axis\n");
                }
                else
                {
                    printf("Single tap interrupt triggered on Z positive axis\n");
                }
            }

            loop++;
        }

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_DOUBLE_TAP)
        {
            printf("\nDouble tap interrupt asserted\n");

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_X)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Double tap interrupt triggered on X negative axis\n");
                }
                else
                {
                    printf("Double tap interrupt triggered on X positive axis\n");
                }
            }

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_Y)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Double tap interrupt triggered on Y negative axis\n");
                }
                else
                {
                    printf("Double tap interrupt triggered on Y positive axis\n");
                }
            }

            if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_Z)
            {
                if (int_status.int_status_2 & BMA2_INT_2_ASSERTED_TAP_SIGN)
                {
                    printf("Double tap interrupt triggered on Z negative axis\n");
                }
                else
                {
                    printf("Double tap interrupt triggered on Z positive axis\n");
                }
            }

            loop++;
        }
    }

    bma2_coines_deinit();

    return rslt;
}
