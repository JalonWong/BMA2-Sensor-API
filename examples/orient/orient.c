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
    int8_t mode, block;
    struct bma2_dev dev = { 0 };
    uint32_t int_en;
    uint8_t power_mode;
    uint8_t int_stat;
    struct bma2_int_status int_status;
    struct bma2_int_pin pin_conf;
    struct bma2_orient_conf orient_sett, orient_curr_set;

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

    rslt = bma2_get_orient_conf(&orient_sett, &dev);
    bma2_error_codes_print_result("bma2_get_orient_conf", rslt);

    orient_sett.blocking = BMA2_THETA_BLOCK_MODE_2;
    orient_sett.hysteresis = 0x05;
    orient_sett.theta = 0x1C;
    orient_sett.ud_enable = BMA2_ENABLE;

    rslt = bma2_set_orient_conf(&orient_sett, &dev);
    bma2_error_codes_print_result("bma2_set_orient_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("Orientation blocking     : %d\n", orient_sett.blocking);
    printf("Orientation hysteresis   : %d\n", orient_sett.hysteresis);
    printf("Orientation mode         : %d\n", orient_sett.mode);
    printf("Orientation theta        : %d\n", orient_sett.theta);
    printf("Orientation ud_enable    : %d\n", orient_sett.ud_enable);

    rslt = bma2_get_orient_conf(&orient_curr_set, &dev);
    bma2_error_codes_print_result("bma2_get_orient_conf", rslt);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_ORIENTATION, &dev);
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

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_ORIENTATION, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    /* For different modes */
    printf("\nFor different modes: symm(value : 0), high asymm(value : 1), low asymm(value : 2)\n");

    for (mode = 0; mode <= BMA2_ORIENT_LOW_SYMMETRICAL; mode++)
    {
        /* Configure orientation mode */
        orient_sett.mode = mode;

        /* Set the configurations */
        rslt = bma2_set_orient_conf(&orient_sett, &dev);
        bma2_error_codes_print_result("bma2_set_orient_conf", rslt);

        printf("\nOrientation mode   : %d\n", orient_sett.mode);

        printf("Move the board for orientation:\n");

        while (1)
        {
            /* Check the interrupt status of the orientation */
            rslt = bma2_get_int_status(&int_status, &dev);
            bma2_error_codes_print_result("bma2_get_int_status", rslt);

            if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_ORIENTATION)
            {
                printf("Orientation interrupt detected\n");

                int_stat = BMA2_GET_BITS(int_status.int_status_3, BMA2_ORIENT_STATUS);

                if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_ORIENTATION_Z)
                {
                    printf("Z axes looking down\n");
                }
                else
                {
                    printf("Z axes looking up\n");
                }

                if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_POTRAIT_UPSIDE_DOWN)
                {
                    printf("Portrait up side down\n");
                }
                else if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_LANDSCAPE_LEFT)
                {
                    printf("Landscape left\n");
                }
                else if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_LANDSCAPE_RIGHT)
                {
                    printf("Landscape right\n");
                }
                else
                {
                    printf("Portrait upright\n");
                }

                printf("Orient func test success\n");
                break;
            }
        }
    }

    /* For different blocking conditions */
    printf("\nFor different blocking conditions: no blocking(value : 0), theta blocking mode 1(value : 1),\n");
    printf("theta blocking mode 2(value : 2), theta blocking mode 3(value : 3)\n");

    for (block = 0; block <= BMA2_THETA_BLOCK_MODE_3; block++)
    {
        printf("\nFor blocking condition = %d\n", block);

        /* Configure orientation blocking modes */
        orient_sett.blocking = block;

        printf("Orientation blocking  : %d\n", orient_sett.blocking);

        /* Set the configurations */
        rslt = bma2_set_orient_conf(&orient_sett, &dev);
        bma2_error_codes_print_result("bma2_set_orient_conf", rslt);

        printf("Move the board for orientation:\n");

        while (1)
        {
            /* Check the interrupt status of the orientation */
            rslt = bma2_get_int_status(&int_status, &dev);
            bma2_error_codes_print_result("bma2_get_int_status", rslt);

            if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_ORIENTATION)
            {
                printf("Orientation interrupt detected\n");

                int_stat = BMA2_GET_BITS(int_status.int_status_3, BMA2_ORIENT_STATUS);

                if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_ORIENTATION_Z)
                {
                    printf("Z axes looking down\n");
                }
                else
                {
                    printf("Z axes looking up\n");
                }

                if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_POTRAIT_UPSIDE_DOWN)
                {
                    printf("Portrait up side down\n");
                }
                else if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_LANDSCAPE_LEFT)
                {
                    printf("Landscape left\n");
                }
                else if (int_stat == BMA2_INT_3_ASSERTED_ORIENTATION_LANDSCAPE_RIGHT)
                {
                    printf("Landscape right\n");
                }
                else
                {
                    printf("Portrait upright\n");
                }

                printf("Orient func test success\n");
                break;
            }
        }
    }

    bma2_coines_deinit();

    return rslt;
}
