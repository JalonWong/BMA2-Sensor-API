/*
 * Copyright (C) 2020 Bosch Sensortec GmbH
 *
 * The license is available at root folder
 *
 */

#include <stdio.h>
#include <math.h>

#include "bma2.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! PI Value */
#define PI  (3.141592654f)

/******************************************************************************/
int main(void)
{
    int8_t rslt = 0;
    struct bma2_dev dev = { 0 };
    uint32_t int_en;
    uint8_t power_mode;
    double flat_angle = 0.0;
    struct bma2_int_status int_status;
    struct bma2_int_pin pin_conf;
    struct bma2_flat_conf flat_sett, flat_curr_set;

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

    rslt = bma2_get_flat_conf(&flat_sett, &dev);
    bma2_error_codes_print_result("bma2_get_flat_conf", rslt);

    flat_sett.theta = 0x05;
    flat_sett.hold_time = BMA2_FLAT_DELAY_0_MS;
    flat_sett.hysteresis = 0x04;

    rslt = bma2_set_flat_conf(&flat_sett, &dev);
    bma2_error_codes_print_result("bma2_set_flat_conf", rslt);

    printf("Sensor settings which are set\n");
    printf("Theta        : %d\n", flat_sett.theta);
    printf("Hold time    : %d\n", flat_sett.hold_time);
    printf("Hysteresis   : %d\n", flat_sett.hysteresis);

    flat_angle = atan((double)(0.125 * (sqrt(flat_sett.theta))));
    printf("Flat angle in degrees : %.2f \n", ((flat_angle * 180) / PI));

    rslt = bma2_get_flat_conf(&flat_curr_set, &dev);
    bma2_error_codes_print_result("bma2_get_flat_conf", rslt);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_FLAT, &dev);
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

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_FLAT, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    printf("\nKeep the board flat\n");

    while (1)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_0 & BMA2_INT_0_ASSERTED_FLAT)
        {
            printf("Flat interrupt occured\n");

            if (int_status.int_status_3 & BMA2_INT_3_ASSERTED_FLAT_POSITION)
            {
                printf("Flat condition full-filled\n");
                printf("Flat test success\n");
                break;
            }
        }
    }

    bma2_coines_deinit();

    return rslt;
}
