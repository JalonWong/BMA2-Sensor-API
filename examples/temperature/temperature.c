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
    uint16_t counter = 1;
    uint8_t power_mode;

    /* Value of temperature is returned to this variable */
    float temperature_data;

    /* Variable to store temperature raw data */
    uint8_t temp_raw_data;
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

    accel_conf.range = BMA2_ACC_RANGE_4G;

    rslt = bma2_set_accel_conf(&accel_conf, &dev);
    bma2_error_codes_print_result("bma2_set_accel_conf", rslt);

    rslt = bma2_set_power_mode(BMA2_NORMAL_MODE, &dev);
    bma2_error_codes_print_result("bma2_set_power_mode", rslt);

    rslt = bma2_get_power_mode(&power_mode, &dev);
    bma2_error_codes_print_result("bma2_get_power_mode", rslt);

    printf("Power mode set is : %d \n", power_mode);

    while (counter <= 200)
    {
        /*
         * Delay to read temperature data and observe varying values
         */
        dev.delay_us(6000, dev.intf_ptr);
        rslt = bma2_get_temperature_data(&temp_raw_data, &dev);
        bma2_error_codes_print_result("bma2_get_temperature_data", rslt);

        /* The slope of temperature as per data-sheet is 0.5K/LSB
         * And the center temperature is 23'C
         */
        temperature_data = (float)(((int8_t)temp_raw_data) * 0.5 + 23);

        printf("Temperature [%d] : %.1f deg C\n", counter, temperature_data);
        counter++;
    }

    bma2_coines_deinit();

    return rslt;
}
