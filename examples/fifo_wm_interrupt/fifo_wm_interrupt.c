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

/*! Watermark level for FIFO */
#define BMA2_FIFO_WM_LEVEL                       UINT8_C(16)

/*! Number of Accel frames to be extracted from FIFO */
#define BMA2_FIFO_EXTRACTED_DATA_FRAME_COUNT     UINT8_C(32)

/*! Number of Accel frames to be extracted from FIFO */
#define BMA2_FIFO_WM_EXTRACTED_DATA_FRAME_COUNT  UINT8_C(16)

/******************************************************************************/
int main(void)
{
    int8_t rslt = 0;
    struct bma2_dev dev = { 0 };
    uint8_t idx;
    uint8_t try = 0;
    uint32_t int_en;
    uint8_t power_mode;
    uint16_t acc_index;
    struct bma2_int_pin pin_conf;
    struct bma2_acc_conf get_accel_conf;
    struct bma2_acc_conf set_accel_conf;
    struct bma2_int_status int_status;
    struct bma2_fifo_frame fifo, get_fifo;
    uint8_t fifo_buff[BMA2_FIFO_BUFFER] = { 0 };
    struct bma2_sensor_data accel_data[BMA2_FIFO_EXTRACTED_DATA_FRAME_COUNT] = { { 0 } };

    /* Interface reference is given as a parameter
     *         For I2C : BMA2_I2C_INTF
     *         For SPI : BMA2_SPI_INTF
     */
    rslt = bma2_interface_init(&dev, BMA2_I2C_INTF);
    bma2_error_codes_print_result("bma2_interface_init", rslt);

    rslt = bma2_init(&dev);
    bma2_error_codes_print_result("bma2_init", rslt);
    printf("Chip id : 0x%x\n", dev.chip_id);

    rslt = bma2_get_accel_conf(&get_accel_conf, &dev);
    bma2_error_codes_print_result("bma2_get_accel_conf", rslt);

    set_accel_conf.range = BMA2_ACC_RANGE_2G;

    rslt = bma2_set_accel_conf(&set_accel_conf, &dev);
    bma2_error_codes_print_result("bma2_set_accel_conf", rslt);

    rslt = bma2_get_accel_conf(&get_accel_conf, &dev);
    bma2_error_codes_print_result("bma2_get_accel_conf", rslt);

    rslt = bma2_get_fifo_config(&fifo, &dev);
    bma2_error_codes_print_result("bma2_get_fifo_config", rslt);

    /* Data selection can be used to select data axis,
     * all XYZ axes - BMA2_XYZ_AXES,
     * X axis only - BMA2_X_AXIS,
     * Y axis only - BMA2_Y_AXIS,
     * Z axis only - BMA2_Z_AXIS,
     * Here, all XYZ axes data is selected(BMA2_XYZ_AXES)
     */
    fifo.fifo_data_select = BMA2_XYZ_AXES;

    fifo.fifo_mode_select = BMA2_MODE_FIFO;
    fifo.wm_level = BMA2_FIFO_WM_LEVEL;

    rslt = bma2_set_fifo_config(&fifo, &dev);
    bma2_error_codes_print_result("bma2_set_fifo_config", rslt);

    rslt = bma2_get_fifo_config(&get_fifo, &dev);
    bma2_error_codes_print_result("bma2_get_fifo_config", rslt);

    rslt = bma2_enable_interrupt(BMA2_INT_EN_FIFO_WM, &dev);
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

    rslt = bma2_set_int_mapping(BMA2_INT_MAP, BMA2_INT1_MAP_FIFO_WM, &dev);
    bma2_error_codes_print_result("bma2_set_int_mapping", rslt);

    /* Fifo user buffer and length config */
    fifo.data = fifo_buff;
    fifo.length = BMA2_FIFO_BUFFER;

    rslt = bma2_set_power_mode(BMA2_NORMAL_MODE, &dev);
    bma2_error_codes_print_result("bma2_set_power_mode", rslt);

    rslt = bma2_get_power_mode(&power_mode, &dev);
    bma2_error_codes_print_result("bma2_get_power_mode", rslt);

    printf("Power mode set is : %d \n", power_mode);

    while (try < 10)
    {
        rslt = bma2_get_int_status(&int_status, &dev);
        bma2_error_codes_print_result("bma2_get_int_status", rslt);

        if (int_status.int_status_1 & BMA2_INT_1_ASSERTED_FIFO_WM)
        {
            printf("\nIteration: %d\n", try);

            /* Read fifo data */
            rslt = bma2_read_fifo_data(&fifo, &dev);
            bma2_error_codes_print_result("bma2_read_fifo_data", rslt);

            if (rslt == BMA2_OK)
            {
                printf("FIFO data bytes available : %d\n", fifo.length);
                printf("FIFO watermark level      : %d (In Frames)\n", fifo.wm_level);
                printf("FIFO frames available     : %d (6 bytes for each frame as XYZ-axes are enabled)\n",
                       fifo.fifo_frame_count);

                rslt = bma2_extract_accel(accel_data, &acc_index, &fifo);
                bma2_error_codes_print_result("bma2_extract_accel", rslt);

                printf("FIFO frames valid       : %d\n", acc_index);

                /* Print the accel data frames */
                for (idx = 0; idx < acc_index; idx++)
                {
                    printf("Accel[%d] X : %d   Y : %d   Z : %d\n",
                           idx,
                           accel_data[idx].x,
                           accel_data[idx].y,
                           accel_data[idx].z);
                }

                try++;
            }
        }
    }

    bma2_coines_deinit();

    return rslt;
}
