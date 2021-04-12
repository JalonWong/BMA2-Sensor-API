/**
 * Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "coines.h"
#include "bma2_defs.h"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMA2_INTF_RET_TYPE bma2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_i2c(dev_addr, reg_addr, reg_data, (uint16_t)length);
}

/*!
 * I2C write function map to COINES platform
 */
BMA2_INTF_RET_TYPE bma2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_i2c(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

/*!
 * SPI read function map to COINES platform
 */
BMA2_INTF_RET_TYPE bma2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return coines_read_spi(dev_addr, reg_addr, reg_data, (uint16_t)length);
}

/*!
 * SPI write function map to COINES platform
 */
BMA2_INTF_RET_TYPE bma2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    dev_addr = *(uint8_t*)intf_ptr;

    return coines_write_spi(dev_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

/*!
 * Delay function map to COINES platform
 */
void bma2_delay_us(uint32_t period, void *intf_ptr)
{
    coines_delay_usec(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bma2_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMA2_OK)
    {
        printf("API name %s\t", api_name);

        if (rslt == BMA2_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMA2_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMA2_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMA2_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode configuration\r\n", rslt);
        }
        else if (rslt == BMA2_E_RW_LEN_INVALID)
        {
            printf("Error [%d] : Invalid read write length\r\n", rslt);
        }
        else if (rslt == BMA2_E_INVALID_CONFIG)
        {
            printf("Error [%d] : Invalid configuration\r\n", rslt);
        }
        else if (rslt == BMA2_W_NO_NEW_AVAILABLE)
        {
            printf("Warning [%d] : Non-availability of data\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bma2_interface_init(struct bma2_dev *dev, uint8_t intf)
{
    int8_t rslt = BMA2_OK;
    struct coines_board_info board_info;

    if (dev != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB);

        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        /* Get board info to initialize the APP2.0 compatible pins in APP3.0 board */
        rslt = coines_get_board_info(&board_info);

        coines_set_shuttleboard_vdd_vddio_config(0, 0);
        coines_delay_msec(100);

        /* Bus configuration : I2C */
        if (intf == BMA2_I2C_INTF)
        {
            printf("I2C Interface\n");

            dev_addr = BMA2_I2C_ADDR1;
            dev->read = bma2_i2c_read;
            dev->write = bma2_i2c_write;
            dev->intf = BMA2_I2C_INTF;

            coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
            coines_delay_usec(500);

            /* PS pin is made high for selecting I2C protocol */
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
        }
        /* Bus configuration : SPI */
        else if (intf == BMA2_SPI_INTF)
        {
            printf("SPI Interface\n");

            dev_addr = COINES_SHUTTLE_PIN_7;
            dev->read = bma2_spi_read;
            dev->write = bma2_spi_write;
            dev->intf = BMA2_SPI_INTF;
            coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
            coines_set_pin_config(COINES_SHUTTLE_PIN_9, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
        }

        coines_delay_msec(100);

        coines_set_shuttleboard_vdd_vddio_config(3300, 3300);

        coines_delay_msec(100);

        /* Holds the I2C device addr or SPI chip selection */
        dev->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        dev->delay_us = bma2_delay_us;
    }
    else
    {
        rslt = BMA2_E_NULL_PTR;
    }

    return rslt;
}

void bma2_coines_deinit(void)
{
    fflush(stdout);

    coines_set_shuttleboard_vdd_vddio_config(0, 0);
    coines_delay_msec(1000);

    /* Coines interface reset */
    coines_soft_reset();
    coines_delay_msec(1000);

    coines_close_comm_intf(COINES_COMM_INTF_USB);
}
