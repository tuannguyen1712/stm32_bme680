/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bme68x.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "common.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID  0x93
#define I2C_TIMEOUT 	   200

uint8_t tx1[100];

I2C_HandleTypeDef hi2c;
TIM_HandleTypeDef htim;
UART_HandleTypeDef huart;

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;
    if (HAL_I2C_Master_Transmit(&hi2c, 0x76 , &reg_addr, 1, I2C_TIMEOUT) != HAL_OK) {
    	return - 1;
    }
//    return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
    if (HAL_I2C_Master_Receive(&hi2c, 0x76 , reg_data, len, I2C_TIMEOUT) != HAL_OK) {
    	return -1;
    }
    else
    	return 0;
}

/*!
 * I2C write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    uint8_t info[len + 1];
	info[0] = reg_addr;
	for (int i = 0; i < len; i++) {
		info[i + 1] = reg_data[i];
	}

    (void)intf_ptr;

//    return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
    if (HAL_I2C_Master_Transmit(&hi2c, 0x76 , info, len + 1, I2C_TIMEOUT) != HAL_OK) {
    	return -1;
    }
    else
    	return 0;
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BME68X_OK:
        	sprintf((char*) tx1, "OK\n");
			HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
			break;
        case BME68X_E_NULL_PTR:
            sprintf((char*) tx1, "API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
            HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        case BME68X_E_COM_FAIL:
        	sprintf((char*) tx1, "API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        case BME68X_E_INVALID_LENGTH:
        	sprintf((char*) tx1, "API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        case BME68X_E_DEV_NOT_FOUND:
        	sprintf((char*) tx1, "API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        case BME68X_E_SELF_TEST:
        	sprintf((char*) tx1, "API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        case BME68X_W_NO_NEW_DATA:
        	sprintf((char*) tx1, "API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
        default:
        	sprintf((char*) tx1, "API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
        	HAL_UART_Transmit(&huart, tx1, strlen((char*) tx1), 200);
            break;
    }
}

void bme68x_delay_us(uint32_t us, void *intf_ptr) {
	(void)intf_ptr;
	__HAL_TIM_SetCounter(&htim, 0);
	while (__HAL_TIM_GetCounter(&htim) < us);
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf, I2C_HandleTypeDef hi2cx, TIM_HandleTypeDef htimx, UART_HandleTypeDef huartx)
{

    int8_t rslt = BME68X_OK;
    hi2c = hi2cx;
    htim = htimx;
    huart = huartx;
    if (bme != NULL)
    {
        /* Bus configuration : I2C */
		dev_addr = BME68X_I2C_ADDR_LOW;
		bme->read = bme68x_i2c_read;
		bme->write = bme68x_i2c_write;
		bme->intf = BME68X_I2C_INTF;
		HAL_Delay(200);
        bme->delay_us = bme68x_delay_us;
        bme->intf_ptr = &dev_addr;
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    return rslt;
}

//void bme68x_coines_deinit(void)
//{
//    (void)fflush(stdout);
//
//    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//    coines_delay_msec(1000);
//
//    /* Coines interface reset */
//    coines_soft_reset();
//    coines_delay_msec(1000);
//    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
//}
