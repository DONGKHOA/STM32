/*
 * sx_1278.c
 *
 *  Created on: Sep 21, 2023
 *      Author: Khoa dong
 */

#include "sx_1278_hw.h"
#include "main.h"

#define TIME_OUT 1000

/**
 * The function initializes the SX1278 hardware by setting the NSS pin and writing a high value to the
 * reset pin.
 * 
 * @param sx_1278_hw A pointer to a structure of type SX1278_hw_t.
 */
void SX_1278_HW_Init(SX1278_hw_t *sx_1278_hw)
{
    SX_1278_HW_Set_NSS(sx_1278_hw, 1);
    HAL_GPIO_WritePin(sx_1278_hw->reset.port, sx_1278_hw->reset.pin,
                      GPIO_PIN_SET);
}

/**
 * The function `SX_1278_HW_Command` is used to send commands and receive data over SPI communication
 * for the SX1278 hardware.
 * 
 * @param sx_1278_hw A pointer to a structure that represents the hardware configuration of the SX1278
 * module.
 * @param cmd The cmd parameter is the command byte that needs to be sent to the SX1278 module. It is a
 * uint8_t type, which means it can hold values from 0 to 255.
 * @param option The "option" parameter is used to determine whether the command is a write or read
 * operation. If the value of "option" is WRITE_OPTION, it means that the command is a write operation.
 * If the value of "option" is not WRITE_OPTION, it means that the command is a read
 * 
 * @return either 1 or the value of rx_Byte, depending on the value of the option parameter.
 */
uint8_t SX_1278_HW_Command(SX1278_hw_t *sx_1278_hw, uint8_t cmd, uint8_t option)
{
    uint32_t time = HAL_GetTick();

    uint8_t tx_Byte = 0x00;
    uint8_t rx_Byte = 0x00;

    if (option == WRITE_OPTION)
    {
        HAL_SPI_Transmit(sx_1278_hw->spi, &cmd, 1, 100);
    }
    else
    {
        HAL_SPI_TransmitReceive(sx_1278_hw->spi, &tx_Byte, &rx_Byte, 1, 1000);
    }

    while (HAL_SPI_GetState(sx_1278_hw->spi) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - time) > TIME_OUT)
            return 0;
    }
    return (option == WRITE_OPTION) ? 1 : rx_Byte;
}

/**
 * The function sets the NSS pin of the SX1278 hardware to a specified value.
 * 
 * @param sx_1278_hw A pointer to a structure of type SX1278_hw_t. This structure contains information
 * about the hardware configuration of the SX1278 module, such as the GPIO port and pin used for the
 * NSS (chip select) signal.
 * @param value The value parameter is a uint8_t type variable that represents the value to be written
 * to the NSS (chip select) pin. It can be either 0 or 1.
 */
void SX_1278_HW_Set_NSS(SX1278_hw_t *sx_1278_hw, uint8_t value)
{
    HAL_GPIO_WritePin(sx_1278_hw->nss.port, sx_1278_hw->nss.pin,
                      (value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * The function SX_1278_HW_Reset resets the SX1278 hardware by toggling the reset pin.
 * 
 * @param sx_1278_hw The parameter "sx_1278_hw" is a pointer to a structure of type "SX1278_hw_t". This
 * structure contains information about the hardware configuration of the SX1278 module, such as the
 * GPIO pins used for communication and reset.
 */
void SX_1278_HW_Reset(SX1278_hw_t *sx_1278_hw)
{
    SX_1278_HW_Set_NSS(sx_1278_hw, 1);
    HAL_GPIO_WritePin(sx_1278_hw->reset.port, sx_1278_hw->reset.pin,
                      GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(sx_1278_hw->reset.port, sx_1278_hw->reset.pin,
                      GPIO_PIN_SET);
    HAL_Delay(10);
}
