/*
 * sx_1278_hw.h
 *
 *  Created on: Sep 21, 2023
 *      Author: Khoa dong
 */

#ifndef SX_1278_HW_H_
#define SX_1278_HW_H_

#include <stdint.h>

#define READ_OPTION 0
#define WRITE_OPTION 1

typedef struct
{
	uint8_t pin;
	void *port;
} SX1278_hw_dio_t;

typedef struct
{
	SX1278_hw_dio_t reset;
	SX1278_hw_dio_t dio0;
	SX1278_hw_dio_t nss;
	void *spi;
} SX1278_hw_t;

void SX_1278_HW_Init(SX1278_hw_t *sx_1278_hw);
uint8_t SX_1278_HW_Command(SX1278_hw_t *sx_1278_hw, uint8_t cmd, uint8_t option);
void SX_1278_HW_Set_NSS(SX1278_hw_t *sx_1278_hw, uint8_t value);
void SX_1278_HW_Reset(SX1278_hw_t *sx_1278_hw);

#endif /* SX_1278_HW_H_ */
