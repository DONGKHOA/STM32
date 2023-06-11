/*
 * mode.h
 *
 *  Created on: May 29, 2023
 *      Author: thanh
 */

#ifndef MODE_H_
#define MODE_H_

#include "main.h"

typedef struct
{
    uinit8_t mode;
    SPI_HandleTypeDef *spi_x;
    
} MODE_HandleTypeDef;


void MODE_Init();


#endif /* MODE_H_ */
