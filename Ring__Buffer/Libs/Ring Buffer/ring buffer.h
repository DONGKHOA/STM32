/*
 * Ring Buffer.h
 *
 *  Created on: Apr 19, 2023
 *      Author: KHOA DONG
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include "main.h"

typedef struct 
{
    //Initialize front, rear
    int8_t front, rear;

    //Circular Queue
    uint8_t size;
    char *data_arr;
    UART_HandleTypeDef *huart;
}RING_BUFFER_HandleTypeDef;


void RING_BUFFER_Init(RING_BUFFER_HandleTypeDef *ring_buffer_x, uint8_t size, UART_HandleTypeDef *huart_x);

void RING_BUFFER_Is_Full(RING_BUFFER_HandleTypeDef *ring_buffer_x);

void RING_BUFFER_Is_Empty(RING_BUFFER_HandleTypeDef *ring_buffer_x);

void RING_BUFFER_Push_Data(RING_BUFFER_HandleTypeDef *ring_buffer_x);

char RING_BUFFER_Pull_Data(RING_BUFFER_HandleTypeDef *ring_buffer_x);

#endif /* RING_BUFFER_H_ */
