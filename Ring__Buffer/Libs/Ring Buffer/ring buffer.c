/**
 * This is a C program for implementing a ring buffer data structure.
 * 
 * @param ring_buffer_x a pointer to a RING_BUFFER_HandleTypeDef structure, which contains information
 * about the ring buffer such as its size, front and rear indices, and data array.
 * @param size The size of the ring buffer, i.e., the maximum number of elements that can be stored in
 * the buffer.
 * @param huart_x UART handle used for communication with the device.
 */
/*
 * Ring Buffer.c
 *
 *  Created on: Apr 19, 2023
 *      Author: KHOA DONG
 */

#include "ring buffer.h"
#include "stdlib.h"

#define STM_OK  1
#define STM_ERR 0

/**
 * The function initializes a ring buffer with a given size and assigns a UART handle to it.
 * 
 * @param ring_buffer_x A pointer to a RING_BUFFER_HandleTypeDef structure, which contains information
 * about the ring buffer.
 * @param size The size parameter is the maximum number of elements that the ring buffer can hold. It
 * determines the capacity of the buffer.
 * @param huart_x huart_x is a pointer to a UART_HandleTypeDef structure, which contains the
 * configuration and state information for a UART (Universal Asynchronous Receiver/Transmitter)
 * peripheral. This parameter is used to associate the ring buffer with a specific UART peripheral, so
 * that data can be transmitted and received using that UART.
 */
void RING_BUFFER_Init(RING_BUFFER_HandleTypeDef *ring_buffer_x, uint8_t size, UART_HandleTypeDef *huart_x)
{
    ring_buffer_x->front = -1;
    ring_buffer_x->rear = -1;
    ring_buffer_x->huart = huart_x;
    ring_buffer_x->size = size;
    ring_buffer_x->data_arr = (char *)malloc(size * sizeof(ring_buffer_x->data_arr));
}

/**
 * The function checks if a ring buffer is full or not.
 * 
 * @param ring_buffer_x a pointer to a struct of type RING_BUFFER_HandleTypeDef, which contains
 * information about the ring buffer being checked.
 * 
 * @return either STM_OK or STM_ERR depending on whether the ring buffer is full or not. If the ring
 * buffer is full, STM_OK is returned, otherwise STM_ERR is returned.
 */
uint8_t RING_BUFFER_Is_Full(RING_BUFFER_HandleTypeDef *ring_buffer_x)
{
    if ((ring_buffer_x->front == ring_buffer_x->rear + 1)
            || ((ring_buffer_x->front == 0) && (ring_buffer_x->rear == ring_buffer_x->size - 1)))
    {
        return STM_OK;
    }
    return STM_ERR;
}

/**
 * The function checks if a ring buffer is empty.
 * 
 * @param ring_buffer_x a pointer to a struct of type RING_BUFFER_HandleTypeDef, which contains
 * information about a ring buffer.
 * 
 * @return a value of type uint8_t, which is an unsigned 8-bit integer. The value being returned is the
 * result of the comparison between the front index of the ring buffer and -1. If the front index is
 * -1, indicating that the buffer is empty, the function will return 1 (true). Otherwise, it will
 * return 0 (false).
 */
uint8_t RING_BUFFER_Is_Empty(RING_BUFFER_HandleTypeDef *ring_buffer_x)
{
    return (ring_buffer_x->front == -1);
}

/**
 * The function pushes a character element into a ring buffer if it is not full.
 * 
 * @param ring_buffer_x a pointer to a RING_BUFFER_HandleTypeDef structure, which contains information
 * about the ring buffer such as its size, front and rear indices, and data array.
 * @param element The element parameter is the data that needs to be pushed into the ring buffer. It
 * can be of any data type, but in this case, it is a character (char) data type.
 */
void RING_BUFFER_Push_Data(RING_BUFFER_HandleTypeDef *ring_buffer_x, char element)
{
    if (RING_BUFFER_Is_Full(ring_buffer_x))
    {
        printf("RING BUFFER is Full !!!");
    }
    else
    {
        if (ring_buffer_x->front == -1)
        {
            ring_buffer_x->front = 0;
        }
        ring_buffer_x->rear = (ring_buffer_x->rear + 1) % ring_buffer_x->size;
        ring_buffer_x->data_arr[ring_buffer_x->rear] = element;
    }
}

/**
 * This function pulls data from a ring buffer and returns the element, or returns -1 if the buffer is
 * empty.
 * 
 * @param ring_buffer_x a pointer to a RING_BUFFER_HandleTypeDef structure, which contains information
 * about the ring buffer such as its size, front and rear indices, and data array.
 * 
 * @return a character (type 'char') which is the element pulled from the front of the ring buffer. If
 * the ring buffer is empty, the function returns -1.
 */
char RING_BUFFER_Pull_Data(RING_BUFFER_HandleTypeDef *ring_buffer_x)
{
    if (RING_BUFFER_Is_Empty(ring_buffer_x))
    {
        printf("RING BUFFER is Empty !!!");
        return -1;
    }
    else
    {
        char element;
        element = ring_buffer_x->data_arr[ring_buffer_x->front];
        if (ring_buffer_x->front == ring_buffer_x->rear)
        {
            ring_buffer_x->front = -1;
            ring_buffer_x->rear = -1;
        }
        else
        {
            ring_buffer_x->front = (ring_buffer_x->front + 1) % ring_buffer_x->size;
        }
        return element;
    }
    
}