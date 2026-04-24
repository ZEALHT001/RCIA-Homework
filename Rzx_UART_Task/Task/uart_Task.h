#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#include "stm32f4xx_hal.h"

typedef struct{
    UART_HandleTypeDef huart1;//UART1句柄
    uint8_t *rx_buffer;//接收缓冲区
    uint16_t rx_buffer_size;//接收缓冲区大小
    void (*data_received_callback)(uint8_t *data, uint16_t length);//数据接收回调函数

    uint8_t *tx_buffer;//发送缓冲区
    uint16_t tx_buffer_size;//发送缓冲区大小
    void (*data_send_callback)(uint8_t *data, uint16_t length);//数据发送回调函数
} uart_Task_t;

//初始化并启动DMA + 空闲中断接收数据
void uart_Task_Init(uart_Task_t *uart_task);
void uart_Task_SendData(uart_Task_t *uart_task, uint8_t *data, uint16_t length);//发送数据函数  
void uart_dma_idle_handler(UART_HandleTypeDef *huart);//DMA空闲中断处理函数

#endif 