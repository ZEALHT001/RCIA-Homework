#include "uart_Task.h"

void uart_Task_Init(uart_Task_t *uart_task) {
    
    if (HAL_UART_Init(&uart_task->huart1) != HAL_OK) {
        // 初始化错误处理
        Error_Handler();
    }

    // 启动DMA接收和空闲中断
    HAL_UART_Receive_DMA(&uart_task->huart1, uart_task->rx_buffer, uart_task->rx_buffer_size);
    __HAL_UART_ENABLE_IT(&uart_task->huart1, UART_IT_IDLE);

    void uart_Task_SendData(uart_Task_t *uart_task, uint8_t *data, uint16_t length) {
    // 发送数据
}
}

// DMA空闲中断处理
void uart_Task_IdleCallback(uart_Task_t *uart_task) {
    // 获取接收数据长度
    uint16_t received_length = uart_task->rx_buffer_size - __HAL_DMA_GET_COUNTER(uart_task->huart1.hdmarx);

    // 调用数据接收回调函数
    if (uart_task->data_received_callback) {
        uart_task->data_received_callback(uart_task->rx_buffer, received_length);
    }
// 重新启动DMA接收
   HAL_UART_Receive_DMA(&uart_task->huart1, uart_task->rx_buffer, uart_task->rx_buffer_size);  
    
}

void uart_Task_SendData(uart_Task_t *uart_task, uint8_t *data, uint16_t length) {
    // 发送数据
    HAL_UART_Transmit(&uart_task->huart1, data, length, HAL_MAX_DELAY);
}