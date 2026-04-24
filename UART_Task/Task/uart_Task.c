#include "uart_Task.h"

void uart_Task_Init(uart_Task_t *uart_task) {
    // 初始化UART1
    uart_task->huart1.Instance = USART1;
    uart_task->huart1.Init.BaudRate = 115200;
    uart_task->huart1.Init.WordLength = UART_WORDLENGTH_8B;
    uart_task->huart1.Init.StopBits = UART_STOPBITS_1;
    uart_task->huart1.Init.Parity = UART_PARITY_NONE;
    uart_task->huart1.Init.Mode = UART_MODE_TX_RX;
    uart_task->huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_task->huart1.Init.OverSampling = UART_OVERSAMPLING_16;

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

void uart_dma_idle_handler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart); // 清除空闲标志

        // 获取接收数据长度
        uint16_t received_length = uart_task->rx_buffer_size - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        // 调用数据接收回调函数
        if (uart_task->data_received_callback) {
            uart_task->data_received_callback(uart_task->rx_buffer, received_length);
        }

        // 重新启动DMA接收
        HAL_UART_Receive_DMA(huart, uart_task->rx_buffer, uart_task->rx_buffer_size);
    }
}