#include "UART.hpp"

#define UART_RX_BUFFER_SIZE 256

static uint8_t uart1_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart2_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart3_rx_buffer[UART_RX_BUFFER_SIZE];

SerialPort uart1(&huart1, uart1_rx_buffer, UART_RX_BUFFER_SIZE);
SerialPort uart2(&huart2, uart2_rx_buffer, UART_RX_BUFFER_SIZE);
SerialPort uart3(&huart3, uart3_rx_buffer, UART_RX_BUFFER_SIZE);

SerialPort::SerialPort(UART_HandleTypeDef* huart, uint8_t* rxBuffer, uint16_t bufferSize)
    : huart(huart), rxBuffer(rxBuffer), bufferSize(bufferSize), callback(nullptr), error(false) {
}

void SerialPort::startReceive() {
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, rxBuffer, bufferSize);
    if (status != HAL_OK) {
        error = true;
    } else {
        error = false;
    }
}

bool SerialPort::send(const uint8_t* data, uint16_t length) {
    return HAL_UART_Transmit(huart, (uint8_t*)data, length, HAL_MAX_DELAY) == HAL_OK;
}

void SerialPort::setCallback(UART_Callback callback) {
    this->callback = callback;
}

bool SerialPort::hasError() const {
    return error;
}

void SerialPort::clearError() {
    error = false;
}

void SerialPort::onReceiveComplete(uint16_t length) {
    if (callback) {
        callback(rxBuffer, length);
    }
}

extern "C" void UART_OnReceiveComplete(uint8_t uart_num, uint16_t length) {
    SerialPort* port = nullptr;
    
    switch(uart_num) {
        case 1:
            port = &uart1;
            break;
        case 2:
            port = &uart2;
            break;
        case 3:
            port = &uart3;
            break;
        default:
            return;
    }
    
    if (port) {
        port->onReceiveComplete(length);
        port->startReceive();
    }
}