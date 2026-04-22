#ifndef __UART_HPP__
#define __UART_HPP__

#include "usart.h"
#include "dma.h"

#define time_out 1000

class serial_port
{
    private:
        UART_HandleTypeDef *huart;
        bool is_connected;
        uint16_t last_tick;
    public:
    uint8_t rx_buffer[18];
    serial_port(UART_HandleTypeDef *h) : huart(h), is_connected(false) {}
    void init();
    void send(const uint8_t *data,uint16_t size);
    bool is_online();
    void up_tick() { last_tick = HAL_GetTick(); }
    UART_HandleTypeDef* get_uart_handle() { return huart; }
};



#endif /* __UART_HPP__ */