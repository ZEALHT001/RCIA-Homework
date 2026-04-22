#include "uart.hpp"

void serial_port::init()
{
   HAL_UART_Receive_DMA(huart, rx_buffer, sizeof(rx_buffer));
}

void serial_port::send(const uint8_t *data,uint16_t size)
{
    HAL_UART_Transmit(huart, (uint8_t *)data, size, 100);
}

bool serial_port::is_online()
{
    uint16_t tick = HAL_GetTick();
    if (tick - last_tick > time_out) {
        is_connected = false;
    }
    else 
    {
        is_connected = true;
    }

    return is_connected;
}

