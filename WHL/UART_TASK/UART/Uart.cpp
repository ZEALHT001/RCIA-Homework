#include "Uart.hpp"

void Uart::start_receive_IT()
{
    
    HAL_UART_AbortReceive(m_handle);
    // 开启 DMA 空闲中断接收
    HAL_UARTEx_ReceiveToIdle_DMA(m_handle, m_rx_buffer, m_buffer_len);

}

void Uart::send(const uint8_t* data, uint16_t len) {

    if (data == nullptr || len == 0) return;
    HAL_UART_Transmit_DMA(m_handle, data, len); 
}
//初始化一次
void Uart::set_rx_callback(RxCallback cb) {
    m_callback = cb;
}

void Uart::rx_event_handler(uint16_t size) {
    // 1. 如果外部注册了解析函数，把数据传给它
    if (m_callback != nullptr) {
        m_callback(m_rx_buffer, size);
    }
    // 处理完后重新挂载接收
    start_receive_IT();
}



#ifdef __cplusplus
extern "C" {
#endif 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
   
    
     if (huart->Instance == USART1) {

    //类.rx_event_handler(Size);
    
    }
}
#ifdef __cplusplus
}
#endif
