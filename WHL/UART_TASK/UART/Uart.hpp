#ifndef UART_HPP
#define UART_HPP

#include <cstdint>
#include "usart.h"       
 

class Uart {

public:   

    //构造函数
    Uart(UART_HandleTypeDef* h,uint8_t* buffer, uint16_t buffer_len) : m_handle(h), m_rx_buffer(buffer), m_buffer_len(buffer_len) {}

//开启接收
    void start_receive_IT();
//发送数据
    void send(const uint8_t* data, uint16_t len);

   //回调函数
    void set_rx_callback(RxCallback cb);
    
  //解析函数接口
    void rx_event_handler(uint16_t size);

    typedef void (*RxCallback)(uint8_t* data, uint16_t len);//函数指针的类型定义,只要有一个函数，它不返回值（void），并且接收 (uint8_t*, uint16_t) 这两个参数，那么这种函数的指针类型，我就把它统称为 RxCallback。”
    
private:

  UART_HandleTypeDef* m_handle;   

  uint8_t *m_rx_buffer;

  uint16_t m_buffer_len;

  RxCallback m_callback; 
};


#endif // UART_HPP