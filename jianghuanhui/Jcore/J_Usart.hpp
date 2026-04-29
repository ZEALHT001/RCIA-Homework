#include "main.h"
#include "usart.h"

namespace Usart
{

  class Usart_data
  {
    typedef void (*J_uart_callback)(uint8_t* data, uint16_t len);
    private:
    UART_HandleTypeDef* huartx;                     //一下子传入所有的usart
    uint16_t length;                                //一下子传入所有的usart的长度
    J_uart_callback J_huartx;
    uint8_t rx_buffer_uart[18];
    uint8_t tx_buffer_uart[18];               
    

    public :
    Usart_data(UART_HandleTypeDef* port,uint16_t len,J_uart_callback J_port):huartx(port),length(len),J_huartx(J_port){
    }


    void send_data_DMA()//Usart_size就是要选第几个USART
    {
    HAL_UART_Transmit_DMA(huartx, tx_buffer_uart,sizeof(tx_buffer_uart));
    }


    void receive_data_DMA()
    {
    HAL_UART_Receive_DMA(huartx, rx_buffer_uart,sizeof(rx_buffer_uart));
    }


    void J_callback(uint8_t a,J_uart_callback Juartx)   //a:  1发送   0:接收
    {
      if(a==1)      Juartx(tx_buffer_uart,sizeof(tx_buffer_uart));
      else if(a==0)  Juartx(rx_buffer_uart,sizeof(rx_buffer_uart));
    }


    void prase_data(uint8_t uart_len)
    {
      if(J_huartx != nullptr) 
      {
        J_huartx(rx_buffer_uart, uart_len); 
      }
      receive_data_DMA(); 
    }
  };
}


#ifdef __cplusplus
extern "C" {
#endif 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
   
    
     if (huart->Instance == USART1) {

    //类.prase_data(uart_len);
    
    }
}
#ifdef __cplusplus
}
#endif
