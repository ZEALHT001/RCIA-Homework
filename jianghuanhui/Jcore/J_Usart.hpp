#include "main.h"
#include "usart.h"

namespace Usart
{

  class Usart_data
  {
    private:
    UART_HandleTypeDef* huartx[8];                     //一下子传入所有的usart
    uint16_t length[8];                                //一下子传入所有的usart的长度
    
    public :
    Usart_data(UART_HandleTypeDef* port[8],uint16_t len[8]):huartx{},length{}
    {
      for (int i=0;i<8;i++)
      {
        huartx[i]=port[i];
        length[i]=len[i];
      }
    }
  
    protected:

    void send_data_DMA(uint8_t*data,uint8_t Usart_idx)//Usart_size就是要选第几个USART
    {
    HAL_UART_Transmit_DMA(huartx[Usart_idx], data,sizeof(data));
    }

    void receive_data_DMA(uint8_t*data,uint8_t Usart_idx)
    {
    HAL_UART_Receive_DMA(huartx[Usart_idx], data,sizeof(data));
    }

  public:
    void Tx_Rx_DMA(uint8_t*data,uint8_t Usart_idx)
    {
    receive_data_DMA(data,Usart_idx);
    send_data_DMA(data,Usart_idx);
    }
  };
}