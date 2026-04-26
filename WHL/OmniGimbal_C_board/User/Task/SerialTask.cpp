#include "SerialTask.hpp"
#include "../User/Task/ControlTask.hpp"
#include "../User/Task/CommunicationTask.hpp"



/**
 * @brief 初始化
 */
/* 陀螺仪 ---------------------------------------------------------------------------------------------------*/
BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

/* 遥控器 ---------------------------------------------------------------------------------------------------*/
BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];



/* 串口接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief 串口初始化函数
 * 
 * 初始化串口并注册设备反馈数据解析回调函数
 */
// void SerialInit()
// {
//     // 实例串口
//     auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
//     auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
//      auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
//     // 设置缓冲区
//     HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
//     HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};
//     HAL::UART::Data uart6_rx_buffer{BoardRx_buffer, sizeof(BoardPacket_t)}; // 24字节

//     // 注册串口接收回调函数
//     uart1.receive_dma_idle(uart1_rx_buffer);
//     uart3.receive_dma_idle(uart3_rx_buffer);
//     uart1.register_rx_callback([](const HAL::UART::Data &data) 
//     {
//         if(data.size == 82 && data.buffer != nullptr)
//         {
//             HI12.DataUpdate(data.buffer);
//         }
//     });
//     uart3.register_rx_callback([](const HAL::UART::Data &data) 
//     {
//         if(data.size == 18 && data.buffer != nullptr)
//         {
//             DT7.parseData(data.buffer);
//         }
//     });
// }

// 增加接收缓冲区定义
uint8_t BoardRx_buffer[sizeof(BoardPacket_t)]; 
//BoardCommunication Cboard; // 确保你有这个实例

void SerialInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    
    
    HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
    HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};
    

    uart1.receive_dma_idle(uart1_rx_buffer);
    uart3.receive_dma_idle(uart3_rx_buffer);
   

    uart1.register_rx_callback([](const HAL::UART::Data &data) 
     {
         if(data.size == 82 && data.buffer != nullptr)
       {
            HI12.DataUpdate(data.buffer);
        }
    });
    uart3.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 18 && data.buffer != nullptr)
        {
            DT7.parseData(data.buffer);
        }
    });
  
    }

   

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 串口接收任务函数
 * 
 * 任务主循环，任务为空
 * 
 * @param argument 任务参数指针
 */
extern "C" {
void Serial(void const * argument)
{
    SerialInit();
    for(;;)
    {
        
        osDelay(5);
    }
}

}
