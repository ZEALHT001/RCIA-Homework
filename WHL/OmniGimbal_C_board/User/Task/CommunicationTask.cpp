 #include "CommunicationTask.hpp"
 #include "../User/Task/MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp" // 为了使用 DT7Rx_buffer

BoardPacket_t TxPacket;

void BoardCommunicationTX()
{
    // 1. 获取硬件实例 (UART6)
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart6);
    
    // 2. 获取数据：电机角度（假设从你的 Motor 库获取）
    float angle = MotorJ4310.getAngleRad(2); 

    // 3. 填充结构体
    TxPacket.header = 0x5A;
    memcpy(TxPacket.dt7_raw, DT7Rx_buffer, 18);
    TxPacket.motor_angle = angle;

    // 4. 计算和校验
    uint8_t sum = 0;
    uint8_t *ptr = (uint8_t *)&TxPacket;
    for (size_t i = 0; i < sizeof(BoardPacket_t) - 1; i++) {
        sum += ptr[i];
    }
    TxPacket.checksum = sum;

    // 5. 调用发送
    HAL::UART::Data uart6_tx_buffer{(uint8_t*)&TxPacket, sizeof(BoardPacket_t)};
    uart6.transmit_dma(uart6_tx_buffer);
}

// 在你的 Communication 任务循环中调用

// void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
// {
//    const uint8_t sendSize = sizeof(float); // 鍗曟诞鐐规暟鍗?瀛楄妭

//    // 灏?涓诞鐐规暟鎹啓鍏ョ紦鍐插尯锛堝皬绔ā寮忥級
//    *((float*)&send_str2[sendSize * 0]) = x1;
//    *((float*)&send_str2[sendSize * 1]) = x2;
//    *((float*)&send_str2[sendSize * 2]) = x3;
//    *((float*)&send_str2[sendSize * 3]) = x4;
//    *((float*)&send_str2[sendSize * 4]) = x5;
//    *((float*)&send_str2[sendSize * 5]) = x6;

//    // 鍐欏叆甯у熬锛堝崗璁姹?0x00 0x00 0x80 0x7F锛?
//    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 灏忕瀛樺偍涓?00 00 80 7F

// }


extern "C" {
void Communication(void const * argument)
{
    for(;;)
    {
        BoardCommunicationTX();
        osDelay(5); 
    }
}
}






