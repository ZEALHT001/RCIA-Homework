#include "usart.h"
#include "stdint.h"

class SerialPort {
private: 
    UART_HandleTypeDef* handle;  
    uint8_t* rx_buffer;          
    uint16_t buffer_size;        

    
    bool is_online;       // （true=在线，false=离线）
    uint32_t last_tick;   // 记录“上次收到数据”的毫秒时间戳
    uint32_t timeout_ms;  

public: 
    
    SerialPort(UART_HandleTypeDef* h, uint8_t* buf, uint16_t size, uint32_t timeout = 50) {
        handle = h;           
        rx_buffer = buf;      
        buffer_size = size;   
        
        is_online = false;    // 刚开机，默认是离线状态
        last_tick = 0;
        timeout_ms = timeout; 
    }

    void start() {
        HAL_UARTEx_ReceiveToIdle_DMA(handle, rx_buffer, buffer_size);
    }

    void update_tick() {
        is_online = true;               
        last_tick = HAL_GetTick();      
    }

   
    void check_status() {
      
        if (HAL_GetTick() - last_tick > timeout_ms) {
            is_online = false;          
        }
        //  硬件错误自愈（急救逻辑）
        // 检查底层 HAL 库的句柄里，有没有记录错误代码（ERROR_NONE 就是没错误）
        if (handle->ErrorCode != HAL_UART_ERROR_NONE) {
            // 糟糕，硬件晕倒了！开始抢救：
            
            // 抢救第一步：强行停止当前已经卡死的 DMA 接收
            HAL_UART_DMAStop(handle); 
            
            // 抢救第二步：清除底层寄存器里的 ORE（溢出）标志位，这是卡死的罪魁祸首
            __HAL_UART_CLEAR_OREFLAG(handle);
            
            // 抢救第三步：把句柄里的错误码清零，假装一切正常
            handle->ErrorCode = HAL_UART_ERROR_NONE; 
            
            // 抢救第四步：重新按下我们的启动按钮，满血复活！
            start(); 
        }
    }

    bool get_is_online() {
        return is_online;
    }
};