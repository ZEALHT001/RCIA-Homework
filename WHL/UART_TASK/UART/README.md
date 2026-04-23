
#  吴海林的专属串口驱动 (Uart 类) 使用指南


---

##  接收 

### 1. 准备缓冲区并实例化对象
```cpp

#include "Uart.hpp"

// 1. 定义足够大的接收缓冲区(例如用于接收 18 字节的遥控器数据)
uint8_t dt7_rx_buffer[36];

// 2. 实例化串口对象
// 参数依次为: 串口句柄指针、缓冲区首地址、缓冲区长度
Uart uart1_dt7(&huart1, dt7_rx_buffer, sizeof(dt7_rx_buffer));

// 3. 在初始化时调用一次开启接收
uart1_dt7.start_receive_IT();


```
### 2.编写回调函数
```cpp
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
```

### 3.将解析函数挂载上对应的对象

```cpp
//数据解析函数
void DT7_Parse();
//挂载，放入初始化执行一次即可
uart1_dt7.set_rx_callback(DT7_Parse());

```

### 4.调用
```cpp
//数据解析执行，放入回调函数内
uart1_dt7.rx_event_handler(m_buffer_len);

```

## 发送

### 1.发送缓冲区

```cpp
uint8_t date[25];
uart1_dt7.send(&date sizeof(date));

```