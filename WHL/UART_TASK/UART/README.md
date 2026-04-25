
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


# 编写过程

---

- **构造函数**
  初始化列表：
在 C++ 中，使用初始化列表比在构造函数体内赋值效率更高，尤其是对于 const 成员或引用成员。

语法： Constructor() : member1(val1), member2(val2) { ... }

```cpp

#ifndef UART_HPP
#define UART_HPP

#include <cstdint>
#include "usart.h"       
 

class Uart {

public:   

//构造函数//AI编写输入初始化需求，传入串口句柄，邮箱，长度
    Uart(UART_HandleTypeDef* h,uint8_t* buffer, uint16_t buffer_len) : m_handle(h), m_rx_buffer(buffer), m_buffer_len(buffer_len) {}

//开启接收
    void start_receive_IT();

//发送数据
    void send(const uint8_t* data, uint16_t len);

  //解析函数
    void rx_event_handler(uint16_t size);

    
private:

  UART_HandleTypeDef* m_handle;   

  uint8_t *m_rx_buffer;

  uint16_t m_buffer_len;

};


#endif // UART_HPP



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


void Uart::rx_event_handler(uint16_t size) {
    // 1. 如果外部注册了解析函数，把数据传给它
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
   
    
     if (huart->Instance == USART1) {

    //类.rx_event_handler(Size);
    
    }
}
}


```
哈哈根本跑不了，函数不能在函数内定义，思考回调应该怎么放，突然发觉自己是个chunb。突然想到了学长代码的回调函数写法，得出：

```cpp

#ifdef __cplusplus
extern "C" {
#endif 
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
   
    
     if (huart->Instance == USART1) {

    //数据解析
    
    }
    //多个串口
    // if (huart->Instance == USART2) {

    // //数据解析
    
    // }
}
#ifdef __cplusplus
}
#endif


```
① extern "C"
这是该代码段的核心。它的作用是告诉 C++ 编译器：请按照 C 语言 的规则来链接这个函数。

原因：C++ 支持函数重载，编译时会进行 "Name Mangling"（名字修饰），将函数名改得面目全非（例如把 foo() 变成 _Z3foov）。而 C 语言不支持重载，函数名保持原样。

作用：如果没有 extern "C"，当你用 C++ 编写回调函数时，STM32 的底层 C 驱动库（硬编码寻找 HAL_UARTEx_RxEventCallback）将找不到这个函数，导致链接报错。

② #ifdef __cplusplus
这是一个预处理器指令。

__cplusplus：这是一个内置宏。如果当前编译器是 C++ 编译器，该宏会被定义；如果是纯 C 编译器，则不会。

逻辑：只有在 C++ 环境下，才插入 extern "C" { 这一行。这样同一份代码既能被 .c 文件包含，也能被 .cpp 文件包含而不会报错。

| 层面 | 编译器 | 识别出的函数名 |
| :--- | :----: | :---: |
| 底层驱动 (C) |C 编译器 | HAL_UARTEx_RxEventCallback |
| 你的代码 (C++) | C++ 编译器 | HAL_UARTEx_RxEventCallback (因为有 extern "C") |
| 如果没有 extern "C"|C++ 编译器 | _Z28HAL_UARTEx_RxEventCallbackP18... (无法匹配！) |

### AI检查，补充
```cpp
//初始化一次
void Uart::set_rx_callback(RxCallback cb) {//传入的是解析数据的函数
    m_callback = cb;
}
//类内声明
typedef void (*RxCallback)(uint8_t* data, uint16_t len);//函数指针的类型定义,只要有一个函数，它不返回值（void），并且接收 (uint8_t*, uint16_t) 这两个参数，那么这种函数的指针类型，我就把它统称为 RxCallback。”

 RxCallback m_callback; 
```
**typedef void (*RxCallback)(uint8_t* data, uint16_t len)** 叫做 **函数指针类型定义**
|         |             |
|   :---  |    :---:    |
|定义规则|typedef void (*RxCallback)...（规定了回调函数的“长相”）|
|声明容器|类成员 RxCallback m_callback;（在类里挖了个坑，准备放函数地址）|
|填坑（注册）|调用 set_rx_callback(your_func);（把实际函数的地址放进坑里）|
|执行（回调）| m_callback(m_rx_buffer, size);（数据到了，顺着地址去找 your_func 并执行它|


1. 它的作用是什么？
简单来说，它是在**解耦（Decoupling）**硬件触发与业务逻辑。

技术层面：它允许你在程序运行过程中，动态地把一个“函数指针”或者“函数对象”传递给 Uart 类。

流程层面：

你告诉 Uart 对象：“当中断触发并接收完数据时，请调用这个函数。”

Uart 类通过 m_callback 记录下这个函数的地址。

当硬件中断发生（进入 HAL_UARTEx_RxEventCallback），类内部会通过 rx_event_handler 执行你之前传进去的那个函数。

2. 这样写的好处
① 逻辑解耦（驱动与业务分离）
Uart 类只需要负责底层的硬件操作（DMA开启、中断处理），它不需要知道数据收到后具体要做什么。

如果你在做 GPS 解析，你就传一个 parse_gps 函数进去。

如果你在做传感器数据读取，你就传一个 handle_sensor 函数进去。

好处：你不需要为了不同的项目去修改 Uart.cpp 底层源码。

② 提高代码复用性
想象一下，如果你有三个串口，分别连接不同的模块。如果没有回调：

你可能要在 rx_event_handler 里写一堆 if...else 或 switch 来判断现在是给哪个模块解析数据。
有了回调：

你只需创建三个 Uart 实例，并分别为它们 set_rx_callback 不同的解析函数即可。

③ 支持异步处理
中断是突然发生的。通过回调，你可以让主程序继续跑，等到数据真的准备好了，底层驱动会自动“回过头”来调用你的业务逻辑，这符合事件驱动编程的思想。

