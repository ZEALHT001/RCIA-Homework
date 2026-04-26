# 全向底盘驱动代码

## 底层通讯（串口/can）

### 串口uart
1. 初始化实例硬件接口

```cpp
#include "SerialTask.hpp"
#include "../core/APP/Referee/RM_RefereeSystem.h"
#include "CommunicationTask.hpp"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];


/* 设备通讯回调与数据解析 -------------------------------------------------------------------------------------*/

void SerivalInit()
{

    auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);

    
    HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};

    // 【核心命令】真正开启底层 DMA 空闲中断接收
    uart8.receive_dma_idle(uart8_rx_buffer);
  
    // 注册硬件层回调，将收到的生肉数据直接丢给 Cboard 解析层处理
    uart8.register_rx_callback([](const HAL::UART::Data &data) 
    {
       if(data.buffer != nullptr && data.size >= 48 )
        {
            Cboard.ParseProtocol(data.buffer, data.size);
        }
    });
}

extern "C" {
void Serival(void const * argument)
{
    SerivalInit();
    for(;;)
    {
       
       
        osDelay(1);
    }
}

}

```
 2. 回调函数
 ```cpp
 /* 串口回调 ---------------------------------------------------------------------------------------------*/
#include "MotorTask.hpp"
#include "CommunicationTask.hpp"
#include "SerialTask.hpp"
#include "../core/APP/Referee/RM_RefereeSystem.h"


extern "C" 
{
    // 处理DMA接收中的空闲线路检测
    void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
    {
        // 板间通讯
        if(huart->Instance == UART8)
        {
            HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
            
            if(huart == uart8.get_handle())
            {
                uart8.receive_dma_idle(uart8_rx_buffer);
                uart8.trigger_rx_callbacks(uart8_rx_buffer);
            }
        }
			}

    // 极其关键：防止通信刚建立时因为数据量太大导致的 ORE 死锁
    void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
    {
        if (huart->Instance == UART8) 
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            
            HAL::UART::Data uart8_rx_buffer{BoardRx, sizeof(BoardRx)};
            auto &uart8 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart8);
            uart8.receive_dma_idle(uart8_rx_buffer);
        }
    }

		}

     ```
3. 板间通信数据接收处理
```cpp
#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include <cstdint>
#include <cstring>

// 协议总长度：1(帧头) + 18(DT7) + 4(电机角度) + 1(校验和) = 24字节
#define BOARD_PACKET_SIZE 24

// 【核心修复1】：开辟双倍缓冲区，必定能容纳至少一帧完整的错位包
extern uint8_t BoardRx[BOARD_PACKET_SIZE * 2];

class BoardCommunication
{
public:
    BoardCommunication() : yaw_angle(0.0f) 
    {
        memset(dt7_raw_data, 0, 18);
    }

    // 【核心修复2】：滑动窗口扫描解析
    void ParseProtocol(const uint8_t* buffer, uint16_t size) 
    {
        // 扫包：从头开始找 0x5A，并且保证后面有足够的长度
        for(int i = 0; i <= size - BOARD_PACKET_SIZE; i++)
        {
            if(buffer[i] == 0x5A) // 找到了潜在的包头
            {
                uint8_t sum = 0;
                // 计算除了校验位之外的 23 个字节的和
                for(int j = 0; j < BOARD_PACKET_SIZE - 1; j++) {
                    sum += buffer[i + j];
                }

                // 比对校验位
                if(sum == buffer[i + BOARD_PACKET_SIZE - 1]) 
                {
                    // 校验成功！提取数据
                    memcpy(dt7_raw_data, &buffer[i + 1], 18);
                    memcpy(&yaw_angle, &buffer[i + 19], sizeof(float));
                    
                    //可选：如果你需要直接把遥控器数据喂给底层的 DT7 解析器
                    extern BSP::REMOTE_CONTROL::RemoteController DT7;
                    DT7.parseData(dt7_raw_data);

                    return; // 成功解析一包，直接退出
                }
            }
        }
    }

    float GetYawAngle() const { return yaw_angle; }
    const uint8_t* GetRemoteDT7Data() const { return dt7_raw_data; }

private:
    float yaw_angle;     
    uint8_t dt7_raw_data[18];   
};

extern BoardCommunication Cboard;

#endif

```
### can通信
整个文件包含了can初始化，接收数据解析，发送控制帧，模板可以照套修改操作对象和通讯接口即可。can的发送是分批发送，一个一个发可以减轻can总线负载，对控制频率的影响可以满足现实工况。实例电机对象
```cpp
 #include "MotorTask.hpp"


BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);

 
void MotorInit(void)
{
    
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    
    
  
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
    });
   
}


static void motor_control_logic(uint32_t tick)
{
   
    if (tick % 2 == 0)
    {
        for(int i = 0; i < 4; i++)
        {
            Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
        }
        Motor3508.sendCAN();
    }

  
}
//  RTOS任务函数
extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    static uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        osDelay(1);
    } 
}

}

```
**can回调函数**

```cpp
/* CAN回调 ---------------------------------------------------------------------------------------------*/
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame rx_frame;
    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    //若有can2
    //auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);


    if (hcan == can1.get_handle())
    {
        can1.receive(rx_frame);  
    }
    //can2
    // if (hcan == can2.get_handle())
    // {
    //     can2.receive(rx_frame);  
    // }
}
```

### 底盘控制链路（重点！！！）

结构体用于存放期望值和输出值
```cpp
typedef struct 
{
    float target_translation_x;
    float target_translation_y;
    float target_rotation;
    float target_dial;
    float target_x;
    float target_y;
}ControlTask;

typedef struct
{
    float out_wheel[4];
    float out_dial;
}Output_chassis;
```

**底盘关键解算**
```cpp
float wheel_azimuth[4] = {-PI_/4, -3*PI_/4, 3*PI_/4, PI_/4};      //底盘基本数据计算，用于解算传入
float wheel_direction[4] = {-3*PI_/4, 3*PI_/4, PI_/4, -PI_/4};   //
Alg::CalculationBase::Omni_IK omni_ik(0.24f, 0.07f, wheel_azimuth, wheel_direction);//全向轮底盘运动解算逆解，计算每个轮子的速度分配        
Alg::CalculationBase::Omni_FK omni_fk(0.24f, 0.07f, 4.0f, wheel_azimuth, wheel_direction);  //全向轮底盘运动学正解，用于计算底盘当前实际状态（速度），用于传入斜坡函数的当前反馈值

//底盘4轮的pid实例

ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 150.0f)      
};

//斜坡函数，用于减缓数值大跳变 
Alg::Utility::SlopePlanning omni_target[3] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // X
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // Y
    Alg::Utility::SlopePlanning(0.015f, 0.015f)       // Z
};
/*
**底盘不跟随
*期望设定
*/
void Notfollow_SetTarget(){
    if (DT7.get_s1() == 2 && DT7.get_s2() == 2) {
            chassis_target.target_translation_x = 0.0f;
            chassis_target.target_translation_y = 0.0f;
            chassis_target.target_rotation = 0.0f;//旋转
           }else {
            CalculateTranslation_xy(Cboard.GetYawAngle(),-DT7.get_left_x() *MAX_SPEED, -DT7.get_left_y()* MAX_SPEED, 0.0f, &chassis_target.target_x, &chassis_target.target_y, 0.0f);
            omni_target[1].TIM_Calculate_PeriodElapsedCallback(chassis_target.target_y , omni_fk.GetChassisVy());
            omni_target[0].TIM_Calculate_PeriodElapsedCallback(chassis_target.target_x , omni_fk.GetChassisVx());

             chassis_target.target_translation_x= omni_target[0].GetOut();
             chassis_target.target_translation_y= omni_target[1].GetOut();
            chassis_target.target_rotation = 0.0f;//旋转
           }
}


//期望值状态机，根据不同底盘状态进行期望值的输入
void SetTarget()
{
    // 璁剧疆姝诲尯
    DT7.SetDeadzone(20.0f);
    // 搴曠洏
   switch(chassis_fsm.Get_Now_State()) 
   {
       case STOP:  
           chassis_target.target_translation_x = 0.0f;
           chassis_target.target_translation_y = 0.0f;
           chassis_target.target_rotation = 0.0f;//旋转
           break;
       case FOLLOW:  
       Follow_SetTarget();

           break;
       case NOTFOLLOW:  //不跟随
          Notfollow_SetTarget();

           break;
       default:
           chassis_target.target_translation_x = 0.0f;
           chassis_target.target_translation_y = 0.0f;
           chassis_target.target_rotation = 0.0f;
           break;
   }
}

/**
 * @brief 底盘不跟随的控制器闭环计算，最终输出output为发送给电机的值
 * 
 * 
 */
void chassis_notfollow()
{
  
  omni_ik.OmniInvKinematics(chassis_target.target_translation_x,chassis_target.target_translation_y,  chassis_target.target_rotation, 0 , 1.0f , 1.0f);
  
  for (int i = 0; i < 4 ; i++) {

   wheel_pid[i].UpDate(omni_ik.GetMotor(i),Motor3508.getVelocityRads(i+1));
   chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
   chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
  }
}

/**
 * @brief 任务主函数
 * 根据状态机调用控制器函数
 * @param left_sw 左摇杆
 * @param right_sw 右摇杆
 * @param is_online 断联检测
 */
void main_loop_chassis(uint8_t left_sw, uint8_t right_sw, bool is_online,bool *alphabet) 
{   
   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online,alphabet);//底盘状态机
    SetTarget();//期望值技术传入结构体

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        default:
            chassis_stop();
            break;
    }
}

//最终放入RTOS任务函数运行

extern "C"{
void Control(void const * argument)
{
   
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();//蜂鸣器
    fsm_init();//状态机初始化
    for(;;)
    {
        
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
		bool dummy_alphabet[20] = {false};	
        main_loop_chassis(DT7.get_s1(), DT7.get_s2(), check_online(), dummy_alphabet);
       
        osDelay(1);
    } 
}
}
```

**流程图**

1. 
graph NotFollow
    
 获取遥控器控制值（x/y） ---> 可添加斜坡函数平滑曲线
 可添加斜坡函数平滑曲线 ---> 获取yaw电机偏航角进行底盘相对云台坐标系的平移解算（二维旋转矩阵） 
 获取yaw电机偏航角进行底盘相对云台坐标系的平移解算（二维旋转矩阵） ---> 进行运动学逆解
 进行运动学逆解 ---> 进行各轮组的速度pid计算
 进行各轮组的速度pid计算 ---> 发送电机


 2. 
graph Follow
    
 获取遥控器控制值（x/y） ---> 可添加斜坡函数平滑曲线
 可添加斜坡函数平滑曲线 ---> 获取yaw电机偏航角进行底盘相对云台坐标系的平移解算（二维旋转矩阵x/y） 
 获取yaw电机偏航角进行底盘相对云台坐标系的平移解算（二维旋转矩阵x/y） ---> 底盘跟随云台的角速度(w)规划，偏航值为0的follow——pid闭环
 底盘跟随云台的角速度(w)规划，偏航值为0的follow——pid闭环 ---> x/y/w传入运动学逆解
 x/y/w传入运动学逆解 ---> 进行各轮组的速度pid计算
 进行各轮组的速度pid计算 ---> 发送电机