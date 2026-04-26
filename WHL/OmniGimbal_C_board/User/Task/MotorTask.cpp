#include "MotorTask.hpp"

BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {2, 6}, {0x01, 0x05}); // 1是pitch，2是yaw
BSP::Motor::DM::J4340<1> MotorJ4340(0x00, {4}, {0x03});

/* CAN接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN初始化函数
 * 
 * 初始化CAN总线并注册电机反馈数据解析回调函数
 */
void MotorInit(void)
{
    
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    can2.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        MotorJ4310.Parse(frame);
        MotorJ4340.Parse(frame);
    });
}

/* 电机发送控制帧 ----------------------------------------------------------------------------------------*/
template <typename MotorType>
static void handle_dm_motor_state_machine(MotorType& motor, uint8_t id, uint8_t &step, bool target_enable, float pos, float vel, float kp, float kd, float torq)
{
    if (target_enable)
    {
        // 如果需要使能，并且已经成功使能且无错误
        if (motor.getIsenable(id) && motor.getError(id) == 1)
        {
            step = 0;
            motor.ctrl_Mit(id, pos, vel, kp, kd, torq); // 发送正常控制帧
            return;
        }

        switch (step)
        {
            case 0:
                motor.ClearErr(id, BSP::Motor::DM::MIT);
                step = 1;
                break;
            case 1:
                if (motor.getError(id) == 0) // 等待清错成功
                {
                    motor.On(id, BSP::Motor::DM::MIT);
                    step = 2;
                }
                break;
            case 2:
                if (motor.getError(id) == 1) // 等待使能成功
                {
                    motor.setIsenable(id, true);
                    step = 0;
                }
                else if (motor.getError(id) > 1) // 出现错误
                {
                    step = 0; // 重启流程
                }
                break;
            default: step = 0; break;
        }
    }
    else
    {
        // 如果需要失能，并且已经成功失能且无报错
        if (!motor.getIsenable(id) && motor.getError(id) == 0)
        {
            step = 0;
            motor.ctrl_Mit(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f); // 收数据用（此时非常安全，不打断On）
            return;
        }

        switch (step)
        {
            case 0:
                motor.Off(id, BSP::Motor::DM::MIT);
                step = 1;
                break;
            case 1:
                motor.ClearErr(id, BSP::Motor::DM::MIT);
                step = 2;
                break;
            case 2:
                if (motor.getError(id) == 0) // 等待失能并清错成功
                {
                    motor.setIsenable(id, false);
                    step = 0;
                }
                else if (motor.getError(id) > 1) // 出现错误
                {
                    step = 0; // 重启流程
                }
                break;
            default: step = 0; break;
        }
    }
}


/**
 * @brief 电机控制函数
 * 
 * 根据控制系统的输出结果向各电机发送控制指令
 */
static void motor_control()
{
    static uint8_t send_seq = 0;
    send_seq++;


    static uint8_t re_enable_step_pitch = 0;
    static uint8_t re_enable_step_yaw = 0;
    static uint8_t re_enable_step_pitch1 = 0;
 

    bool is_gimbal_active = (gimbal_fsm.Get_Now_State() != STOP);

    if (send_seq % 3 == 0) // 第三次
    {
        // can2 (Pitch2)
        handle_dm_motor_state_machine(MotorJ4310, 
                                        1, 
                                        re_enable_step_pitch, 
                                        is_gimbal_active,
                                        0.0f, 
                                        gimbal_output.out_pitch2, 
                                        0.0f, 
			                            0.5f, 
                                        0.0f);

    }
    else if(send_seq % 3 == 1)  // 第一次
    {
        // can2 (Yaw)
        handle_dm_motor_state_machine(MotorJ4310, 
                                      2, 
                                      re_enable_step_yaw, 
                                      is_gimbal_active,
                                      0.0f, 
                                      0.0f, 
                                      0.0f, 
                                      0.0f, 
                                      gimbal_output.out_yaw);

       
    }
    else if(send_seq % 3 == 2)  // 第二次， pitch1
    {
        // can2 (Pitch1)
        handle_dm_motor_state_machine(MotorJ4340, 
                                      1, 
                                      re_enable_step_pitch1, 
                                      is_gimbal_active,
                                      gimbal_target.target_pitch1, 
                                      gimbal_target.target_pitch1_vel, 
                                      6.0f, 
                                      0.5f, 
                                    //   0.0f,
                                    //   0.0f,
                                    //   0.0f,
                                    //   0.0f,
                                      gimbal_output.out_pitch1);
    }
}                               

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制任务函数
 * 
 * 任务主循环，负责初始化电机并持续执行电机控制
 * 
 * @param argument 任务参数指针
 */
extern "C"{
void Motor(void const * argument)
{
    MotorInit();    // 初始化电机和CAN总线
    for(;;)
    {
        motor_control();  // 执行电机控制
        osDelay(1);       // 延时1ms，控制频率1000Hz
    } 
}

}