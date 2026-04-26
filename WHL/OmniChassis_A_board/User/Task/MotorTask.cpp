#include "MotorTask.hpp"

/* 瀹炰緥鐢垫満 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);


/*CAN鎺ユ敹 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN鍒濆鍖栧嚱鏁?
 * 
 * 鍒濆鍖朇AN鎬荤嚎骞舵敞鍐岀數鏈哄弽棣堟暟鎹В鏋愬洖璋冨嚱鏁?
 */
void MotorInit(void)
{
    // 瀹炰緥CAN
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


