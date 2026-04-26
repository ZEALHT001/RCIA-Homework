#include "ControlTask.hpp"
#include <algorithm>


/* 外部变量 ------------------------------------------------------------------------------------------------*/
extern bool alphabet[28];
extern float pitch2_angle = 0;

#define PI_ 3.14159265
#define MAX_PITCH2 30.Of
const float STANDUP_TOLERANCE = 5.0f;
const float TARGET_BIG_PITCH_ANGLE = -103.0f;
// 小Pitch初始值捕捉与状态切换标志位
static bool is_first_init = true;
static float init_pitch2_target = 0.0f;
static bool is_standup_complete = false;
static bool just_standup = true;
/**
 * @brief 初始化
 */
/* 有限状态机 -----------------------------------------------------------------------------------------------*/
Gimbal_FSM gimbal_fsm;  // 云台

/* 前馈 -----------------------------------------------------------------------------------------------------*/

Alg::Feedforward::Gravity gravity_forward_pitch1(1.0f, 0.0f);   // pitch1重补
Alg::Feedforward::Gravity gravity_forward_pitch2(1.0f, 0.0f);   // pitch2重补

/*TD跟踪微分器*/
TDFilter pitch1_transfore(20.0f,0.002f);
TDFilter pitch2_control(50.0f,0.002);
TDFilter yaw_control(50.0f,0.002);
//imu
LPFFilter imu_pitch2(0.2f);

/* 控制器 ---------------------------------------------------------------------------------------------------*/
ALG::ADRC::FirstLADRC yaw_adrc(25.0f, 70.0f, 105.0f, 0.002f, 800.0f, 3.0f);     // 普通模式Yaw 速控ADRC

ALG::PID::PID transform_pitch1_angle_pid(0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 10.0f);
ALG::PID::PID transform_pitch1_velocity_pid(0.0f,0.0f,0.0f,16384.0f, 2500.0f, 10.0f);
ALG::ADRC::FirstLADRC pitch2_adrc(25.0f, 70.0f, 105.0f, 0.002f, 800.0f, 3.0f);
ALG::PID::PID pitch2_transform_angle_pid(0.0f, 0.0f, 0.0f, 16384.0f, 30.0f, 5.0f);
ALG::PID::PID pitch2_transform_velocity_pid(0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 500.0f);
ALG::PID::PID pitch2_Angle_pid(50.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 500.0f);
ALG::PID::PID pitch2_velocity_pid(0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 500.0f);






/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
ControlTask gimbal_target;      // 云台与发射机构期望�?

Output_gimbal gimbal_output;    // 云台输出

/**
 * @brief 云台控制逻辑
 */
/* ---------------------------------------------------------------------------------------------------*/





/**
 * @brief 检查所有关键设备是否在线
 * 
 * @return true 设备全部在线
 * @return false 存在离线设备
 */
bool check_online()
{
    bool isconnected = true;

    // if(!Motor6020.isConnected(1, 6) || !MotorJ4310.isConnected(1, 4) || !Motor3508.isConnected(1, 2) || !Motor3508.isConnected(1, 3) || !Motor3508.isConnected(1, 1))
    // {
    //     isconnected = false;
    // }

    if(!DT7.isConnected() || !HI12.isConnected())
    {
        isconnected = false;
    }
    
    if(!isconnected)
    {
        return false;
    }

    return true;
}

/**
 * @brief 状态机初始化
 */
void fsm_init()
{
    gimbal_fsm.Init();
    
}





/**
 * @brief 云台目标设定
 */
void SetTarget_Gimbal()
{
    // 全局遥控器死区 (针对底层通道)
    DT7.SetDeadzone(20.0f);
    
    switch (gimbal_fsm.Get_Now_State())
    {
        case STOP:
            // ================= STOP 状态复位 =================
            // ⚠️ 极其关键：当云台掉电或停止时，必须把所有标志位恢复到初始状态！
            is_first_init = true;
            is_standup_complete = false;
            just_standup = true; // 重置单次触发标志位，为下一次起立做准备
            
            gimbal_target.target_yaw_vel = 0.0f;
            gimbal_target.target_yaw_angle = MotorJ4310.getAngleDeg(2);
            gimbal_target.target_pitch1 = MotorJ4340.getAngleDeg(1); 
            gimbal_target.target_pitch2_vel = 0.0f;
            gimbal_target.target_pitch2_angle = MotorJ4310.getAngleRad(1); // 停止时目标设定为当前电机实际角度
            break;

        case MANUAL:
               DT7.SetDeadzone(20);
            // ================= 1. 初次进入 MANUAL 时的捕捉 =================
            if (is_first_init) {
                // 捕捉小 Pitch 此时的陀螺仪绝对角度，作为升起时的水平镇定目标
                init_pitch2_target = HI12.GetAngle(1)*PI_/180; 
                is_first_init = false;
            }

            // ================= 2. Yaw 轴目标设定 =================
            gimbal_target.target_yaw_vel = yaw_control.filter(-DT7.get_right_x() * 1.5f);

            // ================= 3. 大 Pitch (Pitch1) 目标设定 =================
            gimbal_target.target_pitch1 = pitch1_transfore.filter(-103.0f * 3.14159265f / 180.0f);  
            gimbal_target.target_pitch1_vel = pitch1_transfore.getDerivative();

            // ================= 4. 小 Pitch (Pitch2) 分段目标设定 =================
            if (!is_standup_complete) 
            {
                // 【阶段一：升起中】锁死初始绝对角度，无视遥控器
                gimbal_target.target_pitch2_angle = init_pitch2_target;
                gimbal_target.target_pitch2_vel = 0.0f; 
            } 
            else 
            {
                // 【阶段二：升起完毕，摇杆积分速控 + 电机角度限幅】
                
                // 1. 单次触发逻辑：只在刚升起完毕的那一瞬间执行一次！
                if (just_standup) 
                {
                    // 同步此时电机的实际弧度，作为遥控器积分的起点
                    gimbal_target.target_pitch2_angle =HI12.GetAngle(1)*PI_/180; //rad
                    
                    // ⚠️ 用完立刻锁死这扇门，防止后续每 2ms 都被覆盖
                    just_standup = false; 
                }
                
                // 2. 读取遥控器原始杆量，加入软件死区防止手抖或零偏
                 
                
                // 3. 将遥控器杆量转化为角度增量 (0.0001f 是灵敏度系数，可根据手感微调)
                float rc_increment = DT7.get_right_y() * 0.001f; 
                
                // 4. 将增量累加到目标角度上
                gimbal_target.target_pitch2_angle += rc_increment;
                
                // 5. 核心限幅：绝对安全的位置卡死
                const float MOTOR_PITCH_MAX = 0.523598f;  // 往上抬的机械极限 
                const float MOTOR_PITCH_MIN = -0.523598f;  // 往下压的机械极限 (弧度)
                
                // std::clamp 保证目标角度绝对不可能飞出这两个极限值
                gimbal_target.target_pitch2_angle = std::clamp(gimbal_target.target_pitch2_angle, MOTOR_PITCH_MIN, MOTOR_PITCH_MAX);
            }
            break;

        case TRANSFORM:
            // 保留你的变形模式预留位置
            break;

        case VISION:
            // 保留你的视觉模式预留位置
            break;

        default:
            // ================= 默认状态复位 (等同于 STOP) =================
            is_first_init = true;
            is_standup_complete = false;
            just_standup = true; // ⚠️ 重置单次触发标志位
            
            gimbal_target.target_yaw_vel = 0.0f;
            gimbal_target.target_yaw_angle = MotorJ4310.getAngleDeg(2);
            gimbal_target.target_pitch1 = -MotorJ4340.getAngleDeg(1) - 103.0f;
            gimbal_target.target_pitch2_vel = 0.0f;
            gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
            break;
    }
}


/*云台部分--------------------------------------------------------------------*/

/**
 * @brief 云台停止模式控制函数
 * 
 * 关闭电机使能并重置控制器参数
 */
void gimbal_stop()
{
    yaw_adrc.Reset();
    pitch2_transform_angle_pid.reset();
    pitch2_transform_velocity_pid.reset();
    pitch2_adrc.Reset();
     

    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch1 = 0.0f;
    gimbal_output.out_pitch2 = 0.0f;
}

void gimbal_manual()
{   
   // =========================================================
    // 1. 大 Pitch (Pitch1) 升起控制与状态检测
    // =========================================================
    float current_p1_enc = MotorJ4340.getAngleDeg(1);
    
    // 判断大 Pitch 是否升起到位（目标为 -103度，容差 5度）
    const float STANDUP_GOAL = -103.0f; 
    const float TOLERANCE = 5.0f;
    if (std::abs(current_p1_enc - STANDUP_GOAL) < TOLERANCE) {
        is_standup_complete = true;
    }

    // 大 Pitch 重力前馈计算 (保留你原有的逻辑)
    gravity_forward_pitch1.GravityFeedforward(-current_p1_enc - 103.0f);
    float G_feedforward = gravity_forward_pitch1.getFeedforward();
    
    // 大 Pitch 最终力矩输出限幅
    gimbal_output.out_pitch1 = std::clamp(G_feedforward, -9.0f, 9.0f);
   
    
    
       // =========================================================
    // 2. 小 Pitch (Pitch2) 分段闭环控制
    // =========================================================
    float current_imu_pitch = HI12.GetAngle(1)*PI_/180; //rad
    // 获取低通滤波后的干净陀螺仪角速度
    float current_imu_gyro_y = imu_pitch2.filter(HI12.GetGyroRad(1)); 
    float current_imu_gyro_z = HI12.GetGyroRad(2);
   
    if (!is_standup_complete) 
    {
        // 【阶段一：升起中】使用 角度 + 速度 双闭环，死守水平姿态
        pitch2_transform_angle_pid.UpDate(gimbal_target.target_pitch2_angle, current_imu_pitch); 
        float target_vel = pitch2_transform_angle_pid.getOutput();

         float v_output_pitch2 = pitch2_adrc.LADRC_1(target_vel, HI12.GetGyroRad(1));
        // pitch2_transform_velocity_pid.UpDate
        gravity_forward_pitch2.GravityFeedforward(current_imu_pitch);
        float G_pitch2_1 = gravity_forward_pitch2.getFeedforward();
           
        gimbal_output.out_pitch2 = std::clamp(v_output_pitch2 + G_pitch2_1 , -7.0f, 7.0f);
    } 
    else 
    {
        
       
        float control_pitch2_output = pitch2_Angle_pid.UpDate(gimbal_target.target_pitch2_angle, current_imu_pitch);
        //  float pitch2_angle_out = pitch2_adrc.LADRC_1(control_pitch2_output, HI12.GetGyroRad(1));
        
        // 重力前馈必须保留
        gravity_forward_pitch2.GravityFeedforward(HI12.GetAngle(1));
        float G_pitch2_2 = gravity_forward_pitch2.getFeedforward();  
        gimbal_output.out_pitch2 = std::clamp(control_pitch2_output , -7.0f, 7.0f);

        // --- Yaw 控制保持不变 ---
        float control_yaw_output = yaw_adrc.LADRC_1(gimbal_target.target_yaw_vel, current_imu_gyro_z);
        gimbal_output.out_yaw = std::clamp(control_yaw_output , -7.0f, 7.0f);
    }
}


void gimbal_vision()
{

}


void gimbal_transform()
{
    
}

/**
 * @brief 云台主控制循
 * 
 * @param left_sw 遥控器左开关状
 * @param right_sw 遥控器右开关状
 * @param is_online 设备在线状
 */
void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online, false, false);    // 最后一个传参要改成键鼠的键�?
    gimbal_fsm.TIM_Update();

    SetTarget_Gimbal();

    switch(gimbal_fsm.Get_Now_State()) 
    {
        case STOP:      // 停止模式
            gimbal_stop();
            break;
        case MANUAL:    // 普通模�?
            gimbal_manual();
            break;
        case VISION:    // 视觉模式
            gimbal_manual();    

            //gimbal_vision();
            break;
        case TRANSFORM: // 变形模式
            gimbal_transform();
            break;
        default:        // 默认模式（停止）
            gimbal_stop();
            break;
    }
}




/* 控制任务部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 控制任务主函�?
 * 
 * @param argument 任务参数
 */
extern "C"{
void Control(void const * argument)
{
    // 初始化蜂鸣器管理器；初始�?310到失能；初始化状态机
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
     
    static uint8_t control_tick = 0;
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请�?
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(),  check_online());
        // 1kHz 采样 (捕捉信号)
        
        
        osDelay(2);
    } 
}
}



