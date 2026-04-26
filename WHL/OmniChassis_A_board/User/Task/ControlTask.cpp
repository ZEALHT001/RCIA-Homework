#include "ControlTask.hpp"


extern bool alphabet[28];


#define PI_ 3.1415926535897932384626433832795
#define STANDARD -2.39347f  

Chassis_FSM chassis_fsm;    

#define MAX_SPEED 2 
  


float wheel_azimuth[4] = {-PI_/4, -3*PI_/4, 3*PI_/4, PI_/4};      
float wheel_direction[4] = {-3*PI_/4, 3*PI_/4, PI_/4, -PI_/4};   
Alg::CalculationBase::Omni_IK omni_ik(0.24f, 0.07f, wheel_azimuth, wheel_direction);        
Alg::CalculationBase::Omni_FK omni_fk(0.24f, 0.07f, 4.0f, wheel_azimuth, wheel_direction);  


ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 1鍙疯疆
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 2鍙疯疆
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 3鍙疯疆
    ALG::PID::PID(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 150.0f)      // 杞悜閫熷害pid 4鍙疯疆
};  

ALG::PID::PID follow_pid(5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);  // 閫熷害鐜痯id 鐢ㄤ簬搴曠洏璺熼殢


Alg::Utility::SlopePlanning omni_target[3] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // X
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // Y
    Alg::Utility::SlopePlanning(0.015f, 0.015f)       // Z
};

ControlTask chassis_target;     // 搴曠洏鐩爣
Output_chassis chassis_output;  // 搴曠洏杈撳嚭




bool check_online()
{
    bool isconnected = true;
    for(int i = 0; i < 4; i++)
    {
        if(!Motor3508.isConnected(i+1, i+1))
        {
            isconnected = false;
        }
    }

    if(/*!Cboard.isConnected() ||*/ !DT7.isConnected())
    {
        isconnected = false;
    }

    
    
    if(!isconnected)
    {
        return false;
    }

 
    
    return true;
}


void fsm_init()
{
    chassis_fsm.Init();
}

/**
 * @brief 底盘相对云台坐标系的平移解算（二维旋转矩阵）
 * * @param theta 云台当前反馈的 Yaw 轴角度（单位：弧度）
 * @param vx    笛卡尔坐标系下的 X 方向输入（通常对应遥控器的左右摇杆）
 * @param vy    笛卡尔坐标系下的 Y 方向输入（通常对应遥控器的前后摇杆）
 * @param phi   角度偏差补偿值（单位：弧度，用于微调或者机械安装误差补偿）
 * @param out_vx [输出指针] 计算旋转矩阵后，底盘坐标系下的 X 方向速度（前后）
 * @param out_vy [输出指针] 计算旋转矩阵后，底盘坐标系下的 Y 方向速度（左右）
 * @param psi   额外的相位角补偿（单位：弧度，用于特殊机动如小陀螺等）
 */
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi)
{
    // 将云台反馈角减去正前方的零位基准角，再叠加额外的补偿角
    theta = theta - STANDARD + phi;

    // 过零处理 (将角度归一化限制在 -PI 到 PI 之间)
    theta = fmod(theta, 2 * PI_);
    if (theta > PI_) theta -= 2 * PI_;
    else if (theta < -PI_) theta += 2 * PI_;

    // 预计算正弦和余弦值，包含额外的 psi 相位补偿
    float s = sinf(theta + psi);
    float c = cosf(theta + psi);
    
    // 遥控器控制量输入 (云台坐标系下)
    // 放大系数 4.5f，且注意遥控器的映射：vy(前后摇杆)对应机器人的X轴(前后)
    float raw_vx = 1.5f * vy;  // raw_vx是机器人云台坐标系下的前后方向
    float raw_vy = -1.5f * vx; // 横移方向取反，修正在云台坐标系下左右与遥控输入相反的问题
    
    // 核心：二维旋转矩阵计算
    // 将云台坐标系下的期望速度，通过旋转矩阵投影到底盘自身的局部坐标系下
    // [ cos(θ)  -sin(θ) ] * [ raw_vx ]
    // [ sin(θ)   cos(θ) ]   [ raw_vy ]
    *out_vx = raw_vx * c - raw_vy * s; 
    *out_vy = raw_vx * s + raw_vy * c;
}


/**
 * @brief 底盘跟随云台的角速度(w)规划
 * * 计算底盘偏航角(Yaw)与云台相对基准角度(STANDARD)之间的误差，
 * 并通过 PID 算法输出一个用于底盘旋转的角速度，使底盘始终朝向云台的正面。
 */
void CalculateFollow()
{
    // 计算跟随误差：标准基准角度(云台相对底盘的正前方编码器值) - 当前云台的实际反馈偏航角
    float follow_error = STANDARD - Cboard.GetYawAngle(); 
    
    // 角度归一化处理 (将误差限制在 -PI/2 到 PI/2，即 -90度 到 +90度之间)
    // 注意：这里的 1.5707963267f 就是 PI / 2
    // 如果误差超过 90 度，代码将其反转，这通常意味着底盘可以通过反向转动更近地到达目标，
    // 或者机械结构上允许底盘正反都能算作“跟随”（比如轮式步兵底盘前后对称）。
    while (follow_error > 1.5707963267f) follow_error -= 2 * 1.5707963267f;
    while (follow_error < -1.5707963267f) follow_error += 2 * 1.5707963267f;

    // 死区处理 (防止极小误差引起的底盘微小高频震荡，通常称为“点头”或“发抖”)
    // 0.01 弧度大约等于 0.57 度
    if(fabs(follow_error) < 0.01f) follow_error = 0.0f;
    
    // 将处理后的误差送入跟随 PID 控制器计算
    // 目标值为 0.0f（即期望误差为0），当前值为算出的误差量 follow_error
    // 返回值通常会赋值给底盘的期望角速度 target_vw
    follow_pid.UpDate(0.0f, follow_error);
}

/*
**底盘不跟随
*
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

void Follow_SetTarget(){
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
             CalculateFollow();
            float follow_pid_output = follow_pid.getOutput();//旋转
            chassis_target.target_rotation = std::clamp(follow_pid_output, -2.0f, 2.0f);
           }
}

/**
 * @brief 璁剧疆搴曠洏鐨勭洰鏍囧€?
 * 
 * 鏍规嵁褰撳墠宸ヤ綔妯″紡璁剧疆鐩稿簲鐨勭洰鏍囧€?
 */
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
 * @brief 底盘急停
 * 
 */
void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        
        wheel_pid[i].reset();
        
        chassis_output.out_wheel[i] = 0.0f;
    }
}

/**
 * @brief 底盘不跟随
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
 * @brief 底盘跟随
 * 
 *
 */
void chassis_follow() 
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
 * 
 * @param left_sw 左摇杆
 * @param right_sw 右摇杆
 * @param is_online 断联检测
 */
void main_loop_chassis(uint8_t left_sw, uint8_t right_sw, bool is_online,bool *alphabet) 
{   
   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online,alphabet);
    SetTarget();

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






/* 鎺у埗浠诲姟閮ㄥ垎 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 鎺у埗浠诲姟涓诲嚱鏁?
 * 
 * @param argument 浠诲姟鍙傛暟
 */
extern "C"{
void Control(void const * argument)
{
    // 鍒濆鍖栬渹楦ｅ櫒绠＄悊鍣?
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
    for(;;)
    {
        // 鏇存柊铚傞福鍣ㄧ鐞嗗櫒锛屽鐞嗛槦鍒椾腑鐨勫搷閾冭姹?
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
		// 	  SetTarget();
        // chassis_notfollow();
		bool dummy_alphabet[20] = {false};	
        main_loop_chassis(DT7.get_s1(), DT7.get_s2(), check_online(), dummy_alphabet);
       
        osDelay(1);
    } 
}
}






