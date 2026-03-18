#include "vision.h"
#include "Remote_control.h"
#include "struct_typedef.h"
#include "string.h"
#include "INS.h"
#include "tim.h"

#include "gimbal.h"
#include "Shoot.h"




#ifdef __cplusplus
extern "C" {
#endif

#include "CRC8_CRC16.h"
#ifdef __cplusplus
};
#endif


// #include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern imu_can_ms imu_can;

uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

extern RC_ctrl_t rc_ctrl;
extern INS imu;
extern Gimbal gimbal;
extern Shoot shoot;

//角度初始化补偿
float Vision_Comps_Yaw = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;           //固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST; //根据距离补偿


//是否识别到装甲板
bool_t if_identify_target = FALSE;

//视觉误差判断
uint16_t yaw_para_rv2 = 125;//125
uint16_t pitch_para_rv2 = 125;//150
fp32 error_pitch = 0.0f;//-0.05 ;
fp32 error_yaw = 0.0f;//0.01;
//fp32 q_g2b = [cos(pi/2),0,0,-cos((pi/2))];

extern bool auto_aim_flag;//火控状态,现在默认打开火控
extern Shoot shoot;


//滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime=0;

uint8_t error_ture_rv2_limit(float angle , float error);
//void quat_multiply(const fp32 q1[4], const fp32 q2[4], fp32 q_out[4]);
//void imu_quat_transform_to_B(fp32 imu_quat[4], fp32 q_B[4]);

void vision_init()
{
  usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);

}

/**
  * @brief  判断是否识别到装甲板
  * @param  void
  * @retval TRUE识别到   FALSE未识别到
  * @attention  为自瞄做准备
  */
bool_t vision_if_find_target(void)
{
  return if_identify_target;
}

/**
  * @brief 获取us级时间
  * @return us级时间
  * @note 滴答定时器分频在此函数处理好像会损失精度
  */
uint64_t getSysTimeUs(void)
{
	uint64_t ms;
	uint64_t value;
	ms = sysTickUptime;
	value = (ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD) / MPRE;
	return value;
}



/*---------------------------------------------------- rv2 -----------------------------------------------------------*/

#define VISION_READ_LEN_PACKED_PC 30 //接受数据包长度

//extern bool auto_aim_flag;

// 添加计数器变量定义
uint32_t target_detect_count = 0;
uint32_t shoot_detect_count = 0;

VisionSendData_rv2_t VisionSendData_rv2[4]; //君瞄发送结构体

VisionRecvData_rv2_t VisionRecvData_rv2;    //君瞄接收结构体

uint8_t vision_send_pack_rv2[sizeof(VisionSendData_rv2)] = {0};

void vision_send_data_rv2()
{
   
  for(int i = 0;i < 4;i++)
  {
      VisionSendData_rv2[i].BEGIN = VISION_BEGIN;
      VisionSendData_rv2[i].mode = 3;
      VisionSendData_rv2[i].q[0] = imu_can.quet[0];
      VisionSendData_rv2[i].q[1] = imu_can.quet[1];
      VisionSendData_rv2[i].q[2] = imu_can.quet[2];
      VisionSendData_rv2[i].q[3] = imu_can.quet[3];
      VisionSendData_rv2[i].yaw = gimbal.gimbal_yaw_motor.absolute_angle;//imu.INS_angle[0];
      VisionSendData_rv2[i].yaw_vel = gimbal.gimbal_yaw_motor.motor_measure->speed_rpm;
      VisionSendData_rv2[i].pitch = gimbal.gimbal_pitch_motor.absolute_angle;//imu.INS_angle[1];
      VisionSendData_rv2[i].pitch_vel = gimbal.gimbal_pitch_motor.motor_measure->speed_rpm;
      VisionSendData_rv2[i].bullet_speed = 20.0f;
      VisionSendData_rv2[i].bullet_count = 100;
      VisionSendData_rv2[i].END = 0xFF;

      // VisionSendData_rv2[i].BEGIN = VISION_BEGIN;
      // VisionSendData_rv2[i].roll = 0;//imu.INS_angle[1];
      // VisionSendData_rv2[i].pitch = gimbal.gimbal_pitch_motor.absolute_angle; //imu.INS_angle[2];
      // VisionSendData_rv2[i].yaw = gimbal.gimbal_yaw_motor.relative_angle;// imu.INS_angle[0];//imu.INS_angle[0];
      // VisionSendData_rv2[i].END = 0xFF;

    memcpy(vision_send_pack_rv2, &VisionSendData_rv2,sizeof(VisionSendData_rv2));
  }
  HAL_UART_Transmit(&huart1, vision_send_pack_rv2, sizeof(vision_send_pack_rv2), 0xFFF);

}

TJU_vision_send_imu imu_send;








float debug_int;
void vision_read_data_rv2(uint8_t *ReadFormUart)
{
//    //判断帧头数据是否为0xff
   if (verify_CRC16_check_sum(ReadFormUart, VISION_READ_LEN_PACKED_PC))
   {

     //接收数据拷贝
     memcpy(&VisionRecvData_rv2, ReadFormUart, VISION_READ_LEN_PACKED_PC);
    
      VisionRecvData_rv2.pitch = (VisionRecvData_rv2.pitch - gimbal.gimbal_pitch_motor.absolute_angle);
      VisionRecvData_rv2.yaw = (VisionRecvData_rv2.yaw - gimbal.gimbal_yaw_motor.absolute_angle);

      if(VisionRecvData_rv2.target == 1 || VisionRecvData_rv2.target == 2)
      {  if_identify_target = TRUE;} // 识别到装甲板
     else
       {if_identify_target = FALSE;}// 未识别到装甲板
      
   }
}

uint8_t error_ture_rv2_limit(float angle , float error)
{
  if (error >  angle  > -error)
  {
    return 1;
    /* code */
  }
  return 0 ;
  
}


void vision_error_angle_rv2(float *yaw_angle_error, float *pitch_angle_error)
{
  
  *yaw_angle_error = (VisionRecvData_rv2.yaw + error_yaw )/ yaw_para_rv2;
  *pitch_angle_error = (VisionRecvData_rv2.pitch + error_pitch) / pitch_para_rv2;

  if (VisionRecvData_rv2.yaw == 0)
  {
    *yaw_angle_error = 0;
  }
  if (VisionRecvData_rv2.pitch == 0)
  {
    *pitch_angle_error = 0;
  }
}


//void quat_multiply(const fp32 q1[4], const fp32 q2[4], fp32 q_out[4]) {
//    // 提取四元数分量（w/x/y/z）
//    fp32 w1 = q1[0], x1 = q1[1], y1 = q1[2], z1 = q1[3];
//    fp32 w2 = q2[0], x2 = q2[1], y2 = q2[2], z2 = q2[3];

//    // 四元数乘法核心公式
//    q_out[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;  // w
//    q_out[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;  // x
//    q_out[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;  // y
//    q_out[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;  // z
//}

//// 你的业务逻辑：将陀螺仪四元数转换到B系
//void imu_quat_transform_to_B(fp32 imu_quat[4], fp32 q_B[4]) {
//    // 1. 定义G→B的偏移四元数（绕Z轴顺时针90°）
//    fp32 q_g2b[4] = {cos(M_PI/4), 0.0f, 0.0f, -sin(M_PI/4)};
//    
//    // 2. 核心操作：imu_quat 右乘 q_g2b → 结果存入q_B
//    quat_multiply(imu_quat, q_g2b, q_B);
//    
//    // 可选：归一化四元数（避免浮点误差累积，建议加上）
//    fp32 norm = sqrt(q_B[0]*q_B[0] + q_B[1]*q_B[1] + q_B[2]*q_B[2] + q_B[3]*q_B[3]);
//    if (norm > 1e-6f) {  // 避免除以0
//        q_B[0] /= norm;
//        q_B[1] /= norm;
//        q_B[2] /= norm;
//        q_B[3] /= norm;
//    }
//}

