#include "Can_receive.h"

#include "cmsis_os.h"
#include "main.h"
#include "string.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp_delay.h"

#ifdef __cplusplus
}
#endif
#include "bsp_can.h"
#include "can.h"


#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
uint8_t call_back_cnt=0;
imu_can_ms imu_can;


uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

void Can_receive::init()
{
    can_filter_init();
}

void Can_receive::get_gimbal_motor_measure(uint8_t num, uint8_t data[8])
{
    gimbal_motor[num].last_ecd = gimbal_motor[num].ecd;
    gimbal_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_motor[num].temperate = data[6];
}

uint8_t ERR = 0;

void Can_receive::get_gimbal_dmmotor_measure(uint8_t data[8])
{
    ERR = (data[0] & 0xF0) >> 4;
    gimbal_dmmotor.pos_tmp = (data[1]<<8)|data[2];
    gimbal_dmmotor.vel_tmp = (data[3]<<4)|(data[4]>>4);
    gimbal_dmmotor.tor_tmp = ((data[4]&0xF)<<8)|data[5];
    gimbal_dmmotor.pos_set = uint_to_float(gimbal_dmmotor.pos_tmp, gimbal_dmmotor.P_MIN, gimbal_dmmotor.P_MAX, 16); 
    gimbal_dmmotor.vel_set = uint_to_float(gimbal_dmmotor.vel_tmp, gimbal_dmmotor.V_MIN, gimbal_dmmotor.V_MAX, 16);
    gimbal_dmmotor.tor_set = uint_to_float(gimbal_dmmotor.tor_tmp, gimbal_dmmotor.T_MIN, gimbal_dmmotor.T_MAX, 16); 

    if (ERR == 1)
    {
        call_back_cnt =0;
        /* code */
    }
    
    
}

void Can_receive::get_mit_gimmotor_measure(uint8_t num, uint8_t data[8])
{

    gimbal_gimmotor[num].pos_tmp = (data[1]<<8)|data[2];
    gimbal_gimmotor[num].vel_tmp = (data[3]<<4)|(data[4]>>4);
    gimbal_gimmotor[num].pos_set = uint_to_float(gimbal_gimmotor[num].pos_tmp, p_min, p_max, 16); 
    gimbal_gimmotor[num].vel_set = uint_to_float(gimbal_gimmotor[num].vel_tmp, v_min, v_max, 12);

    
}



 void Can_receive::Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7)
 {
    static CAN_TxHeaderTypeDef packet;
    
    packet.StdId = ID;
    packet.IDE = CAN_ID_STD;
    packet.RTR = CAN_RTR_DATA;
    packet.DLC = 0x08;
    uint8_t payload[8];
    payload[0] = (uint8_t)data0;
    payload[1] = (uint8_t)data1;
    payload[2] = (uint8_t)data2;
    payload[3] = (uint8_t)data3;
    payload[4] = (uint8_t)data4;
    payload[5] = (uint8_t)data5;
    payload[6] = (uint8_t)data6;
    payload[7] = (uint8_t)data7;

    /*找到空的发送邮箱，把数据发送出去*/
	  if(HAL_CAN_AddTxMessage(hcan, &packet, payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet, payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet, payload, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

void Can_receive::DM_Enable(uint8_t DM_id )
{ 
    uint8_t Data_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};		//电机使能命令
    uint32_t send_mail_box;
    static CAN_TxHeaderTypeDef   Tx_Header;
    Tx_Header.StdId=DM_id;
	Tx_Header.ExtId=0;
	Tx_Header.IDE=0;
	Tx_Header.RTR=0;
	Tx_Header.DLC=0x08;

     HAL_CAN_AddTxMessage(&GIMBAL_CAN, &Tx_Header, Data_Enable, &send_mail_box);
}

/**
* @brief          
* @param[in]      
* @param[in]          
* @retval         none
*/
void Can_receive::can_cmd_gimbal_motor(int16_t yaw, int16_t pitch, int16_t empty1, int16_t empty2)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = yaw >> 8;
    can_send_data[1] = yaw;
    can_send_data[2] = pitch >> 8;
    can_send_data[3] = pitch;
    can_send_data[4] = empty1 >> 8;
    can_send_data[5] = empty1;
    can_send_data[6] = empty2 >> 8;
    can_send_data[7] = empty2;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


void Can_receive::can_cmd_gimbal_dmmotor(float p_set , float v_set , float t_set , uint16_t kp_set_m ,uint16_t kd_set_m)
{
    uint32_t send_mail_box;
    uint16_t pos_tmp , vel_tmp , tor_tmp , kp_tmp , kd_tmp;
    can_tx_message.StdId = CAN_PITCH_DMMOTOR_ID+0x01;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    pos_tmp = float_to_uint(p_set,  gimbal_dmmotor.P_MIN,  gimbal_dmmotor.P_MAX,  16);
    vel_tmp = float_to_uint(v_set,  gimbal_dmmotor.V_MIN,  gimbal_dmmotor.V_MAX,  12);
    tor_tmp = float_to_uint(t_set,  gimbal_dmmotor.T_MIN,  gimbal_dmmotor.T_MAX,  12);

    kp_tmp = kp_set_m;
    kd_tmp = kd_set_m;

    can_send_data[0] = ( pos_tmp >> 8);
    can_send_data[1] =  pos_tmp;
    can_send_data[2] = ( vel_tmp >> 4);
    can_send_data[3] = (( vel_tmp&0xF)<<4)|( kp_tmp>>8);
    can_send_data[4] =  kp_tmp;
    can_send_data[5] = ( kd_tmp >> 4);
    can_send_data[6] = (( kd_tmp&0xF)<<4)|( tor_tmp>>8);
    can_send_data[7] =  tor_tmp;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);

}

/**
  * @brief          返回云台电机 6020电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_gimbal_motor_measure_point(uint8_t i)
{
    return &gimbal_motor[i];
}

void Can_receive::get_shoot_motor_measure(uint8_t num, uint8_t data[8])
{
    shoot_motor[num].last_ecd = shoot_motor[num].ecd;
    shoot_motor[num].ecd = (uint16_t)(data[0] << 8 | data[1]);
    shoot_motor[num].speed_rpm = (uint16_t)(data[2] << 8 | data[3]);
    shoot_motor[num].given_current = (uint16_t)(data[4] << 8 | data[5]);
    shoot_motor[num].temperate = data[6];
}

/**
* @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
* @param[in]      left_fric: (0x201) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      right_fric: (0x202) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      tigger: (0x203) 3508电机控制电流, 范围 [-16384,16384]
* @param[in]      cover: (0x204) 3508电机控制电流, 范围 [-16384,16384]
* @retval         none
*/
void Can_receive::can_cmd_shoot_motor_motor(int16_t left_fric, int16_t right_fric, int16_t tigger, int16_t cover)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_SHOOT_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = left_fric >> 8;
    can_send_data[1] = left_fric;
    can_send_data[2] = right_fric >> 8;
    can_send_data[3] = right_fric;
    can_send_data[4] = tigger >> 8;
    can_send_data[5] = tigger;
    can_send_data[6] = cover >> 8;
    can_send_data[7] = cover;


    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void Can_receive::can_cmd_shoot_motor_reset_ID(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = 0x700;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;



    //HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


/**
  * @brief          返回云台电机 6020电机数据指针
  * @param[in]      i: 电机编号,范围[0,1]
  * @retval         电机数据指针
  */
const motor_measure_t *Can_receive::get_shoot_motor_measure_point(uint8_t i)
{
    return &shoot_motor[i];
}

void Can_receive::receive_cooling_and_id_board_com(uint8_t data[8])
{
    gimbal_receive.id1_17mm_cooling_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.id1_17mm_cooling_rate = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.id1_17mm_cooling_heat = (uint16_t)(data[4] << 8 | data[5]);
    gimbal_receive.color = (data[6]);
    gimbal_receive.robot_id = (data[7]);
}

void Can_receive::receive_17mm_speed_and_mode_board_com(uint8_t data[8])
{
    gimbal_receive.id1_17mm_speed_limit = (uint16_t)(data[0] << 8 | data[1]);
    gimbal_receive.bullet_speed = (uint16_t)(data[2] << 8 | data[3]);
    gimbal_receive.chassis_behaviour = data[4];
}

void Can_receive::send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v)
{
    //数据填充
    gimbal_send.ch_0 = ch_0;
    gimbal_send.ch_2 = ch_2;
    gimbal_send.ch_3 = ch_3;
    gimbal_send.v = v;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_RC_BOARM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = ch_0 >> 8;
    can_send_data[1] = ch_0;
    can_send_data[2] = ch_2 >> 8;
    can_send_data[3] = ch_2;
    can_send_data[4] = ch_3 >> 8;
    can_send_data[5] = ch_3;
    can_send_data[6] = v >> 8;
    can_send_data[7] = v;


    //HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


void Can_receive::send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle)
{
    int32_t temp_gimbal_yaw_angle = (int32_t)(gimbal_yaw_angle * 1000);

    //数据填充
    gimbal_send.s0 = s0;
    gimbal_send.gimbal_behaviour = gimbal_behaviour;
    gimbal_send.gimbal_yaw_angle = gimbal_yaw_angle;

    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_GIMBAL_BOARD_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = s0;
    can_send_data[1] = gimbal_behaviour;
    can_send_data[2] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 24);
    can_send_data[3] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 16);
    can_send_data[4] = (uint8_t)((int32_t)temp_gimbal_yaw_angle >> 8);
    can_send_data[5] = (uint8_t)((int32_t)temp_gimbal_yaw_angle);
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    //HAL_CAN_AddTxMessage(&BOARD_COM_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


//达妙失能看门狗
uint8_t Can_receive::DM_disenble_dog(void)
{
    call_back_cnt ++;
    if (call_back_cnt >= DM_LOSE_ERROR_CNT)
    {
        return 0;
        /* code */
    }
    return 1;
    
}


// void Can_receive::CanGIM_Cmd(uint8_t id , uint8_t cmd)
// {
//     uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
//     switch(cmd)
//     {
//         case CMD_MOTOR_MODE:
//             buf[7] = 0xFC;
//             break;
        
//         case CMD_RESET_MODE:
//             buf[7] = 0xFD;
//         break;
        
//         case CMD_ZERO_POSITION:
//             buf[7] = 0xFE;
//         break;
        
//         default:
//         return; /* 直接退出函数 */
//     }
//         uint32_t send_mail_box;
//     can_tx_message.StdId = id;
//     can_tx_message.IDE = CAN_ID_STD;
//     can_tx_message.RTR = CAN_RTR_DATA;
//     can_tx_message.DLC = 0x08;
//     can_send_data[0] = buf[0];
//     can_send_data[1] = buf[1];
//     can_send_data[2] = buf[2];
//     can_send_data[3] = buf[3];
//     can_send_data[4] = buf[4];
//     can_send_data[5] = buf[5];
//     can_send_data[6] = buf[6];
//     can_send_data[7] = buf[7];


//     HAL_CAN_AddTxMessage(&GIMABL_CAN, &can_tx_message, can_send_data, &send_mail_box);
    
// }

/**
 * @brief 启动电机命令（协议3.2.5）
 * @param id 电机CAN ID
 */
void Can_receive::can_cmd_motor_start(uint8_t id)
{
    uint32_t send_mail_box;
    // 设置CAN消息头
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    // 填充启动指令（0x91=MOTCTRL_CMD_START_MOTOR）
    can_send_data[0] = 0x91;
    memset(&can_send_data[1], 0, 7); // 其余字节填0
    // 发送启动指令
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::can_cmd_motor_start_mit(uint8_t id)
{
    uint32_t send_mail_box;
    
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    
    // MIT协议启动电机命令
    can_send_data[0] = 0xFF;     // 固定值
    can_send_data[1] = 0xFF;     // 固定值
    can_send_data[2] = 0xFF;     // 固定值
    can_send_data[3] = 0xFF;     // 固定值
    can_send_data[4] = 0xFF;     // 固定值
    can_send_data[5] = 0xFF;     // 固定值
    can_send_data[6] = 0xFF;     // 固定值
    can_send_data[7] = 0xFC;     // 启动电机命令字
    
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

/**
 * @brief 发送力矩控制指令到指定电机
 * @param id 电机CAN ID
 * @param torque 目标力矩值(N.m)
 */
void Can_receive::can_cmd_torque_control(uint8_t id, float torque)
{
    uint32_t send_mail_box;
    
    // ========== 前置检查：扭矩量程限制（匹配电机协议） ==========
    // 按例程扭矩量程[-225, 225]限制，需根据电机实际参数调整
    if (torque < -225.0f || torque > 225.0f) {
        torque = (torque > 0) ? 225.0f : -225.0f;
    }
    
    // ========== 1. 设置CAN消息头（保持原有配置） ==========
    can_tx_message.StdId = id;          // 电机控制指令ID（如0x201）
    can_tx_message.IDE = CAN_ID_STD;    // 标准帧（确认电机协议要求）
    can_tx_message.RTR = CAN_RTR_DATA;  // 数据帧（非远程帧）
    can_tx_message.DLC = 0x08;          // 固定8字节（匹配例程协议）
    
    // ========== 2. 取float扭矩的字节数组（安全方式，符合C标准） ==========
    uint8_t* torquePtr = (uint8_t*)(&torque); 
    uint32_t duration = 20; // 0=无限时间执行，非0则填对应ms值（24位范围）
    uint8_t* durationPtr = (uint8_t*)(&duration);
    
    // ========== 3. 填充指令帧（严格按大端序：高字节在前，低字节在后） ==========
    can_send_data[0] = 0x93;                     // 命令字：力矩控制（0x93=MOTCTRL_CMD_TORQUE_CONTROL）
    
    // 扭矩4字节填充（大端序）：torquePtr[3](MSB) → [1]，torquePtr[0](LSB) → [4]
    can_send_data[1] = torquePtr[0];             // Torque LSB
    can_send_data[2] = torquePtr[1];
    can_send_data[3] = torquePtr[2];
    can_send_data[4] = torquePtr[3];             // Torque MSB
    // 填充24位持续时间（低24位，小端序）
    can_send_data[5] = durationPtr[0];           // Duration LSB
    can_send_data[6] = durationPtr[1];
    can_send_data[7] = durationPtr[2];           // Duration 24bit MSB
    
    // ========== 4. 发送CAN消息（增加错误检查） ==========
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
    if (status != HAL_OK) {
        vTaskDelay(2);
        HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
        // 可选：添加发送失败处理（如日志、重发）
    }	 
}

/**
 * @brief 解析GIM电机力矩控制响应数据（移植示例MCResTorqueControl解析逻辑）
 * @param num 电机编号
 * @param data 接收到的8字节响应数据
 */
void Can_receive::get_gimbal_gimmotor_measure(uint8_t num, uint8_t data[8])
{

    // 保存上次编码器值（保留原有逻辑）
    gimbal_gimmotor[num].last_ecd = gimbal_gimmotor[num].ecd;
    
    // ===================== 移植示例解析逻辑 =====================
    // 1. 解析响应结果码（示例resBuf[1] = 响应码）
    gimbal_gimmotor[num].res_code = data[1];
    
    // 2. 解析电机温度（示例resBuf[2] = 温度，int8_t类型）
    gimbal_gimmotor[num].temperate = (int8_t)data[2];
    
    // 3. 解析16位位置原始值（示例逻辑：LSB字节序）
    uint16_t pos_int;
    uint8_t *tmp = (uint8_t *)&pos_int;
    tmp[0] = data[3];  // 位置低字节（示例resBuf[3]）
    tmp[1] = data[4];  // 位置高字节（示例resBuf[4]）
    // 转换为rad（示例公式：-12.5~12.5）
    gimbal_gimmotor[num].actual_position = (float)pos_int * 25.0f / 65535.0f - 12.5f;
    // 更新编码器值（保留原有逻辑，同步位置数据）
    gimbal_gimmotor[num].ecd = pos_int;

    // 4. 解析12位速度原始值（示例逻辑：符号位保留）
    int16_t speed_int = (int16_t)(((uint16_t)data[5] << 4) | (data[6] >> 4));
    // 转换为rad/s（示例公式：-65~65）
    gimbal_gimmotor[num].actual_speed = (float)speed_int * 130.0f / 4095.0f - 65.0f  +0.01587677;

    // 5. 解析12位扭矩原始值（示例逻辑：符号位保留）
    int16_t torque_int = (int16_t)(((uint16_t)(data[6] & 0x0F) << 8) | data[7]);
    // 转换为电流值（示例公式：-225~225，移除原有转矩常数/减速比）
    gimbal_gimmotor[num].actual_torque = (float)torque_int * 450.0f / 4095.0f - 225.0f;
    // ============================================================
    
    // 更新最后更新时间（保留原有逻辑）
    gimbal_gimmotor[num].last_update_time = HAL_GetTick();
    
}

/**
* @brief          发送MIT电机控制参数(P/V/Kp/Kd/T)
* @param[in]      id: MIT电机CAN ID, 范围 [0x000,0x7FF]
* @param[in]      f_p: 目标位置, 范围 [P_MIN, P_MAX] (rad)
* @param[in]      f_v: 目标速度, 范围 [V_MIN, V_MAX] (rad/s)
* @param[in]      f_kp: 位置增益, 范围 [KP_MIN, KP_MAX]
* @param[in]      f_kd: 速度增益, 范围 [KD_MIN, KD_MAX]
* @param[in]      f_t: 目标扭矩, 范围 [T_MIN, T_MAX] (N·m)
* @retval         none
*/
void Can_receive::can_cmd_mit_gimmotor(uint16_t id, float f_p, float f_v, float f_kp, float f_kd, float f_t)
{
    uint32_t send_mail_box;
    uint16_t p, v, kp, kd, t;
    
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    
    /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  p_min,  p_max);
    LIMIT_MIN_MAX(f_v,  v_min,  v_max);
    LIMIT_MIN_MAX(f_kp, kp_min, kp_max);
    LIMIT_MIN_MAX(f_kd, kd_min, kd_max);
    LIMIT_MIN_MAX(f_t,  t_min,  t_max);

    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      p_min,  p_max,  16);            
    v = float_to_uint(f_v,      v_min,  v_max,  12);
    kp = float_to_uint(f_kp,    kp_min, kp_max, 12);
    kd = float_to_uint(f_kd,    kd_min, kd_max, 12);
    t = float_to_uint(f_t,      t_min,  t_max,  12);
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    can_send_data[0] = p >> 8;
    can_send_data[1] = p & 0xFF;
    can_send_data[2] = v >> 4;
    can_send_data[3] = ((v & 0xF) << 4) | (kp >> 8);
    can_send_data[4] = kp & 0xFF;
    can_send_data[5] = kd >> 4;
    can_send_data[6] = ((kd & 0xF) << 4) | (t >> 8);
    can_send_data[7] = t & 0xff;
    
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}

void Can_receive::Can1Comm_ControlCmd(uint8_t id, uint8_t cmd)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = id;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    can_send_data[0] = 0xff;
    can_send_data[1] = 0xff;
    can_send_data[2] = 0xff;
    can_send_data[3] = 0xff;
    can_send_data[4] = 0xff;
    can_send_data[5] = 0xff;
    can_send_data[6] = 0xff;
    switch(cmd)
    {
        case 1:
            can_send_data[7] = 0xFC;
            break;
        
        case 2:
            can_send_data[7] = 0xFD;
        break;
        
        case 3:
            can_send_data[7] = 0xFE;
        break;
        
        default:
        return; /* 直接退出函数 */
    }
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &can_tx_message, can_send_data, &send_mail_box);
}


uint16_t float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}





void Can_receive::get_imu_quet(uint8_t rx_data[8])
{
    imu_can.quet[0] = ((fp32)(int16_t)((rx_data[0]<<8)|rx_data[1]))/32765;
    imu_can.quet[1] = ((fp32)(int16_t)((rx_data[2]<<8)|rx_data[3]))/32765;
    imu_can.quet[2] = ((fp32)(int16_t)((rx_data[4]<<8)|rx_data[5]))/32765;
    imu_can.quet[3] = ((fp32)(int16_t)((rx_data[6]<<8)|rx_data[7]))/32765;

}

void Can_receive::get_imu_angle(uint8_t rx_data[8])
{
    imu_can.angle[0] = ((fp32)(int16_t)((rx_data[0]<<8)|rx_data[1]))/10401;
    imu_can.angle[1] = ((fp32)(int16_t)((rx_data[2]<<8)|rx_data[3]))/10401;
    imu_can.angle[2] = ((fp32)(int16_t)((rx_data[4]<<8)|rx_data[5]))/10401;
}

void Can_receive::send_imu_open_flag(void)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_IMU_OPEN_BORAM_COM_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;

    can_send_data[0] = 1;
    can_send_data[1] = 0;
    can_send_data[2] = 0;
    can_send_data[3] = 0;
    can_send_data[4] = 0;
    can_send_data[5] = 0;
    can_send_data[6] = 0;
    can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &can_tx_message, can_send_data, &send_mail_box);
}




