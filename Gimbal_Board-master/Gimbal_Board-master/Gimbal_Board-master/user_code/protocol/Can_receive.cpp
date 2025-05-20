#include "Can_receive.h"

#include "cmsis_os.h"
#include "main.h"

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


int float_to_uint(float x_float, float x_min, float x_max, int bits);
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

void Can_receive::get_gimbal_dmmotor_measure( uint8_t data[8])
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


int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
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

