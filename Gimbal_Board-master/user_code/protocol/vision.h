#ifndef __VISION_H
#define __VISION_H

#include "main.h"
#include "struct_typedef.h"

#include "bsp_usart.h"

#define VISION_BUFFER_LEN 50
#define NOW 0
#define LAST 1

#define ERROR_RV2_ANGLE	0.000001f

#define ATTACK_NONE 0 //不识别
#define ATTACK_RED 1  //识别红方
#define ATTACK_BLUE 2 //识别蓝方

#define VISION_DATA_ERROR 0	  //视觉数据错误
#define VISION_DATA_CORRECT 1 //视觉数据错误

#define VISION_LEN_HEADER 3		  //帧头长
#define VISION_LEN_DATA 15		  //数据段长度,可自定义
#define VISION_SEND_LEN_PACKED 16 //发送数据包长度
#define VISION_READ_LEN_PACKED 18 //接受数据包长度

#define VISION_OFF (0x00)			  //关闭视觉
#define VISION_RED (0x01)			  //识别红色
#define VISION_BLUE (0x02)			  //识别蓝色
#define VISION_RBUFF_ANTI (0x03)	  //红逆 大符
#define VISION_BBUFF_ANTI (0x04)	  //蓝逆 大符
#define VISION_RBUFF_CLOCKWISE (0x05) //红顺 大符
#define VISION_BBUFF_CLOCKWISE (0x06) //蓝顺 大符
#define VISION_RBUFF_STAND (0x07)	  //红 小符
#define VISION_BBUFF_STAND (0x08)	  //蓝 小符

//起始字节,协议固定为0xA5
#define VISION_BEGIN (0xA5) //可更改
#define VISION_END (0xFF)	//帧尾

/*-------视觉分辨率预编译--------*/
#define VISION_MID_YAW 444	 // 640
#define VISION_MID_PITCH 500 // 360

/*------------------自瞄预编译,角度初始化补偿------------------------*/
#define COMPENSATION_YAW 0
#define COMPENSATION_PITCH 0
#define COMPENSATION_PITCH_DIST 0


#define TICK_PER_SECOND 1000
#define TICK_US	(1000000/TICK_PER_SECOND)

//滴答定时器再分频，MPRE次中断为1ms
#define MPRE 4

typedef enum
{
	VISION_MANU = 0,
	VISION_BUFF = 1,
	VISION_AUTO = 2,
} VisionActData_t; //视觉模式选择


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

extern uint8_t Vision_Buffer[2][VISION_BUFFER_LEN]; //视觉数据暂存

#ifdef __cplusplus
extern "C" {
#endif
void vision_send_data_rv2();
#ifdef __cplusplus
};
#endif

extern void vision_init();

bool_t vision_if_find_target();

uint64_t getSysTimeUs(void);


/*--------------------------------------------------- rv2 ----------------------------------------------------------*/


typedef __packed struct
{
	uint8_t BEGIN;           // 0xA5
    uint8_t mode;	//模式：0空闲；1自瞄；2小符；3大符；
	float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;//弹速由裁判系统发出，视觉那边范围控制在10-25
    uint16_t bullet_count;//发弹剩余量
    uint8_t END;            // 0xFF

//	//相机标定参数需求
	//  uint8_t BEGIN;           // 0xA5
    // uint8_t mode;
	//  float roll;
	//  float pitch;
	//  float yaw;
	//  uint8_t crc;
	//  uint8_t END;

} VisionSendData_rv2_t;

#pragma pack(push, 1)
typedef __packed struct
{
	/* 头 */
	uint8_t head[2];
    uint8_t target;	//模式；0空闲；1自瞄不开火；2自瞄又开火
    float yaw;
    float yaw_vel;
	float yaw_acc;
    float pitch;
    float pitch_vel;
	float pitch_acc;
    uint16_t crc16;

} VisionRecvData_rv2_t;
#pragma pack(pop)

void vision_read_data_rv2(uint8_t *ReadFormUart);

#ifdef __cplusplus
extern "C" {
#endif

void vision_send_data_rv2();

#ifdef __cplusplus
};
#endif

extern VisionSendData_rv2_t VisionSendData_rv2[4]; //rv2发送结构体

extern VisionRecvData_rv2_t VisionRecvData_rv2;    //rv2接收结构体

extern void vision_error_angle_rv2(float *yaw_angle_error, float *pitch_angle_error);


extern bool tracking_state_rv2;

// 添加识别目标计数器
extern uint32_t target_detect_count;

/*----------------------------------------------导航-------------------------------------------------*/

typedef __packed struct
{
	uint8_t header;

	fp32 linear_x;
	fp32 linear_y;
	fp32 linear_z;
	
	fp32 vacant1;
	fp32 vacant2;
	float angular_z;
	uint8_t mode;

}SendPacketTwist_t;


typedef __packed struct TJU_vision_receive_ctrl
{
	uint8_t header;
	uint8_t ID;
	uint8_t data_lengs;

	uint8_t ctrl_cmd;
	uint8_t shoot_cmd;
	int16_t yaw;
	int16_t pitch;
	int16_t H_dis;

	uint16_t CRC16_send;
	uint8_t trailer;
	/* data */
};

typedef __packed struct TJU_vision_send_imu
{
    uint8_t header;       // 0xA5
    uint8_t ID;           // 0x01
    uint8_t data_length;  // 固定8字节
	typedef __packed struct data
	{
		int16_t qw;           // 四元数w分量（放大10000倍）
    	int16_t qx;           // 四元数x分量（放大10000倍）
    	int16_t qy;           // 四元数y分量（放大10000倍）
    	int16_t qz;           // 四元数z分量（放大10000倍）
	};


    uint16_t CRC16;       // CRC16校验值
    uint8_t trailer;      // 0xFF
};



#endif
