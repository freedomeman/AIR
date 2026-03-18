#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"

#include "struct_typedef.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define SHOOT_CAN hcan2
#define GIMBAL_CAN hcan1
#define BOARD_COM_CAN hcan2

#define DM_LOSE_ERROR_CNT   10

// 补充MIT电机KP/KD范围（原有结构体中缺失）
#define p_min -12.56f    // Radians
#define p_max 12.56f        
#define v_min -6.04f    // Rad/s
#define v_max 6.04f
#define kp_min 0.0f     // N-m/rad
#define kp_max 500.0f
#define kd_min 0.0f     // N-m/rad/s
#define kd_max 5.0f
#define t_min -0.84f
#define t_max 0.84f

// 参数范围限制宏（对齐原有代码风格）
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))



//云台电机编号
enum gimbal_motor_id_e
{
    //底盘动力电机接收
    YAW_MOTOR = 0,
    PITCH_MOTOR,
};

//发射机构电机编号
enum shoot_motor_id_e
{
    //底盘动力电机接收
    LEFT_FRIC_MOTOR = 0,
    RIGHT_FRIC_MOTOR,
    TRIGGER_MOTOR,
    COVER_MOTOR,
};

typedef enum
{
    //发射机构电机接受ID CAN1
    CAN_LEFT_FRIC_MOTOR_ID = 0x201,
    CAN_RIGHT_FRIC_MOTOR_ID = 0x202,
    CAN_TRIGGER_MOTOR_ID = 0x203,
    CAN_COVER_MOTOR_ID = 0X204,
    CAN_SHOOT_ALL_ID = 0x200,

    //云台电机接收ID CAN1
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PITCH_MOTOR_ID = 0x206,
    CAN_PITCH_DMMOTOR_ID = 0x00,
    CAN_PITCH_GIMOTOR_ID = 0x02,
    CAN_YAW_GIMOTOR_ID = 0x01,
    CAN_GIMBAL_ALL_ID = 0x1FF,

    //板间通信ID
    CAN_RC_BOARM_COM_ID = 0x301,
    CAN_GIMBAL_BOARD_COM_ID = 0x302,
    CAN_COOLING_BOARM_COM_ID = 0x303,
    CAN_17MM_SPEED_BOARD_COM_ID = 0x304,
    CAN_IMU_QUET_BORAM_COM_ID = 0x101,
    CAN_IMU_ANGLE_BORAM_COM_ID = 0x102,
    CAN_IMU_OPEN_BORAM_COM_ID = 0x303
} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

typedef struct
{
    uint16_t pos_tmp;
    uint16_t kp_tmp;
    uint16_t tor_tmp;
    uint16_t vel_tmp;
    uint16_t kd_tmp;
	float pos_set;
	float vel_set;
	float tor_set;
	float kp_set;
	float kd_set;
    float P_MIN;
    float P_MAX;
    float V_MIN;
    float V_MAX;
    float T_MIN;
    float T_MAX;
} dmmotor_measure_t;

// GIM电机数据结构体
typedef struct
{
    // 电机基本状态
    uint16_t last_ecd;          // 上次编码器值
    uint16_t ecd;               // 当前编码器值
    int16_t speed_rpm;          // 转速 RPM
    int16_t given_current;      // 给定电流
    uint8_t temperate;          // 温度
    
    // GIM电机特有数据（根据协议3.2.7响应格式）
    uint8_t res_code;           // 响应码
    float actual_torque;        // 实际力矩 N.m
    float actual_speed;         // 实际速度 RAD/s
    float actual_position;      // 实际位置 RAD
    uint8_t fault_status;       // 故障状态
    
    // 控制状态
    bool is_running;            // 运行状态
    uint32_t last_update_time;  // 最后更新时间

    uint16_t pos_tmp;
    uint16_t vel_tmp;
    float pos_set;
    float vel_set;
} GimbalGimMotor;

//云台发送数据结构体
typedef struct
{
    //遥控器数据
    int16_t ch_0;
    int16_t ch_2;
    int16_t ch_3;
    uint16_t v;

    //云台状态
    uint8_t s0;
    uint8_t gimbal_behaviour;
    fp32 gimbal_yaw_angle;
} gimbal_send_t;

//云台接收数据结构体
typedef struct
{
    //测试热量及ID
    uint16_t id1_17mm_cooling_limit; //17mm测速热量上限
    uint16_t id1_17mm_cooling_rate;  //17mm测速热量冷却
    uint16_t id1_17mm_cooling_heat;  //17mm测速实时热量
    uint8_t color;                   //判断红蓝方
    uint8_t robot_id;                //机器人编号

    //测速速度及底盘模式
    uint16_t id1_17mm_speed_limit; //17mm测速射速上限
    uint16_t bullet_speed;        //17mm测速实时射速

    uint8_t chassis_behaviour;

} gimbal_receive_t;

class imu_can_ms
{
    public:
        fp32 quet[4];
        fp32 angle[4];
};


class Can_receive
{

public:
    //云台电机反馈数据结构体
    motor_measure_t gimbal_motor[2];
    dmmotor_measure_t gimbal_dmmotor;
    GimbalGimMotor gimbal_gimmotor[2];  // 需要定义电机数量
    //发射机构电机反馈数据结构体
    motor_measure_t shoot_motor[4];

    //发送数据结构体
    CAN_TxHeaderTypeDef can_tx_message;
    uint8_t can_send_data[8];

    //板间通信
    //云台接收信息
    gimbal_receive_t gimbal_receive;
    //云台发送
    gimbal_send_t gimbal_send;

    void init();

    /*-------------------云台电机数据接收--------------------*/
    void get_gimbal_motor_measure(uint8_t num, uint8_t data[8]);
    void get_gimbal_dmmotor_measure( uint8_t data[8]);
    void can_cmd_gimbal_motor(int16_t yaw, int16_t pitch, int16_t empty1, int16_t empty2);
    void can_cmd_gimbal_dmmotor(float p_set , float v_set , float t_set , uint16_t kp_set , uint16_t kd_set);
    const motor_measure_t *get_gimbal_motor_measure_point(uint8_t i);

    /*-------------------发射机构电机数据接收--------------------*/
    void get_shoot_motor_measure(uint8_t num, uint8_t data[8]);
    void can_cmd_shoot_motor_motor(int16_t left_fric, int16_t right_fric, int16_t tigger, int16_t cover); //动力电机数据
    void can_cmd_shoot_motor_reset_ID();
    const motor_measure_t *get_shoot_motor_measure_point(uint8_t i);

    /*-------------------云台达妙电机使能--------------------*/
    void DM_Enable(uint8_t DM_id );
    void Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7);
    
    /*-------------------GIM电机力矩控制--------------------*/
    void can_cmd_torque_control(uint8_t id, float torque);
    void can_cmd_motor_start(uint8_t id);
    void can_cmd_motor_start_mit(uint8_t id);
    //void CanGIM_Cmd(uint8_t id , uint8_t cmd);
    void get_gimbal_gimmotor_measure(uint8_t num, uint8_t data[8]);

    /*-------------------GIM电机MIT控制--------------------*/
    void Can1Comm_ControlCmd(uint8_t id, uint8_t cmd);
    void can_cmd_mit_gimmotor(uint16_t id, float f_p, float f_v, float f_kp, float f_kd, float f_t);
    void get_mit_gimmotor_measure(uint8_t num, uint8_t data[8]);
   

    /*-------------------板间通信函数--------------------*/
    void receive_cooling_and_id_board_com(uint8_t data[8]);
    void receive_17mm_speed_and_mode_board_com(uint8_t data[8]);
    void send_rc_board_com(int16_t ch_0, int16_t ch_2, int16_t ch_3, uint16_t v);            //发送遥控器数据
    void send_gimbal_board_com(uint8_t s0, uint8_t gimbal_behaviour, fp32 gimbal_yaw_angle); //发送云台模式及状态
    uint8_t DM_disenble_dog(void);
    void get_imu_quet(uint8_t rx_data[8]);
    void get_imu_angle(uint8_t rx_data[8]);
    void send_imu_open_flag(void);
};


#endif
