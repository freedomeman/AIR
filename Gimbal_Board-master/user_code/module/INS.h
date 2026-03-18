/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     NOV-04-2021     summerpray       1. doing
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INS_H
#define INS_H

#ifdef __cplusplus
extern "C"{
#endif

#include <string.h>
#include "Pid.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "ahrs.h"

#ifdef __cplusplus
}
#endif

#include "Communicate.h"
#include "detect_task.h"

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100ԭʼ�����ڻ�����buf��λ��
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //�¶ȿ���PID��kp
#define TEMPERATURE_PID_KI 0.2f    //�¶ȿ���PID��ki
#define TEMPERATURE_PID_KD 0.0f    //�¶ȿ���PID��kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //�¶ȿ���PID��max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //�¶ȿ���PID��max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500�����¶ȵ�����TIM������ֵ������PWM���Ϊ MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //����ʼ���� delay һ��ʱ��

//����C��̶���ʽ,�ٴν���������
#define INS_YAW_ADDRESS_OFFSET    0
#define INS_ROLL_ADDRESS_OFFSET   2
#define INS_PITCH_ADDRESS_OFFSET  1



#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \




class INS{
public:

/*******************************************(C) �����ǻ������� ***********************************************/
    bmi088_real_data_t bmi088_real_data;                            //IMU���ݴ洢
    ist8310_real_data_t ist8310_real_data;                          //���������ݴ洢
    
    fp32 INS_gyro[3];
    fp32 INS_accel[3];
    fp32 INS_mag[3];
    fp32 INS_quat[4];
    fp32 INS_angle[3];                                              //euler angle, unit rad.ŷ���� ��λ rad

    //���ٶȼƵ�ͨ�˲�
    fp32 accel_fliter_1[3];
    fp32 accel_fliter_2[3];
    fp32 accel_fliter_3[3];
/*******************************************(C) �����ǻ������� ***********************************************/

/*******************************************(C) ��������ӿ� ************************************************/
    
    SPI_HandleTypeDef hspi1;                                        //��������ӿ�
/*******************************************(C) ��������ӿ� ************************************************/

/*******************************************(C) �����Ƿ��ز��� ***********************************************/
    const fp32 *get_INS_quat_point(void);                                 //��ȡ��Ԫ��
    const fp32 *get_INS_angle_point(void);                                //��ȡŷ����,0:yaw, 1:pitch, 2:roll ��λ rad
    const fp32 *get_gyro_data_point(void);                                //��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ rad/s
    const fp32 *get_accel_data_point(void);                               //��ȡ���ٶ�,0:x��, 1:y��, 2:roll�� ��λ m/s2
/*******************************************(C) �����Ƿ��ز��� ***********************************************/

/*************************************************(C) PID *************************************************/
    Pid imu_temp_pid;                                               //��������ʱPID
    fp32 temperature_fp32;
/*************************************************(C) PID *************************************************/
    float timing_time;                               //tast run time , unit s.�������е�ʱ�� ��λ s
    void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3],bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

    void init(void);                                                //������Ӳ����ʼ��
    void INS_Info_Get(void);                                        //���������ݴ���
};



/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         none
  */
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
 * @brief          open the SPI DMA accord to the value of imu_update_flag
 * @param[in]      none
 * @retval         none
 */
/**
 * @brief          ����imu_update_flag��ֵ����SPI DMA
 * @param[in]      temp:bmi088���¶�
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

extern INS imu;

#endif
