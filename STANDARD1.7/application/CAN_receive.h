/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN 	hcan1
#define SHOOT_CAN 	hcan2
#define CAP_CAN		hcan1


#define CAN_FEEDBACK_FREAM_ID_A       0x11
#define CAN_FEEDBACK_FREAM_ID_B       0x12
#define CAN_CTRL_FREAM_ID             0x21        //CAN帧ID号

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
		CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
	
		CAN_SHOOT_ALL_ID = 0x1FF,
	CAN_3508_LEFT_ID = 0x205,
		CAN_3508_RIGHT_ID = 0x206,
		CAN_TRIGGER_MOTOR_ID = 0x207,
	
		CAP_ID = 0x211,
    
} can_msg_id_e;

typedef enum
{
    BATTERY = 1,
    CAPACITY,
    OUT_OFF,
}power_source_enum; 

//RM 电机数据
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
    int16_t invot;
    int16_t capvot;
    int16_t current;
    int16_t power;
} cap_measure_t;

typedef struct
{
    fp32 voltage;
    fp32 cuttent;
    fp32 power;
} pm_measure_t;


typedef struct 
{
    uint16_t    input_voltage;      //输入电压
    uint16_t    current;            //输入电流
    uint16_t    cap_voltage;        //电容电压
    uint8_t     p_set;              //设定功率
    uint8_t     crc_checksum;
}can_feedback_a_typedef;  //CAN反馈数据A

typedef struct 
{
    uint16_t    output_voltage;     //输出电压
    uint8_t     power_source:7;       //电源来源
    uint8_t     out_auto_en:1;     //控制输出是否自动控制
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                
    uint8_t     nc4;                //空
    uint8_t     crc_checksum;
}can_feedback_b_typedef;  //CAN反馈数据B

typedef struct 
{
    uint8_t     p_set;                  //设定功率
    uint8_t     power_source:7;           //控制电源来源-1前级电源  2电容组  3输出关闭
    uint8_t     out_auto_en:1;     //控制输出是否自动控制
    uint16_t    freq_feedback:15;       //反馈频率，默认100
    uint16_t    wireless_en:1;          //无线开关
    uint8_t     nc1;                
    uint8_t     nc2;                
    uint8_t     nc3;                    //空
    uint8_t     crc_checksum;
}can_control_typedef;  //CAN控制数据

extern cap_measure_t get_cap;
//extern can_feedback_a_typedef get_capA;
//extern can_feedback_b_typedef get_capB;
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      rev1: (0x207) 保留，电机控制电流
  * @param[in]      rev2: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2);

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      s0: (0x201) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      s1: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      s2: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_shoot(int16_t s0, int16_t s1, int16_t s2, int16_t trigger);


extern void CAN_cmd_cap(int16_t temPower);
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_2006_measure_point(void);
		
/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_left_measure_point(void);

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_can_3508_right_measure_point(void);
#endif
