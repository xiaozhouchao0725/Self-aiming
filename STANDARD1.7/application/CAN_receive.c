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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

//#include "crc8_crc16.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机 4 3508电机;
		4: yaw云台电机 6020电机;5:pitch云台电机 6020电机; 6:2006摩擦轮 S0 7:3508摩擦轮left; 8:3508摩擦轮right 9:拨弹电机 3508电机; */
motor_measure_t motor_chassis[10];
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

cap_measure_t get_cap;
//can_feedback_a_typedef get_capA;
//can_feedback_b_typedef get_capB;
	
static CAN_TxHeaderTypeDef  cap_tx_message;
static uint8_t              cap_can_send_data[8];
		
fp32 angle;
	
////CAN_TxHeaderTypeDef  cap_tx_message = {0};
//can_control_typedef cap_data = {0};
//uint32_t cap_send_mail_box;
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
		if(hcan==&hcan1)
		{	
			switch (rx_header.StdId)
			{
					case CAN_3508_M1_ID:
					case CAN_3508_M2_ID:
					case CAN_3508_M3_ID:
					case CAN_3508_M4_ID:
					{
							static uint8_t i = 0;
							//get motor id
							i = rx_header.StdId - CAN_3508_M1_ID;
							get_motor_measure(&motor_chassis[i], rx_data);
							detect_hook(CHASSIS_MOTOR1_TOE + i);
							break;
					}
					case CAN_YAW_MOTOR_ID:
					{
							get_motor_measure(&motor_chassis[4], rx_data);
							detect_hook(YAW_GIMBAL_MOTOR_TOE);
							break;
					}
					case CAN_PIT_MOTOR_ID:
					{
							get_motor_measure(&motor_chassis[5], rx_data);
							detect_hook(PITCH_GIMBAL_MOTOR_TOE);
							break;
					}
				case CAP_ID:
				{
						get_cap.invot = (int16_t)(rx_data[1] << 8 | rx_data[0]);        
						get_cap.capvot = (int16_t)(rx_data[3] << 8 | rx_data[2]);       
						get_cap.current = (int16_t)(rx_data[5] << 8 | rx_data[4]);      
						get_cap.power =(int16_t)(rx_data[7] << 8 | rx_data[6]); 
						break;
				}
//				case CAN_FEEDBACK_FREAM_ID_A:
//				{
//						get_capA.input_voltage = (int16_t)(rx_data[1] << 8 | rx_data[0]);       //输入电压					
//						get_capA.current = (int16_t)(rx_data[3] << 8 | rx_data[2]);             //输入电流
//						get_capA.cap_voltage = (int16_t)(rx_data[5] << 8 | rx_data[4]);         //电容电压
//						get_capA.p_set = rx_data[6];                                            //设定功率
//						get_capA.crc_checksum = rx_data[7];
//						break;
//				}
//				case CAN_FEEDBACK_FREAM_ID_B:
//				{
//						get_capB.output_voltage = (int16_t)(rx_data[1] << 8 | rx_data[0]);       				
//						get_capB.power_source = rx_data[2];
//						get_capB.out_auto_en = rx_data[3];
//						get_capB.nc1 = rx_data[4];
//						get_capB.nc2 = rx_data[5];
//						get_capB.nc3 = rx_data[6];
//						get_capB.nc4 = 0;
//						get_capB.crc_checksum = rx_data[7];
//						break;
//				}
					default:
					{
							break;
					}
			}
		}
		else if(hcan==&hcan2)
		{
			switch (rx_header.StdId)
			{
				case CAN_3508_LEFT_ID:
				{
						get_motor_measure(&motor_chassis[7], rx_data);
						detect_hook(FRIC_LEFT_MOTOR_TOE);
						break;
				}
				case CAN_3508_RIGHT_ID:
				{
						get_motor_measure(&motor_chassis[8], rx_data);
						detect_hook(FRIC_RIGHT_MOTOR_TOE );
						break;
				}
				case CAN_TRIGGER_MOTOR_ID:
				{
						get_motor_measure(&motor_chassis[9], rx_data);
						detect_hook(TRIGGER_MOTOR_TOE);
						break;
				}
				default:
				{
						break;
				}
			}
		}
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      rev: (0x207) 保留，电机控制电流
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t rev1, int16_t rev2)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (rev1 >> 8);
    gimbal_can_send_data[5] = rev1;
    gimbal_can_send_data[6] = (rev2 >> 8);
    gimbal_can_send_data[7] = rev2;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      s: (0x201) 2006电机控制电流, 范围 [-16384,16384]
  * @param[in]      left: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      right: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      trigger: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_shoot(int16_t left, int16_t right, int16_t trigger,int16_t s)
{
    uint32_t send_mail_box;
    shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
    shoot_tx_message.IDE = CAN_ID_STD;
    shoot_tx_message.RTR = CAN_RTR_DATA;
    shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = left >> 8;
    shoot_can_send_data[1] = left;
    shoot_can_send_data[2] = right >> 8;
    shoot_can_send_data[3] = right;
    shoot_can_send_data[4] = trigger >> 8;
    shoot_can_send_data[5] = trigger;
    shoot_can_send_data[6] = s>> 8;
    shoot_can_send_data[7] = s;

    HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

void CAN_cmd_cap(int16_t temPower)//超级电容
{
    uint32_t send_mail_box;
    cap_tx_message.StdId = 0x210;
    cap_tx_message.IDE = CAN_ID_STD;
    cap_tx_message.RTR = CAN_RTR_DATA;
    cap_tx_message.DLC = 0x08;
	  cap_can_send_data[0] = temPower >> 8;
    cap_can_send_data[1] = temPower;

    HAL_CAN_AddTxMessage(&CAP_CAN, &cap_tx_message, cap_can_send_data, &send_mail_box);
}

//void CAN_cmd_cap(uint8_t temPower)//超级电容
//{
//    cap_tx_message.StdId = CAN_CTRL_FREAM_ID;
//    cap_tx_message.IDE = CAN_ID_STD;
//    cap_tx_message.RTR = CAN_RTR_DATA;
//    cap_tx_message.DLC = 0x08;
//	cap_data.p_set = temPower;
//	cap_data.power_source = CAPACITY;
//	cap_data.out_auto_en = 0;
//	cap_data.wireless_en = 1;
//	cap_data.freq_feedback = 100;

//	append_CRC8_check_sum((uint8_t *)&cap_data,sizeof(cap_data));
//    HAL_CAN_AddTxMessage(&CAP_CAN, &cap_tx_message, (uint8_t *)&cap_data, &cap_send_mail_box);
//}

/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[9];
}

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_can_2006_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_can_3508_left_measure_point(void)
{
    return &motor_chassis[7];
}

/**
  * @brief          返回摩擦轮 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_can_3508_right_measure_point(void)
{
    return &motor_chassis[8];
}
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范? ?[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
