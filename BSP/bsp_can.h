/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif
#include "mytype.h"
#include "can.h"

/*CAN发送或是接收的ID*/
typedef enum
{

	CAN_TxPY12V_ID 	        = 0x200,		//电机发送ID
	CAN_TxPY24V_ID	        = 0x1FF,		//电机发送ID

	CAN_MotorLF_ID 	        = 0x041,        //左前
	CAN_MotorRF_ID 	        = 0x042,		//右前
	CAN_MotorLB_ID 	        = 0x043,        //左后
	CAN_MotorRB_ID 	        = 0x044,		//右后
	
	//add by langgo
	CAN_3508Moto_ALL_ID     = 0x200,
    
	CAN_Motor1_ID        = 0x201,        //CAN2:M3508底盘电机1，和 CAN1:M3508攻击机构电机，ID:0X201
	CAN_Motor2_ID        = 0x202,        //CAN2:M3508底盘电机2，和 CAN1:M2006电机1，ID:0X202
	CAN_Motor3_ID        = 0x203,        //CAN2:M3508底盘电机3，和 CAN1:M2006电机2，ID:0X203
	CAN_Motor4_ID        = 0x204,        //CAN2:M3508底盘电机4，和 CAN1:M2006电机3，ID:0X204
    
    CAN_Motor5_ID        = 0x205,        //
    
    CAN_Motor6_ID        = 0x206,        //
    CAN_Motor7_ID        = 0x207,        //
    CAN_Motor8_ID        = 0x208,        //
       
	CAN_DriverPower_ID      = 0x80,
	
	CAN_HeartBeat_ID        = 0x156,
	
}CAN_Message_ID;

#define FILTER_BUF_LEN		5   
/*接收到的电机的参数结构体*/
typedef struct
{
	int16_t	 	speed_rpm;          //转子转速
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	        //abs angle range:[0,8191]
	uint16_t	offset_angle;       //初始编码器值
	int32_t		round_cnt;          //电机转子旋转圈数
	int32_t		total_angle;        //电机转子编码器总计数    2147483647  10位
    
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

/* Extern  ------------------------------------------------------------------*/
extern moto_measure_t  moto_chassis[];      //CAN2 底盘4个电机
extern moto_measure_t  moto_structure[];    //CAN1 攻击机构4个电机

extern moto_measure_t  moto_yaw,moto_pit,moto_poke,moto_info;
extern float real_current_from_judgesys;    //unit :mA
extern float dynamic_limit_current;	        //unit :mA,;	//from judge_sys
extern float ZGyroModuleAngle,yaw_zgyro_angle;

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);     //1~4电机can发送
void set_moto_current1(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);    //5~8电机can发送
void get_total_angle(moto_measure_t *p);
#endif

