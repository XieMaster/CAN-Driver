#ifndef __RMDS_H__
#define __RMDS_H__

#include "mytype.h"
#include "can.h"

#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

void CAN1_Configuration(void);

void CAN_RoboModule_DRV_Reset(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number);
void CAN_RoboModule_DRV_Mode_Choice(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM);
void CAN_RoboModule_DRV_Current_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void CAN_RoboModule_DRV_Config(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Temp_Time,unsigned char Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number);

extern short Real_Current_Value[4];
extern short Real_Velocity_Value[4];
extern long Real_Position_Value[4];
extern char Real_Online[4];
extern char Real_Ctl1_Value[4];
extern char Real_Ctl2_Value[4];

#endif

