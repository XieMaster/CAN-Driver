#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "rmds.h"

#define ABS(x)	( (x>0) ? (x) : (-x) )      //求绝对值

unsigned int CAN_Time_Out = 0;

static void CAN_Delay_Us(unsigned int t)
{
	int i;
	for(i=0;i<t;i++)
	{
		int a=9;
		while(a--);
	}
}


//unsigned char can_tx_success_flag=0;
//void USB_HP_CAN1_TX_IRQHandler(void)            //CAN TX
//{
//    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
//	{
//	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//       can_tx_success_flag=1;
//    }
//}

/****************************************************************************************
                                       复位指令
hcan    选择CAN1/CAN2
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
   
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    hcan->pTxMsg->Data[0] = 0x55;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
    
}

/****************************************************************************************
                                     模式选择指令
hcan    选择CAN1/CAN2
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01        开环 模式
Current_Mode                        0x02        电流 模式
Velocity_Mode                       0x03        速度 模式
Position_Mode                       0x04        位置 模式
Velocity_Position_Mode              0x05        速度-位置 模式
Current_Velocity_Mode               0x06        电流-速度 模式
Current_Position_Mode               0x07        电流-位置 模式
Current_Velocity_Position_Mode      0x08        电流-速度-位置 模式
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    hcan->pTxMsg->Data[0] = Mode;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
    
}

/****************************************************************************************
                                   开环模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
-5000 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400

}

/****************************************************************************************
                                   电流模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = ABS(Temp_PWM);
    }
    
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan->pTxMsg->Data[2] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan->pTxMsg->Data[3] = (unsigned char)(Temp_Current&0xff);
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

/****************************************************************************************
                                   速度模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;          //速度模式 ID：0X004
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = ABS(Temp_PWM);
    }
  
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan->pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan->pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

/****************************************************************************************
                                   位置模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = ABS(Temp_PWM);
    }
    
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan->pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan->pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan->pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

/****************************************************************************************
                                  速度位置模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = ABS(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = ABS(Temp_Velocity);
    }
  
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_PWM>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_PWM&0xff);
    hcan->pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan->pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan->pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan->pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan->pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan->pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}


/****************************************************************************************
                                  电流速度模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = ABS(Temp_Current);
    }
    
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan->pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan->pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}


/****************************************************************************************
                                  电流位置模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = ABS(Temp_Current);
    }
   
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan->pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan->pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan->pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}


/****************************************************************************************
                                  电流速度位置模式下的数据指令
hcan    选择CAN1/CAN2

Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = ABS(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = ABS(Temp_Velocity);
    }
   
    hcan->pTxMsg->Data[0] = (unsigned char)((Temp_Current>>8)&0xff);
    hcan->pTxMsg->Data[1] = (unsigned char)(Temp_Current&0xff);
    hcan->pTxMsg->Data[2] = (unsigned char)((Temp_Velocity>>8)&0xff);
    hcan->pTxMsg->Data[3] = (unsigned char)(Temp_Velocity&0xff);
    hcan->pTxMsg->Data[4] = (unsigned char)((Temp_Position>>24)&0xff);
    hcan->pTxMsg->Data[5] = (unsigned char)((Temp_Position>>16)&0xff);
    hcan->pTxMsg->Data[6] = (unsigned char)((Temp_Position>>8)&0xff);
    hcan->pTxMsg->Data[7] = (unsigned char)(Temp_Position&0xff);
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

/****************************************************************************************
                                      配置指令
hcan    选择CAN1/CAN2
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
    
    hcan->pTxMsg->Data[0] = Temp_Time1;
    hcan->pTxMsg->Data[1] = Temp_Time2;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

/****************************************************************************************
                                      在线检测
hcan    选择CAN1/CAN2
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //标准帧
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //数据帧
    hcan->pTxMsg->DLC = 0x08;                  //帧长度为8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //帧ID为传入参数的CAN_ID
   
    hcan->pTxMsg->Data[0] = 0x55;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN发送数据，超时时间400
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//本接收数据的函数，默认为4个驱动器，都挂在0组，编号为1、2、3、4
/*************************************************************************
                          USB_LP_CAN1_RX0_IRQHandler
描述：CAN1的接收中断函数
*************************************************************************/
//void USB_LP_CAN1_RX0_IRQHandler(void) //CAN RX
//{
//    CanRxMsgTypeDef rx_message;         //CanRxMsgTypeDef    （HAL库）
//    
//    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
//	{
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//        
//        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //标准帧、数据帧、数据长度为8
//        {
//            if(rx_message.StdId == 0x1B)
//            {
//                Real_Current_Value[0] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[0] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[0] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x2B)
//            {
//                Real_Current_Value[1] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[1] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[1] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x3B)
//            {
//                Real_Current_Value[2] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[2] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[2] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x4B)
//            {
//                Real_Current_Value[3] = (rx_message.Data[0]<<8)|(rx_message.Data[1]);
//                Real_Velocity_Value[3] = (rx_message.Data[2]<<8)|(rx_message.Data[3]);
//                Real_Position_Value[3] = ((rx_message.Data[4]<<24)|(rx_message.Data[5]<<16)|(rx_message.Data[6]<<8)|(rx_message.Data[7]));
//            }
//            else if(rx_message.StdId == 0x1F)
//            {
//                Real_Online[0] = 1;
//            }
//            else if(rx_message.StdId == 0x2F)
//            {
//                Real_Online[1] = 1;
//            }
//            else if(rx_message.StdId == 0x3F)
//            {
//                Real_Online[2] = 1;
//            }
//            else if(rx_message.StdId == 0x4F)
//            {
//                Real_Online[3] = 1;
//            }
//            else if(rx_message.StdId == 0x1C)
//            {
//                Real_Ctl1_Value[0] = rx_message.Data[0];
//                Real_Ctl2_Value[0] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x2C)
//            {
//                Real_Ctl1_Value[1] = rx_message.Data[0];
//                Real_Ctl2_Value[1] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x3C)
//            {
//                Real_Ctl1_Value[2] = rx_message.Data[0];
//                Real_Ctl2_Value[2] = rx_message.Data[1];
//            }
//            else if(rx_message.StdId == 0x4C)
//            {
//                Real_Ctl1_Value[3] = rx_message.Data[0];
//                Real_Ctl2_Value[3] = rx_message.Data[1];
//            }
//        }
//    }
//}

