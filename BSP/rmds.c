#include "main.h"
#include "can.h"
#include "bsp_can.h"
#include "rmds.h"

#define ABS(x)	( (x>0) ? (x) : (-x) )      //�����ֵ

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
                                       ��λָ��
hcan    ѡ��CAN1/CAN2
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x000;
   
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan->pTxMsg->Data[0] = 0x55;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
    
}

/****************************************************************************************
                                     ģʽѡ��ָ��
hcan    ѡ��CAN1/CAN2
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

OpenLoop_Mode                       0x01        ���� ģʽ
Current_Mode                        0x02        ���� ģʽ
Velocity_Mode                       0x03        �ٶ� ģʽ
Position_Mode                       0x04        λ�� ģʽ
Velocity_Position_Mode              0x05        �ٶ�-λ�� ģʽ
Current_Velocity_Mode               0x06        ����-�ٶ� ģʽ
Current_Position_Mode               0x07        ����-λ�� ģʽ
Current_Velocity_Position_Mode      0x08        ����-�ٶ�-λ�� ģʽ
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Mode)
{
    unsigned short can_id = 0x001;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan->pTxMsg->Data[0] = Mode;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
    
}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
-5000 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400

}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

/****************************************************************************************
                                   �ٶ�ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;          //�ٶ�ģʽ ID��0X004
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

/****************************************************************************************
                                   λ��ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

/****************************************************************************************
                                  �ٶ�λ��ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}


/****************************************************************************************
                                  �����ٶ�ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}


/****************************************************************************************
                                  ����λ��ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID

    
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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}


/****************************************************************************************
                                  �����ٶ�λ��ģʽ�µ�����ָ��
hcan    ѡ��CAN1/CAN2

Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
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
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

/****************************************************************************************
                                      ����ָ��
hcan    ѡ��CAN1/CAN2
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
Temp_Time1��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Temp_Time2��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�ر���λ�źŷ�������
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number,unsigned char Temp_Time1,unsigned char Temp_Time2)
{
    unsigned short can_id = 0x00A;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
    
    hcan->pTxMsg->Data[0] = Temp_Time1;
    hcan->pTxMsg->Data[1] = Temp_Time2;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

/****************************************************************************************
                                      ���߼��
hcan    ѡ��CAN1/CAN2
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(CAN_HandleTypeDef* hcan,unsigned char Group,unsigned char Number)
{
    unsigned short can_id = 0x00F;
    
    hcan->pTxMsg->IDE = CAN_ID_STD;            //��׼֡
    hcan->pTxMsg->RTR = CAN_RTR_DATA;          //����֡
    hcan->pTxMsg->DLC = 0x08;                  //֡����Ϊ8
    
    if((Group<=7)&&(Number<=15))
    {
        can_id |= Group<<8;
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    hcan->pTxMsg->StdId = can_id;      //֡IDΪ���������CAN_ID
   
    hcan->pTxMsg->Data[0] = 0x55;
    hcan->pTxMsg->Data[1] = 0x55;
    hcan->pTxMsg->Data[2] = 0x55;
    hcan->pTxMsg->Data[3] = 0x55;
    hcan->pTxMsg->Data[4] = 0x55;
    hcan->pTxMsg->Data[5] = 0x55;
    hcan->pTxMsg->Data[6] = 0x55;
    hcan->pTxMsg->Data[7] = 0x55;
    
    HAL_CAN_Transmit(hcan, 200);       //CAN�������ݣ���ʱʱ��400
}

short Real_Current_Value[4] = {0};
short Real_Velocity_Value[4] = {0};
long Real_Position_Value[4] = {0};
char Real_Online[4] = {0};
char Real_Ctl1_Value[4] = {0};
char Real_Ctl2_Value[4] = {0};

//���������ݵĺ�����Ĭ��Ϊ4����������������0�飬���Ϊ1��2��3��4
/*************************************************************************
                          USB_LP_CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
//void USB_LP_CAN1_RX0_IRQHandler(void) //CAN RX
//{
//    CanRxMsgTypeDef rx_message;         //CanRxMsgTypeDef    ��HAL�⣩
//    
//    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
//	{
//        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//        
//        if((rx_message.IDE == CAN_Id_Standard)&&(rx_message.IDE == CAN_RTR_Data)&&(rx_message.DLC == 8)) //��׼֡������֡�����ݳ���Ϊ8
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

