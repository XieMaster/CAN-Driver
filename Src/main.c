/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @����           : л�� (XieFeng)
  * @��ϵ��ʽ       : xiefeng3321@163.com
  * @Date           : 2019.06
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  ******************************************************************************
  * @�����˴���     : ȫ����ѧ�������˴���-ROBOTAC-2019
  * @����оƬ       : STM32F105R8T6
  * @������         : RM������B��
  * @����           : 4WD �ڳ�
  * @ң�ؽ��ջ�ͨ�� : SBUS
  * @M3508�������  : CAN2
  * @M2006�������  : CAN1
  * @�����������   : ��ʱ��PWM���PPM�ź�
  * @�������       : ��������main��while��ѭ����
  * @��汾         : STM32Cube_FW_F1_V1.4.0
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "bsp_uart.h"
#include "oled.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ABS(x)	( (x>0) ? (x) : (-x) )      //�����ֵ

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern rc_info_t rc;                        //ң������ͨ����ֵ
char buf[200];                              //���ڷ����ַ���������
extern uint16_t Sbus_PWM[16];	            //��Ϻ��PWM����             

int16_t set_3508_speed[8] = {0};            //8������ٶȱջ����ٶ�����
int64_t set_3508_pos = 0;                   //2006ת�̵��λ�ñջ���λ��ֵ
int64_t pos0 = 0, pos = 0;                  //2006ת�̵����ʼ��λ����

int16_t key_flage = 0; 
int16_t current_Motor[8] = {0};             //8��������͵���ֵ����
uint32_t power_val = 0;                     //��Դ��ѹ��� ADC 
int16_t num1 = 0, num2 = 0, num3 =0;        //����

int64_t qiut_num = 0;                       //��Ͳ����
int16_t t1 = 0, t2 = 0, t3 = 0;             //ʱ�����

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
    dbus_uart_init();                                           // DBUS ��ʼ�� 
    oled_init();                                                //oled ��ʼ��
    
    oled_Robotac_LOGO();                                        //Robotac LOGO ��ʾ
    HAL_Delay(500);
    oled_JXJD_LOGO();                                           //JXJD LOGO ��ʾ
    HAL_Delay(500);
    oled_JXJD_WZ();                                             //�������� ������ʾ
    
    HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_SET);    //GPIO����ߵ�ƽ��LEDϨ��
    HAL_GPIO_WritePin(ledG_GPIO_Port,ledG_Pin,GPIO_PIN_SET);    //GPIO����ߵ�ƽ��LEDϨ��
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                   //��TIM1_CH1��PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);        

    HAL_ADC_Start(&hadc1);                                      //����ADC��//��ʵ��0~3.3V��ѹת���Ӧ0~4095��ֵ
    HAL_ADC_PollForConversion(&hadc1,5);                        //ADC��ѯת������ʱ����5ms
    
    my_can_filter_init_recv_all(&hcan1);                        //CAN1�����˲�
    my_can_filter_init_recv_all(&hcan2);                        //CAN2�����˲�
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);                      //CAN1 �����ж� CAN_FIFO0
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);                      //CAN2 �����ж� CAN_FIFO0
    
    /* 4WD M3508��� �ٶȻ� PID��ʼ�� */
    for(int i=0; i<4; i++)                                      
	{
		PID_struct_init(&pid_3508_spd[i], POSITION_PID, 8000, 8000, 7.5f, 0.1f, 0.00f);     //7.5f, 0.1f, 0.000f
        /* pid_spd[i], λ��ģʽPID, ����޷�8000, ��������8000�� kp7.5f, ki0.1f, kd0.0f */                                       
	}
    
    /* �ڳ�ת�� M2006 ������ٶȱջ�PID��ʼ�� */
    PID_struct_init(&pid_3508_pos, POSITION_PID, 5000, 5000, 2.1f, 0.0f,15.0f);             //7.6f,0.0f,30.0f  //8.0f, 0.025f, 0.000f
    
    /* 2��ʤ����Ħ���ֵ�� M2006 �ٶȻ� PID��ʼ�� */
    for(int i=6; i<8; i++)                                       
	{
		PID_struct_init(&pid_3508_spd[i], POSITION_PID, 7000, 7000, 8.6f, 0.42f, 0.00f);    //8.0f, 0.025f, 0.000f
	}                                                                                       //����޷�7000���������7000mA
    
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_RESET);      //GPIO����͵�ƽ������LED
    HAL_GPIO_WritePin(ledG_GPIO_Port,ledG_Pin,GPIO_PIN_RESET);      //GPIO����͵�ƽ������LED

    for(int i=0;i<3;i++)
    {
        HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);             //������ʾ����2����GPIO1 ������
        HAL_Delay(200);
    } 
    HAL_GPIO_WritePin(GPIO1_GPIO_Port,GPIO1_Pin,GPIO_PIN_RESET);    //GPIO1����͵�ƽ��Ϩ�������
    
    
//    oled_clear(Pen_Clear);        //oled ����
    
//    oled_showCHN(0,1,0);          //��
//    oled_showCHN(0,2,1);          //��
//    oled_showCHN(0,3,2);          //��
//    oled_showCHN(0,4,3);          //��
//    oled_showCHN(0,5,4);          //ְ
//    oled_showCHN(0,6,5);          //ҵ
//    oled_showCHN(0,7,6);          //��
//    oled_showCHN(0,8,7);          //��
//    oled_showCHN(0,9,8);          //ѧ
//    oled_showCHN(0,10,9);         //Ժ
//    
//    oled_showCHN(1,5,10);         //��
//    oled_showCHN(1,6,13);         //��
    
//    oled_refresh_gram();          //oled ˢ��
    
    int8_t i = 0; 
  while (1)
  {
        /************************************************ ���̵������ BEGIN ******************************************************/
        for(int i=0; i<4; i++)      
        {         
            current_Motor[i] = pid_calc(&pid_3508_spd[i], moto_chassis[i].speed_rpm, set_3508_speed[i]);    //����4M3508��� �ٶȻ�PID                                                   
        } 
      
//        current_Motor[4] = pid_calc(&pid_3508_pos, moto_structure[0].total_angle, set_3508_pos);            //�ڳ�ת��M2006��� λ�û�PID 
        
        current_Motor[5] = pid_calc(&pid_3508_spd[5], moto_structure[1].speed_rpm, set_3508_speed[5]);      //��ʤĦ���ֵ��1 �ٶȻ�PID                        
        current_Motor[6] = pid_calc(&pid_3508_spd[6], moto_structure[2].speed_rpm, set_3508_speed[6]);      //��ʤĦ���ֵ��2 �ٶȻ�PID
      
        set_moto_current(&hcan2, current_Motor[0], current_Motor[1], current_Motor[2], current_Motor[3]);   //CAN2���͵���ֵ��1~4�����4���̵��      
        set_moto_current(&hcan1, current_Motor[4], current_Motor[5], current_Motor[6], current_Motor[7]);   //CAN1���͵���ֵ��5~8�����ת�̺���ʤ�������
               
        set_3508_speed[0] = set_3508_speed[1] = (rc.ch1 * 5) + (rc.ch2 * 5);        
        set_3508_speed[2] = set_3508_speed[3] = (rc.ch1 * 5) - (rc.ch2 * 5);                                //ң�� CH1&CH2 ����㷨���Ƶ����˶�
        
        if((ABS(rc.ch1)<28) & (ABS(rc.ch2)<28))                                                             //����ң��ͨ������ֵ 
        {
            set_3508_speed[0] = set_3508_speed[1] = set_3508_speed[2] = set_3508_speed[3] = 0;
        }
        
        /**********************************************  ���̵������ END   ********************************************************/            
        /*****************************************  �ڳ�ת��M2006������� BEGIN ****************************************************/

        
        /*****************************************  �ڳ�ת��M2006������� END   ****************************************************/     
        /********************************************  ��ʤ����������� BEGIN ******************************************************/     
        if(rc.ch7>50)
        {
            num2 = 0; 
            num1++;
            
            set_3508_speed[5] = -18000;                             //Ħ�����ٶ�
            set_3508_speed[6] = -set_3508_speed[5];                 //2 2006Ħ���ֵ��ת��           
            if(num1 > 30000)                                        //30000*2ms = 60S ʱ�����60�����ֵ���㱣���������ֹ���Ⱥͺĵ�
            {
                set_3508_speed[5] = set_3508_speed[6] = 0;          //2 2006Ħ���ֵ���ٶ�����
                num1 = 30000;
            }                           
        }
        else
        {
            num1 = 0;
            num2++;                                                 //ʱ�����
      
            set_3508_speed[5] = set_3508_speed[6] = 0;              //2 2006Ħ���ֵ���ٶ�����                  
            if(num2 > 2000)                                         //2000*2ms = 4S �ſ��������4�����ֵ���㣬��ֹ���Ⱥͺĵ�
            {
                num2 = 2000;
            }           
        }
        
        /***********************************************  ��ʤ�����������   END    **************************************************/      
        /**************************************************  �ڳ�������� DEGIN  *****************************************************/            
        //PWM��ң��ͨ������������ռ�ձ� ����PPM�ź�������� 
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Sbus_PWM[3]);	//PWM1��ң��CH4�������ڳ������� 360�ȶ����       
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Sbus_PWM[4]);	//PWM2��ң��CH5�������ڳ���2������Ħ���֣�
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Sbus_PWM[5]);	//PWM3��ң��CH6�������ڳ������� 180�ȶ����
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Sbus_PWM[6]);	//PWM4��ң��CH7�������ڳ� (��Ͳ���)/(��ʤ����)       
        
        /**************************************************  �ڳ��������  END   *****************************************************/
        /******************************************** ��ѹ���������ڵ��ԡ�LED��˸ BEGIN ******************************************************/
        num3++;        
        if(num3 > 180)                                                              //ʱ���������200*2ms = 400ms��led��˸
        {
            HAL_GPIO_TogglePin(ledG_GPIO_Port, ledG_Pin);               
            power_val = HAL_ADC_GetValue(&hadc1);                                   //�������һ��ADC1�������ת�����//��ȡ��Դ��ѹ��ģ��ֵ   
            if(power_val < 1200)                                                    // ��Դ��ѹС��11.1V����ģ��ֵԼΪ1200�������������     
            {              
                HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);                     //����������
                HAL_GPIO_TogglePin(ledR_GPIO_Port,ledR_Pin);                        //��ɫLED��˸
            }
            else
            {
                HAL_GPIO_WritePin(GPIO1_GPIO_Port,GPIO1_Pin,GPIO_PIN_RESET);        //GPIO����͵�ƽ��Ϩ�������
                HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_RESET);          //GPIO����͵�ƽ��Ĭ�ϵ���LED
            }                
            num3 = 0;
            sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  CH5: %4d CH6: %4d CH7: %4d PWR: %4d \n", 
                    rc.ch1, rc.ch2, rc.ch3, rc.ch4,  rc.ch5, rc.ch6, rc.ch7, power_val);                    //����ʽ������ת��Ϊ��
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (COUNTOF(buf) - 1), 10);     //����2��ӡ���ջ�ͨ��ֵ����ʱ 10ms
         
        }
        /***************************************************** ��ѹ���������ڵ��ԡ�LED��˸ END *************************************************/
        
        HAL_Delay(2);   //��ʱ2ms
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

///**
//  * ��������: ADת�������ص�����
//  * �������: hadc��AD�豸���;��
//  * �� �� ֵ: ��
//  * ˵    ��: ��
//  */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//  power_val = HAL_ADC_GetValue(&hadc1);
//}  

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
