/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @作者           : 谢锋 (XieFeng)
  * @联系方式       : xiefeng3321@163.com
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
  * @机器人大赛     : 全国大学生机器人大赛-ROBOTAC-2019
  * @控制芯片       : STM32F105R8T6
  * @开发板         : RM开发板B型
  * @车型           : 4WD 炮车
  * @遥控接收机通信 : SBUS
  * @M3508电机驱动  : CAN2
  * @M2006电机驱动  : CAN1
  * @舵机驱动控制   : 定时器PWM输出PPM信号
  * @程序介绍       : 主程序在main的while大循环里
  * @库版本         : STM32Cube_FW_F1_V1.4.0
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
#define ABS(x)	( (x>0) ? (x) : (-x) )      //求绝对值

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern rc_info_t rc;                        //遥控器各通道得值
char buf[200];                              //串口发送字符串缓存区
extern uint16_t Sbus_PWM[16];	            //拟合后的PWM脉宽             

int16_t set_3508_speed[8] = {0};            //8个电机速度闭环的速度数组
int64_t set_3508_pos = 0;                   //2006转盘电机位置闭环的位置值
int64_t pos0 = 0, pos = 0;                  //2006转盘电机初始化位置量

int16_t key_flage = 0; 
int16_t current_Motor[8] = {0};             //8个电机发送电流值数组
uint32_t power_val = 0;                     //电源电压检测 ADC 
int16_t num1 = 0, num2 = 0, num3 =0;        //计数

int64_t qiut_num = 0;                       //球筒计数
int16_t t1 = 0, t2 = 0, t3 = 0;             //时间计数

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
    dbus_uart_init();                                           // DBUS 初始化 
    oled_init();                                                //oled 初始化
    
    oled_Robotac_LOGO();                                        //Robotac LOGO 显示
    HAL_Delay(500);
    oled_JXJD_LOGO();                                           //JXJD LOGO 显示
    HAL_Delay(500);
    oled_JXJD_WZ();                                             //江西机电 文字显示
    
    HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_SET);    //GPIO输出高电平，LED熄灭
    HAL_GPIO_WritePin(ledG_GPIO_Port,ledG_Pin,GPIO_PIN_SET);    //GPIO输出高电平，LED熄灭
    
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                   //打开TIM1_CH1的PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);        

    HAL_ADC_Start(&hadc1);                                      //开启ADC，//可实现0~3.3V电压转变对应0~4095数值
    HAL_ADC_PollForConversion(&hadc1,5);                        //ADC轮询转换，超时设置5ms
    
    my_can_filter_init_recv_all(&hcan1);                        //CAN1接收滤波
    my_can_filter_init_recv_all(&hcan2);                        //CAN2接收滤波
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);                      //CAN1 接收中断 CAN_FIFO0
	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);                      //CAN2 接收中断 CAN_FIFO0
    
    /* 4WD M3508电机 速度环 PID初始化 */
    for(int i=0; i<4; i++)                                      
	{
		PID_struct_init(&pid_3508_spd[i], POSITION_PID, 8000, 8000, 7.5f, 0.1f, 0.00f);     //7.5f, 0.1f, 0.000f
        /* pid_spd[i], 位置模式PID, 输出限幅8000, 积分上限8000， kp7.5f, ki0.1f, kd0.0f */                                       
	}
    
    /* 炮车转盘 M2006 电机的速度闭环PID初始化 */
    PID_struct_init(&pid_3508_pos, POSITION_PID, 5000, 5000, 2.1f, 0.0f,15.0f);             //7.6f,0.0f,30.0f  //8.0f, 0.025f, 0.000f
    
    /* 2速胜机构摩擦轮电机 M2006 速度环 PID初始化 */
    for(int i=6; i<8; i++)                                       
	{
		PID_struct_init(&pid_3508_spd[i], POSITION_PID, 7000, 7000, 8.6f, 0.42f, 0.00f);    //8.0f, 0.025f, 0.000f
	}                                                                                       //输出限幅7000，输出电流7000mA
    
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_RESET);      //GPIO输出低电平，点亮LED
    HAL_GPIO_WritePin(ledG_GPIO_Port,ledG_Pin,GPIO_PIN_RESET);      //GPIO输出低电平，点亮LED

    for(int i=0;i<3;i++)
    {
        HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);             //启动提示音响2声，GPIO1 蜂鸣器
        HAL_Delay(200);
    } 
    HAL_GPIO_WritePin(GPIO1_GPIO_Port,GPIO1_Pin,GPIO_PIN_RESET);    //GPIO1输出低电平，熄灭蜂鸣器
    
    
//    oled_clear(Pen_Clear);        //oled 清屏
    
//    oled_showCHN(0,1,0);          //江
//    oled_showCHN(0,2,1);          //西
//    oled_showCHN(0,3,2);          //机
//    oled_showCHN(0,4,3);          //电
//    oled_showCHN(0,5,4);          //职
//    oled_showCHN(0,6,5);          //业
//    oled_showCHN(0,7,6);          //技
//    oled_showCHN(0,8,7);          //术
//    oled_showCHN(0,9,8);          //学
//    oled_showCHN(0,10,9);         //院
//    
//    oled_showCHN(1,5,10);         //炮
//    oled_showCHN(1,6,13);         //车
    
//    oled_refresh_gram();          //oled 刷新
    
    int8_t i = 0; 
  while (1)
  {
        /************************************************ 底盘电机驱动 BEGIN ******************************************************/
        for(int i=0; i<4; i++)      
        {         
            current_Motor[i] = pid_calc(&pid_3508_spd[i], moto_chassis[i].speed_rpm, set_3508_speed[i]);    //底盘4M3508电机 速度环PID                                                   
        } 
      
//        current_Motor[4] = pid_calc(&pid_3508_pos, moto_structure[0].total_angle, set_3508_pos);            //炮车转盘M2006电机 位置环PID 
        
        current_Motor[5] = pid_calc(&pid_3508_spd[5], moto_structure[1].speed_rpm, set_3508_speed[5]);      //速胜摩擦轮电机1 速度环PID                        
        current_Motor[6] = pid_calc(&pid_3508_spd[6], moto_structure[2].speed_rpm, set_3508_speed[6]);      //速胜摩擦轮电机2 速度环PID
      
        set_moto_current(&hcan2, current_Motor[0], current_Motor[1], current_Motor[2], current_Motor[3]);   //CAN2发送电流值给1~4电调，4底盘电机      
        set_moto_current(&hcan1, current_Motor[4], current_Motor[5], current_Motor[6], current_Motor[7]);   //CAN1发送电流值给5~8电调，转盘和速胜机构电机
               
        set_3508_speed[0] = set_3508_speed[1] = (rc.ch1 * 5) + (rc.ch2 * 5);        
        set_3508_speed[2] = set_3508_speed[3] = (rc.ch1 * 5) - (rc.ch2 * 5);                                //遥控 CH1&CH2 混控算法控制底盘运动
        
        if((ABS(rc.ch1)<28) & (ABS(rc.ch2)<28))                                                             //设置遥控通道死区值 
        {
            set_3508_speed[0] = set_3508_speed[1] = set_3508_speed[2] = set_3508_speed[3] = 0;
        }
        
        /**********************************************  底盘电机驱动 END   ********************************************************/            
        /*****************************************  炮车转盘M2006电机驱动 BEGIN ****************************************************/

        
        /*****************************************  炮车转盘M2006电机驱动 END   ****************************************************/     
        /********************************************  速胜机构电机驱动 BEGIN ******************************************************/     
        if(rc.ch7>50)
        {
            num2 = 0; 
            num1++;
            
            set_3508_speed[5] = -18000;                             //摩擦轮速度
            set_3508_speed[6] = -set_3508_speed[5];                 //2 2006摩擦轮电机转动           
            if(num1 > 30000)                                        //30000*2ms = 60S 时间大于60秒电流值赋零保护电机，防止发热和耗电
            {
                set_3508_speed[5] = set_3508_speed[6] = 0;          //2 2006摩擦轮电机速度置零
                num1 = 30000;
            }                           
        }
        else
        {
            num1 = 0;
            num2++;                                                 //时间计数
      
            set_3508_speed[5] = set_3508_speed[6] = 0;              //2 2006摩擦轮电机速度置零                  
            if(num2 > 2000)                                         //2000*2ms = 4S 张开电机大于4秒电流值赋零，防止发热和耗电
            {
                num2 = 2000;
            }           
        }
        
        /***********************************************  速胜机构电机驱动   END    **************************************************/      
        /**************************************************  炮车舵机部分 DEGIN  *****************************************************/            
        //PWM与遥控通道关联，设置占空比 产生PPM信号驱动舵机 
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Sbus_PWM[3]);	//PWM1与遥控CH4关联，炮车（拨弹 360度舵机）       
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Sbus_PWM[4]);	//PWM2与遥控CH5关联，炮车（2个发弹摩擦轮）
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Sbus_PWM[5]);	//PWM3与遥控CH6关联，炮车（捕网 180度舵机）
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Sbus_PWM[6]);	//PWM4与遥控CH7关联，炮车 (球筒舵机)/(速胜机构)       
        
        /**************************************************  炮车舵机部分  END   *****************************************************/
        /******************************************** 电压警报、串口调试、LED闪烁 BEGIN ******************************************************/
        num3++;        
        if(num3 > 180)                                                              //时间计数大于200*2ms = 400ms，led闪烁
        {
            HAL_GPIO_TogglePin(ledG_GPIO_Port, ledG_Pin);               
            power_val = HAL_ADC_GetValue(&hadc1);                                   //返回最近一次ADC1规则组的转换结果//获取电源电压的模拟值   
            if(power_val < 1200)                                                    // 电源电压小于11.1V，（模拟值约为1200）则蜂鸣器报警     
            {              
                HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);                     //蜂鸣器报警
                HAL_GPIO_TogglePin(ledR_GPIO_Port,ledR_Pin);                        //红色LED闪烁
            }
            else
            {
                HAL_GPIO_WritePin(GPIO1_GPIO_Port,GPIO1_Pin,GPIO_PIN_RESET);        //GPIO输出低电平，熄灭蜂鸣器
                HAL_GPIO_WritePin(ledR_GPIO_Port,ledR_Pin,GPIO_PIN_RESET);          //GPIO输出低电平，默认点亮LED
            }                
            num3 = 0;
            sprintf(buf, "CH1: %4d  CH2: %4d  CH3: %4d  CH4: %4d  CH5: %4d CH6: %4d CH7: %4d PWR: %4d \n", 
                    rc.ch1, rc.ch2, rc.ch3, rc.ch4,  rc.ch5, rc.ch6, rc.ch7, power_val);                    //按格式把数字转换为串
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (COUNTOF(buf) - 1), 10);     //串口2打印接收机通道值，超时 10ms
         
        }
        /***************************************************** 电压警报、串口调试、LED闪烁 END *************************************************/
        
        HAL_Delay(2);   //延时2ms
      
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
//  * 函数功能: AD转换结束回调函数
//  * 输入参数: hadc：AD设备类型句柄
//  * 返 回 值: 无
//  * 说    明: 无
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
