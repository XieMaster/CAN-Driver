/**
 ***************************************(C) COPYRIGHT 2019 ***************************************
 * @file       bsp_uart.c
 * @brief      this file contains rc data receive and processing function
 * @note       
 * @Version    V1.0.0
 * @Date       2019.03.09
 ***************************************(C) COPYRIGHT 2019 ***************************************
 */
                                                                                                              
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"
#include "usart.h"
#include "main.h"

#include <stdio.h> 

/****************************************** 接收机总线信号选择 二选一 ******************************************/

//#define DBUS	    //const DBUS  
#define SBUS	    //const SBUS 

#define __HAL_DMA_SET_COUNTER(__HANDLE__, __COUNTER__) ((__HANDLE__)->Instance->CNDTR = (uint16_t)(__COUNTER__))
//定义了 DMA设置counter ，F1的stm32f1xx_hal_dma.h库中没找到此函数

/************************************************* DBUS ********************************************************/
uint8_t   dbus_buf[DBUS_BUFLEN];
rc_info_t rc;

/************************************************* SBUS ********************************************************/
#define SBUS_INPUT_CHANNELS 16			//通道数
#define RC_Lose	0						//遥控信号丢失
uint8_t Sbus_Buff[SBUS_BUFLEN];			//串口接收缓存 25 字节
uint16_t Sbus_PWM[16];	                //拟合后的PWM脉宽                        
uint16_t Sbus_Val[16];                 //sbus 的值

struct sbus_bit_pick 
{  
	uint8_t byte;			//字节
	uint8_t rshift;			//右移
	uint8_t mask;			//掩码
	uint8_t lshift;			//左移
};

static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	//			
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },		
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

/********************************S.BUS数值转换***************************************************/
//转换的到的数值是PWM脉宽*2000
uint8_t Conversion_PWM(rc_info_t *rc)
{
	unsigned char channel,pick;
	for (channel = 0; channel < SBUS_INPUT_CHANNELS; channel++)   //循环次数
	{
		unsigned value = 0;

        for (pick = 0; pick < 3; pick++)
		{
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];
			if (decode->mask != 0)
			{
				unsigned piece = Sbus_Buff[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}
		Sbus_Val[channel] = value ;      //得到每个通道的sbus值到数组 ;
                
        Sbus_PWM[channel] = (uint16_t)(value * 0.9 + 590);    //得到每个通道PWM到数组
	}
	
    rc->ch1 = Sbus_Val[0] - 1024;
    rc->ch2 = Sbus_Val[1] - 1024;
    rc->ch3 = Sbus_Val[2] - 1024;
    rc->ch4 = Sbus_Val[3] - 1024;
    rc->ch5 = Sbus_Val[4] - 1024;
    rc->ch6 = Sbus_Val[5] - 1024;
    rc->ch7 = Sbus_Val[6] - 1024;
    rc->ch8 = Sbus_Val[7] - 1024;
    rc->ch9 = Sbus_Val[8] - 1024;
    //一个通道最大值888，超过888全部通道置零
    if ((abs(rc->ch1) > 1023) || \
        (abs(rc->ch2) > 1023) || \
        (abs(rc->ch3) > 1023) || \
        (abs(rc->ch4) > 1023) || \
        (abs(rc->ch5) > 1024) || \
        (abs(rc->ch6) > 1024) || \
        (abs(rc->ch7) > 1024) || \
        (abs(rc->ch8) > 1024) || \
        (abs(rc->ch9) > 1024))
    {
        memset(rc, 0, sizeof(rc_info_t));   //把rc中所有字节换做字符“0”，常用来对指针或字符串的初始化
    }
    
    if(Sbus_Buff[24]>=0x0C)
	{
        return RC_Lose;
	}
}


/**
  * @brief      enable global uart it and do not use DMA transfer done it       //打开全局串口中断和不用DMA中断
  * @param[in]  huart: uart IRQHandler id
  * @param[in]  pData: receive buff 
  * @param[in]  Size:  buff size
  * @retval     set success or fail
  */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->State;      //修改了
	
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* 
		 * Enable the DMA transfer for the receiver request by setting the DMAR bit
		 * in the UART CR3 register 
		 */
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}


/**
  * @brief      returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param[in]  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *             to 7 to select the DMA Stream.
  * @retval     The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Channel_TypeDef *dma_stream)      //修改了 DMA_Stream_TypeDef
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->CNDTR));                           //修改了NDTR
}



/**
  * @brief       handle received rc data        //DBUS处理接收到的rc数据
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval 
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    if ((abs(rc->ch1) > 660) || \
        (abs(rc->ch2) > 660) || \
        (abs(rc->ch3) > 660) || \
        (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(rc_info_t));   //把rc中所有字节换做字符“0”，常用来对指针或字符串的初始化
    }
}


/**
  * @brief      clear idle it flag after uart receive a frame data  //在uart接收到帧数据后清除idle it标志
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	/* clear idle it flag avoid idle interrupt all the time */
	__HAL_UART_CLEAR_IDLEFLAG(huart);

	/* handle received data in idle interrupt */
	if (huart == &DBUS_HUART)
	{
		/* clear DMA transfer complete flag */
		__HAL_DMA_DISABLE(huart->hdmarx);

		/* handle dbus data dbus_buf from DMA */
        
		#if defined DBUS
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{
			rc_callback_handler(&rc, dbus_buf);	        //处理接收到的 rc 数据
		}
        
		#elif defined SBUS
        if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == SBUS_BUFLEN)
		{
			Conversion_PWM(&rc);	                    //处理接收到的 RC 数据
		}
        #endif
        
		/* restart dma transmission */
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

/**
  * @brief      callback this function when uart interrupt  //当uart中断时回调此函数
  * @param[in]  huart: uart IRQHandler id
  * @retval  
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		uart_rx_idle_callback(huart);       //在uart接收到帧数据后清除idle it标志             
	}
}

/**
  * @brief   initialize dbus uart device    //初始化dbus/sbus uart设备
  * @param   
  * @retval  
  */
void dbus_uart_init(void)
{
	/* open uart idle it */
	__HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
	__HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    
    #if defined DBUS
	uart_receive_dma_no_it(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
    #elif defined SBUS
    uart_receive_dma_no_it(&SBUS_HUART, Sbus_Buff, SBUS_MAX_LEN);
    #endif
}



