/**
 ***************************************(C) COPYRIGHT 2019 ***************************************
 * @file       bsp_uart.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_uart.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       2019.03.09      
 ***************************************(C) COPYRIGHT 2019 ***************************************
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "usart.h"

#define UART_RX_DMA_SIZE (1024)

#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)       //DBUS 18字节 
#define DBUS_HUART       huart4     /* For RM B 开发板 */

#define SBUS_MAX_LEN     (50)
#define SBUS_BUFLEN      (25)       //SBUS 25字节 
#define SBUS_HUART       huart4     /* For RM B 开发板 */

/** 
  * @brief  remote control information
  */
typedef __packed struct
{
  /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t ch5;
    int16_t ch6; 
    int16_t ch7; 
    int16_t ch8; 
    int16_t ch9;
  /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
} rc_info_t;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);

uint8_t Conversion_PWM(rc_info_t *rc);

#endif

