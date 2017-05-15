

#ifndef BSP_H
#define BSP_H

#include "stm32f10x_conf.h"


#define  APP_START_TASK_STK_SIZE      128u
#define  APP_START_TASK_PRIO      3u


#define LED1_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define LED1_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define LED2_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define LED2_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_1)

#define LED3_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define LED3_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_0)

#define PWR_GPRS_ON()         GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define PWR_GPRS_OFF()         GPIO_SetBits(GPIOA, GPIO_Pin_8)

#define GPRS_PWRKEY_H()         GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define GPRS_PWRKEY_L()         GPIO_SetBits(GPIOA, GPIO_Pin_11)

#define PWR_485_ON()         GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define PWR_485_OFF()         GPIO_SetBits(GPIOB, GPIO_Pin_0)

#define PWR_LORA_ON()         GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define PWR_LORA_OFF()         GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define CTRL_485_2_SEND()         GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define CTRL_485_2_RECV()         GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define CTRL_485_1_SEND()         GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define CTRL_485_1_RECV()         GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define BEEP_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_6)
#define BEEP_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_6)

void BSP_Init(void);
uint32_t BSP_CPU_ClkFreq(void);
void BSP_USART_Init(void);
void BSP_GPIO_Init(void);
void BSP_NVIC_Init(void);
void BSP_SPI_Init(void);
void BSP_IWDG_Init(void);

#endif