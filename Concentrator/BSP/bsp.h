

#ifndef BSP_H
#define BSP_H

#include "stm32f10x_conf.h"


#define  APP_START_TASK_STK_SIZE      256u
#define  APP_START_TASK_PRIO      3u

#define  GPRS_MIC29302  //当GPRS小板使用的电源芯片为MIC29302时使用 默认为LM2596

#define LED1_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_2)
#define LED1_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_2)

#define LED2_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define LED2_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_3)

#define LED3_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_0)
#define LED3_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_0)

#define LED4_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define LED4_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_1)

#define RELAY1_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_5)
#define RELAY1_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_5)

#define RELAY2_ON()         GPIO_SetBits(GPIOB, GPIO_Pin_0)
#define RELAY2_OFF()         GPIO_ResetBits(GPIOB, GPIO_Pin_0)

#define RELAY3_ON()         GPIO_SetBits(GPIOB, GPIO_Pin_1)
#define RELAY3_OFF()         GPIO_ResetBits(GPIOB, GPIO_Pin_1)

#define RELAY4_ON()         GPIO_SetBits(GPIOB, GPIO_Pin_2)
#define RELAY4_OFF()         GPIO_ResetBits(GPIOB, GPIO_Pin_2)


#define RELAY_VALVE_ON()         GPIO_SetBits(GPIOB, GPIO_Pin_15)
#define RELAY_VALVE_OFF()         GPIO_ResetBits(GPIOB, GPIO_Pin_15)

#ifdef GPRS_MIC29302
  // MIC29302
  #define PWR_GPRS_ON()         GPIO_SetBits(GPIOA, GPIO_Pin_8)
  #define PWR_GPRS_OFF()         GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#else
  // LM2596
  #define PWR_GPRS_ON()         GPIO_ResetBits(GPIOA, GPIO_Pin_8)
  #define PWR_GPRS_OFF()         GPIO_SetBits(GPIOA, GPIO_Pin_8)
#endif

#define GPRS_PWRKEY_H()         GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define GPRS_PWRKEY_L()         GPIO_SetBits(GPIOA, GPIO_Pin_11)

#define PWR_485_ON()         GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define PWR_485_OFF()         GPIO_ResetBits(GPIOC, GPIO_Pin_4)

#define PWR_LORA_ON()         GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define PWR_LORA_OFF()         GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define CTRL_485_METER_SEND()         GPIO_SetBits(GPIOB, GPIO_Pin_13)
#define CTRL_485_METER_RECV()         GPIO_ResetBits(GPIOB, GPIO_Pin_13)

#define CTRL_485_CJQ_SEND()         GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define CTRL_485_CJQ_RECV()         GPIO_ResetBits(GPIOB, GPIO_Pin_14)


void BSP_Init(void);
uint32_t BSP_CPU_ClkFreq(void);
void BSP_USART_Init(void);
void BSP_GPIO_Init(void);
void BSP_NVIC_Init(void);
void BSP_SPI_Init(void);
void BSP_IWDG_Init(void);

#endif