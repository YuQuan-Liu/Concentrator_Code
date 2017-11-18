
#include "bsp.h"
#include "device_params.h"

void BSP_Init(void){
  
  //SYSCLK > 48M  two wait state
  FLASH_SetLatency(FLASH_Latency_2);
  
  //the half cycle must used clock < 8M  from the HSI or HSE  not the PLL
  //FLASH_HalfCycleAccessCmd(FLASH_HalfCycleAccess_Enale);
  
  //prefetch buffer switch on/off when sysclk < 24M
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  
  RCC_DeInit();
  
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_WaitForHSEStartUp();
  
  //HSI = 8M
  
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);  //PLL = HSE / 1 * 9 = 72M
  RCC_PLLCmd(ENABLE);
  
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);    //SYSCLK = 72M
  
  //if the HCLK is not div1 the flash prefetch and half cycle is 
  RCC_HCLKConfig(RCC_SYSCLK_Div1);      //HCLK = 72M
  
  RCC_PCLK1Config(RCC_HCLK_Div2);       //APB1 = 36M
  
  RCC_PCLK2Config(RCC_HCLK_Div1);       //APB2 = 72M
  
  //wait PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){
    ;
  }
  
  RCC_LSICmd(ENABLE);
  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){
    ;
  }
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
  
  //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
  
  
  BSP_GPIO_Init();
  BSP_NVIC_Init();
  //BSP_USART_Init();
  BSP_SPI_Init();
  
}

void BSP_GPIO_Init(void){
  GPIO_InitTypeDef gpio_init;
  EXTI_InitTypeDef exti_init;
  
  //set all the gpio to push pull to low
  gpio_init.GPIO_Pin = GPIO_Pin_All;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  GPIO_Init(GPIOB,&gpio_init);
  GPIO_Init(GPIOC,&gpio_init);
  GPIO_ResetBits(GPIOA,GPIO_Pin_All);
  GPIO_ResetBits(GPIOB,GPIO_Pin_All);
  GPIO_ResetBits(GPIOC,GPIO_Pin_All);
  
  //USART1~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART1 Rx as input floating */
  //GPIOA 10
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio_init);  
  
  /* Configure USART1 Tx as alternate function push-pull */
  //GPIOA 9
  gpio_init.GPIO_Pin = GPIO_Pin_9;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio_init);
  
  //USART2~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART1 Rx as input floating */
  //GPIOA 3
  gpio_init.GPIO_Pin = GPIO_Pin_3;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio_init);  
  
  /* Configure USART1 Tx as alternate function push-pull */
  //GPIOA 2
  gpio_init.GPIO_Pin = GPIO_Pin_2;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio_init);
  
  //USART3~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure USART3 Rx as input floating */
  //GPIOB 11
  gpio_init.GPIO_Pin = GPIO_Pin_11;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &gpio_init);  
  
  /* Configure USART3 Tx as alternate function push-pull */
  //GPIOB 10
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &gpio_init);
  
  //UART4~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /* Configure UART4 Rx as input floating */
  //GPIOC 11
  gpio_init.GPIO_Pin = GPIO_Pin_11;
  gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &gpio_init);  
  
  /* Configure UART4 Tx as alternate function push-pull */
  //GPIOC 10
  gpio_init.GPIO_Pin = GPIO_Pin_10;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &gpio_init);
  
  ///* configure SPI1 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
  //SCK
  gpio_init.GPIO_Pin = GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  //MISO
  gpio_init.GPIO_Pin = GPIO_Pin_6;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  //MOSI
  gpio_init.GPIO_Pin = GPIO_Pin_7;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  //NSS  Not used,used a gpio
  gpio_init.GPIO_Pin = GPIO_Pin_4;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  
  /* configure SPI2 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //SCK
  gpio_init.GPIO_Pin = GPIO_Pin_13;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //MISO
  gpio_init.GPIO_Pin = GPIO_Pin_14;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //MOSI
  gpio_init.GPIO_Pin = GPIO_Pin_15;
  gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  //NSS  Not used,used a gpio
  gpio_init.GPIO_Pin = GPIO_Pin_12;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  */
    
  /*
  GPIOA
  1 ~ LORAPOWER
  4 ~ SPI1 CS(W25X16)
  8 ~ GPRSPOWER : [L-ON H-OFF(LM2596)]
  11 ~ GPRSPWRKEY : L
  */
  gpio_init.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_11;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&gpio_init);
  
  //OFF the GPRS PWR
  #ifdef GPRS_MIC29302
    // MIC29302
    GPIO_ResetBits(GPIOA,GPIO_Pin_8); 
  #else
    // LM2596
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
  #endif
  
  GPIO_SetBits(GPIOA,GPIO_Pin_1); //ON the LORA PWR
  /*
  GPIOB
  0 ~ RELAY2 
  1 ~ RELAY3 
  2 ~ RELAY4
  13 ~ 485CTRL : L  Meter USART3
  14 ~ 485CTRL : L  CJQ   USAER2
  15 ~ RELAY_VALVE
  */
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&gpio_init);
  
  /*
  GPIOC
  0 ~ LED3
  1 ~ LED4
  2 ~ LED1
  3 ~ LED2
  
  4 ~ 485 PWR CTRL
  5 ~ RELAY1
  12 ~ MBUS CURRENT OVER INPUT INPUT INPUT
  
  */
  gpio_init.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
  gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC,&gpio_init);
  
  
  gpio_init.GPIO_Pin = GPIO_Pin_12;
  gpio_init.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC,&gpio_init);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
  
  /* Configure EXTI12 line */
  exti_init.EXTI_Line = EXTI_Line12;
  exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
  exti_init.EXTI_Trigger = EXTI_Trigger_Falling;  
  exti_init.EXTI_LineCmd = ENABLE;
  EXTI_Init(&exti_init);
}

void BSP_USART_Init(void){
  USART_InitTypeDef usart_init;
  uint32_t meter_baud_ = 2400;
  
  /*USART1  SIM800G*/
  usart_init.USART_BaudRate = 115200;
  usart_init.USART_WordLength = USART_WordLength_8b;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART1, &usart_init);
  USART_Cmd(USART1, ENABLE);
    
  USART_ITConfig(USART1, USART_IT_TC, DISABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  /*USART2  485 CJQ*/
  usart_init.USART_BaudRate = 2400;
  usart_init.USART_WordLength = USART_WordLength_9b;
  usart_init.USART_Parity = USART_Parity_Even;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART2, &usart_init);
  USART_Cmd(USART2, ENABLE);
  
  USART_ITConfig(USART2, USART_IT_TC, DISABLE);
  USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  
  /*USART3  LORA*/
  usart_init.USART_BaudRate = 115200;
  usart_init.USART_WordLength = USART_WordLength_8b;
  usart_init.USART_Parity = USART_Parity_No;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(USART3, &usart_init);
  USART_Cmd(USART3, ENABLE);
  
  USART_ITConfig(USART3, USART_IT_TC, DISABLE);
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  
  
  /*UART4  MBUS 485 Meter*/
  switch(get_meter_baud()){
  case 0x12:
    meter_baud_ = 1200;
    break;
  case 0x24:
    meter_baud_ = 2400;
    break;
  case 0x48:
    meter_baud_ = 4800;
    break;
  case 0x96:
    meter_baud_ = 9600;
    break;
  }
  
  usart_init.USART_BaudRate = meter_baud_;
  usart_init.USART_WordLength = USART_WordLength_9b;
  usart_init.USART_Parity = USART_Parity_Even;
  usart_init.USART_StopBits = USART_StopBits_1;
  usart_init.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  
  USART_Init(UART4, &usart_init);
  USART_Cmd(UART4, ENABLE);
  
  USART_ITConfig(UART4, USART_IT_TC, DISABLE);
  USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
  
}

void BSP_NVIC_Init(void){
  NVIC_InitTypeDef nvic_init;
  
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USART1 Interrupt */
  nvic_init.NVIC_IRQChannel = USART1_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 0;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the USART2 Interrupt */
  nvic_init.NVIC_IRQChannel = USART2_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 1;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the USART3 Interrupt */
  nvic_init.NVIC_IRQChannel = USART3_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 2;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the UART4 Interrupt */
  nvic_init.NVIC_IRQChannel = UART4_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 3;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
  
  /* Enable the EXTI15_10 Interrupt */
  nvic_init.NVIC_IRQChannel = EXTI15_10_IRQn;
  nvic_init.NVIC_IRQChannelSubPriority = 4;
  nvic_init.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic_init);
}

void BSP_SPI_Init(void){
  SPI_InitTypeDef spi_init;
  
  //the APB2 is 72M
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  spi_init.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init.SPI_CPOL = SPI_CPOL_High;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1,&spi_init);
  
  SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_TXE,DISABLE);
  SPI_I2S_ITConfig(SPI1,SPI_I2S_IT_RXNE,DISABLE);
  
  SPI_Cmd(SPI1,ENABLE);
  
  /*
  //the APB1 is 36M
  spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  spi_init.SPI_CPHA = SPI_CPHA_2Edge;
  spi_init.SPI_CPOL = SPI_CPOL_High;
  spi_init.SPI_DataSize = SPI_DataSize_8b;
  spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  
  spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
  spi_init.SPI_NSS = SPI_NSS_Soft;
  spi_init.SPI_Mode = SPI_Mode_Master;
  spi_init.SPI_CRCPolynomial = 7;
  SPI_Init(SPI2,&spi_init);
  
  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_TXE,DISABLE);
  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,DISABLE);
  
  SPI_Cmd(SPI2,ENABLE);
  */
  
}


void BSP_IWDG_Init(void){
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);   //3276.8ms
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}


//the uC OS need it
uint32_t BSP_CPU_ClkFreq(void){
  
  RCC_ClocksTypeDef rcc_clocks;
  
  RCC_GetClocksFreq(&rcc_clocks);
  
  return rcc_clocks.HCLK_Frequency;
  
}
