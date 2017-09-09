

#ifndef SERIAL_H
#define SERIAL_H

#include "bsp.h"


void USART1_Handler(void);
void USART2_Handler(void);
void UART4_Handler(void);


ErrorStatus Write_LORA(uint8_t * data,uint16_t count);
ErrorStatus Write_485(uint8_t * data,uint16_t count);
ErrorStatus Server_Write(uint8_t * data,uint16_t count);  
ErrorStatus Server_WriteStr(uint8_t * data);  

/**
ptr == 0  中断中不放到buf中
ptr != 0  中断中放到ptr开始的buf中
*/
ErrorStatus Server_Post2Buf(uint8_t * ptr);
ErrorStatus Device_Read(FunctionalState NewState);


#endif