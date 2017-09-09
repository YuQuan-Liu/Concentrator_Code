

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
ptr == 0  �ж��в��ŵ�buf��
ptr != 0  �ж��зŵ�ptr��ʼ��buf��
*/
ErrorStatus Server_Post2Buf(uint8_t * ptr);
ErrorStatus Device_Read(FunctionalState NewState);


#endif