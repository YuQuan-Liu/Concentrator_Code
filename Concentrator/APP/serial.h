
#ifndef SERIAL_H
#define SERIAL_H

void USART1_Handler(void);
void USART2_Handler(void);
void USART3_Handler(void);
void UART4_Handler(void);



uint8_t write_lora(uint8_t * data,uint16_t count);
uint8_t write_meter(uint8_t * data,uint16_t count);
uint8_t write_cjq(uint8_t * data,uint16_t count);
uint8_t write_server(uint8_t * data,uint16_t count);  
uint8_t write_serverstr(uint8_t * data);  

/**
ptr == 0  �ж��в��ŵ�buf��
ptr != 0  �ж��зŵ�ptr��ʼ��buf��
*/
uint8_t server_2buf(uint8_t * ptr);

#endif