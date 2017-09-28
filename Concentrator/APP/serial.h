
#ifndef SERIAL_H
#define SERIAL_H

void USART2_Handler(void);
void USART3_Handler(void);
void UART4_Handler(void);



uint8_t write_lora(uint8_t * data,uint16_t count);
uint8_t write_meter(uint8_t * data,uint16_t count);
uint8_t write_cjq(uint8_t * data,uint16_t count);

#endif