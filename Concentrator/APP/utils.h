#ifndef UTILS_H
#define UTILS_H

#include "stm32f10x_conf.h"


uint8_t check_cs(uint8_t * start,uint16_t len);
uint8_t check_eor(uint8_t * start,uint16_t len);
uint8_t check_frame(uint8_t * start);
uint8_t addSEQ(void);

void cjqaddr2eg(void);

#endif