
#include "stm32f10x_conf.h"
#include "os.h"
#include "serial.h"
#include "bsp.h"
#include "utils.h"

extern OS_Q Q_CJQ_USART;
extern OS_Q Q_LORA_USART;
extern OS_Q Q_METER_USART;

uint8_t * volatile p_server = 0;      //中断中保存GPRS 返回来的数据
uint8_t * volatile p_server_ = 0;     //记录中断的开始指针

/**
 * 处理SIM800G GPRS
 */
void USART1_Handler(void){
  uint8_t rx_byte;
  
  if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) && USART_GetITStatus(USART1,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART1);
    
    if(p_server_ != 0){
      if(p_server - p_server_ < 255){
        *p_server = rx_byte;
        p_server++;
      }
    }
  }
  
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART1);
  }
  
}

/**
 * 处理采集器  集中器之间通信
 */
void USART2_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) && USART_GetITStatus(USART2,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART2);
    
    p_mem = get_memisr();
    if(p_mem){
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_CJQ_USART,
              (void *)p_mem,
              1,
              OS_OPT_POST_FIFO,
              &err);
      if(err != OS_ERR_NONE){
        //没有放进队列  放回MEMPool
        put_membuf(p_mem);
      }
    }
  }
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART2);
  }
  
}

/*
* 处理MBUS 485 抄表
*/
void USART3_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) && USART_GetITStatus(USART3,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART3);
    
    p_mem = get_memisr();
    if(p_mem){
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_METER_USART,
              (void *)p_mem,
              1,
              OS_OPT_POST_FIFO,
              &err);
      if(err != OS_ERR_NONE){
        //没有放进队列  放回MEMPool
        put_membuf(p_mem);
      }
    }
  }
  
  if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART3);
  }
  
}

/*
* 处理LORA无线通信
*/
void UART4_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  //receive the byte
  if(USART_GetFlagStatus(UART4,USART_FLAG_RXNE) && USART_GetITStatus(UART4,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(UART4);
    
    p_mem = get_memisr();
    if(p_mem){
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_LORA_USART,
              (void *)p_mem,
              1,
              OS_OPT_POST_FIFO,
              &err);
      if(err != OS_ERR_NONE){
        //没有放进队列  放回MEMPool
        put_membuf(p_mem);
      }
    }
  }
  
  if(USART_GetFlagStatus(UART4,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(UART4);
  }
  
}


/*
* 通过U4 往LORA发送数据
*/
uint8_t write_lora(uint8_t * data,uint16_t count){
  uint16_t i;
  
  for(i = 0;i < count;i++){
    USART_SendData(UART4,*(data+i));
    while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){}
  }
  return 1;
}

/*
* 通过U2 给采集器发送数据
*/
uint8_t write_cjq(uint8_t * data,uint16_t count){
  uint16_t i;
  
  CTRL_485_CJQ_SEND();
  for(i = 0;i < count;i++){
    USART_SendData(USART2,*(data+i));
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
  }
  
  CTRL_485_CJQ_RECV();
  return 1;
}

/*
* 通过U3 给 MBUS 485 表发送数据
*/
uint8_t write_meter(uint8_t * data,uint16_t count){
  uint16_t i;
  
  CTRL_485_METER_SEND();
  for(i = 0;i < count;i++){
    USART_SendData(USART3,*(data+i));
    while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET){}
  }
  
  CTRL_485_METER_RECV();
  return 1;
}


uint8_t write_server(uint8_t * data,uint16_t count){
  uint16_t i;
  
  for(i = 0;i < count;i++){
    USART_SendData(USART1,*(data+i));
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
  }
  return 1;
}

uint8_t write_serverstr(uint8_t * data){
  uint8_t * str = data;
  
  while(*str != '\0'){
    USART_SendData(USART1,*str);
    str++;
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
  }
  return 1;
}

/**
ptr == 0  中断中不放到buf中
ptr != 0  中断中放到ptr开始的buf中
*/
uint8_t server_2buf(uint8_t * ptr){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  
  p_server = ptr;
  p_server_ = ptr;
  
  CPU_CRITICAL_EXIT();
  return 1;
}




