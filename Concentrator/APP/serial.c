
#include "stm32f10x_conf.h"
#include "os.h"
#include "serial.h"


extern OS_MEM MEM_ISR;
extern OS_SEM SEM_USART1_TX;
extern OS_SEM SEM_USART2_TX;
extern OS_SEM SEM_UART4_TX;

extern OS_Q Q_485_2;
extern OS_Q Q_LORA;

extern uint8_t * volatile server_ptr;      //中断中保存GPRS 返回来的数据
extern uint8_t * volatile server_ptr_;     //记录中断的开始指针
//SIM800G
void USART1_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) && USART_GetITStatus(USART1,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART1);
    
    if(server_ptr_ != 0){
      if(server_ptr - server_ptr_ < 255){
        *server_ptr = rx_byte;
        server_ptr++;
      }
    }
  }
  
  if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART1);
  }
  
  /*
  if(USART_GetFlagStatus(USART1,USART_FLAG_TC)){
    //It must clear the TC 
    //if not it will stay here 
    USART_ClearITPendingBit(USART1,USART_IT_TC);
    OSSemPost(&SEM_USART1_TX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }*/
}




//485 READ
void USART2_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) && USART_GetITStatus(USART2,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART2);
    mem_ptr = OSMemGet(&MEM_ISR,&err);
    
    if(err == OS_ERR_NONE){
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_485_2,
              (void *)mem_ptr,
              1,
              OS_OPT_POST_FIFO,
              &err);
      if(err != OS_ERR_NONE){
        //没有放进队列  放回MEMPool
        OSMemPut(&MEM_ISR,mem_ptr,&err);
      }
    }else{
      asm("NOP");
    }
  }
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART2);
  }
  
  /*
  if(USART_GetFlagStatus(USART2,USART_FLAG_TC)){
    
    USART_ClearITPendingBit(USART2,USART_IT_TC);
    OSSemPost(&SEM_USART2_TX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }*/
}

//LORA
void UART4_Handler(void){
  OS_ERR err;
  uint8_t rx_byte;
  uint8_t *mem_ptr;
  
  //receive the byte
  if(USART_GetFlagStatus(UART4,USART_FLAG_RXNE) && USART_GetITStatus(UART4,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(UART4);
    mem_ptr = OSMemGet(&MEM_ISR,&err);
    
    if(err == OS_ERR_NONE){
      *mem_ptr = rx_byte;
      OSQPost((OS_Q *)&Q_LORA,
              (void *)mem_ptr,
              1,
              OS_OPT_POST_FIFO,
              &err);
      if(err != OS_ERR_NONE){
        //没有放进队列  放回MEMPool
        OSMemPut(&MEM_ISR,mem_ptr,&err);
      }
    }else{
      asm("NOP");
    }
  }
  
  if(USART_GetFlagStatus(UART4,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(UART4);
  }
  
  /*
  //send the data
  if(USART_GetFlagStatus(UART4,USART_FLAG_TC)){
    
    USART_ClearITPendingBit(UART4,USART_IT_TC);
    OSSemPost(&SEM_UART4_TX,
              OS_OPT_POST_1,
              &err);
    
    if(err != OS_ERR_NONE){
      asm("NOP");
    }
  }*/
}


ErrorStatus Write_LORA(uint8_t * data,uint16_t count){
  uint16_t i;
  CPU_TS ts;
  OS_ERR err;
  
  for(i = 0;i < count;i++){
    USART_SendData(UART4,*(data+i));
    while(USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){}
  }
  return SUCCESS;
}

ErrorStatus Write_485_2(uint8_t * data,uint16_t count){
  int16_t i;
  CPU_TS ts;
  OS_ERR err;
  
  CTRL_485_2_SEND();
  for(i = 0;i < count;i++){
    USART_SendData(USART2,*(data+i));
    while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}
  }
  
  CTRL_485_2_RECV();
  return SUCCESS;
}


ErrorStatus Server_Write(uint8_t * data,uint16_t count){
  uint16_t i;
  CPU_TS ts;
  OS_ERR err;
  
  for(i = 0;i < count;i++){
    USART_SendData(USART1,*(data+i));
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
  }
  
  return SUCCESS;
}

ErrorStatus Server_WriteStr(uint8_t * data){
  CPU_TS ts;
  OS_ERR err;
  uint8_t * str = data;
  
  while(*str != '\0'){
    USART_SendData(USART1,*str);
    str++;
    while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}
  }
  return SUCCESS;
}

/**
ptr == 0  中断中不放到buf中
ptr != 0  中断中放到ptr开始的buf中
*/
ErrorStatus Server_Post2Buf(uint8_t * ptr){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  
  server_ptr = ptr;
  server_ptr_ = ptr;
    
  CPU_CRITICAL_EXIT();
  return SUCCESS;
}

extern volatile uint8_t reading;
ErrorStatus Device_Read(FunctionalState NewState){
  CPU_SR_ALLOC();
  if(NewState != DISABLE){
    CPU_CRITICAL_ENTER();
    reading = 1;
    CPU_CRITICAL_EXIT();
  }else{
    CPU_CRITICAL_ENTER();
    reading = 0;
    CPU_CRITICAL_EXIT();
  }
  return SUCCESS;
}

