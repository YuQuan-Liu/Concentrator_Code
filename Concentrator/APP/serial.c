
#include "stm32f10x_conf.h"
#include "os.h"
#include "serial.h"
#include "bsp.h"
#include "utils.h"
#include "device_params.h"


uint8_t * volatile p_server = 0;      //�ж��б���GPRS ������������
uint8_t * volatile p_server_ = 0;     //��¼�жϵĿ�ʼָ��

/**
 * ����SIM800G GPRS
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
 * ����ɼ���  ������֮��ͨ��
 */
void USART2_Handler(void){
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE) && USART_GetITStatus(USART2,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART2);
    
    p_mem = get_memisr();
    if(p_mem){
      *p_mem = rx_byte;
      if(!post_q_cjq_usart(p_mem,1)){
        put_membuf(p_mem);//û�зŽ�����  �Ż�MEMPool
      }
    }
  }
  
  if(USART_GetFlagStatus(USART2,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART2);
  }
  
}

/*
* ����MBUS 485 ����
*/
void USART3_Handler(void){
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) && USART_GetITStatus(USART3,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(USART3);
    
    p_mem = get_memisr();
    if(p_mem){
      *p_mem = rx_byte;
      if(!post_q_meter_usart(p_mem,1)){
        put_membuf(p_mem);//û�зŽ�����  �Ż�MEMPool
      }
    }
  }
  
  if(USART_GetFlagStatus(USART3,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(USART3);
  }
  
}

/*
* ����LORA����ͨ��
*/
void UART4_Handler(void){
  uint8_t rx_byte;
  uint8_t *p_mem;
  
  //receive the byte
  if(USART_GetFlagStatus(UART4,USART_FLAG_RXNE) && USART_GetITStatus(UART4,USART_IT_RXNE)){
    rx_byte = USART_ReceiveData(UART4);
    
    p_mem = get_memisr();
    if(p_mem){
      *p_mem = rx_byte;
      if(!post_q_lora_usart(p_mem,1)){
        put_membuf(p_mem);//û�зŽ�����  �Ż�MEMPool
      }
    }
  }
  
  if(USART_GetFlagStatus(UART4,USART_FLAG_ORE)){
    rx_byte = USART_ReceiveData(UART4);
  }
  
}


/*
* ͨ��U4 ��LORA��������
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
* ͨ��U2 ���ɼ�����������
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
* ͨ��U3 �� MBUS 485 ��������
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

uint8_t write_cjq_lora(uint8_t * data,uint16_t count){
  switch(get_slave()){
  case 0xCC:
    write_lora(data,count);
    break;
  case 0xBB:
    write_cjq(data,count);
    break;
  }
  return 1;
}

/**
ptr == 0  �ж��в��ŵ�buf��
ptr != 0  �ж��зŵ�ptr��ʼ��buf��
*/
uint8_t server_2buf(uint8_t * ptr){
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  
  p_server = ptr;
  p_server_ = ptr;
  
  CPU_CRITICAL_EXIT();
  return 1;
}




