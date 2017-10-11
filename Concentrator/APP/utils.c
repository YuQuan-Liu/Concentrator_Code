
#include "utils.h"
#include "device_params.h"
#include "os.h"
#include "frame.h"

uint8_t local_server_seq = 0;  //�����뼯����ͨ��ʹ�õ����к�
uint8_t local_cjq_seq = 0;  //������ɼ���ͨ��ʹ�õ����к�

uint8_t mem4k[0x1000];  //���ô���Flashʹ�õ�����  Sector==4K  ��Ҫһ��4K������


extern OS_MUTEX MUTEX_GPRS;  
extern OS_MUTEX MUTEX_LORA;
extern OS_MUTEX MUTEX_CJQ;
extern OS_MUTEX MUTEX_METER;
extern OS_MUTEX MUTEX_MEM_4K;

extern OS_MEM MEM_ISR;
extern OS_MEM MEM_Buf;

extern OS_SEM SEM_LORA_OK;
extern OS_SEM SEM_HEART_BEAT;
extern OS_SEM SEM_SERVER_ACK;
extern OS_SEM SEM_SEND_GPRS;
extern OS_SEM SEM_CJQ_ACK;


extern OS_Q Q_CJQ_USART;  //CJQ USART��������
extern OS_Q Q_METER_USART; //METER USART��������
extern OS_Q Q_LORA_USART;  //LORA USART��������
extern OS_Q Q_CJQ;  //LORA 485-CJQ ���շ��͵�����  Q_CJQ_USART  Q_LORA_USART  ������֡
extern OS_Q Q_METER;   //Q_METER_USART������֡
extern OS_Q Q_READ;    //Q_SERVER  Q_CJQ �����ȥ�����֡
extern OS_Q Q_CONFIG;  //Q_SERVER  Q_CJQ �����ȥ���õ�֡

/**
 * ���CS  + 
 */
uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i = 0;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs += *(start+i);
  }
  return cs;
}

/**
 * ���CS ^
 */
uint8_t check_eor(uint8_t * start,uint16_t len){
  uint16_t i = 0;
  uint8_t cs = 0;
  for(i = 0;i < len;i++){
    cs ^= *(start+i);
  }
  return cs;
}

/*
if the frame is ok  return the length of the frame
if the frame is error return 0;
*/
uint8_t check_frame(uint8_t * start){
  uint16_t len1 = 0;
  uint16_t len2 = 0;
  
  uint8_t * s = start;
  len1 = *(uint16_t *)(s + 1);
  len2 = *(uint16_t *)(s + 3);
  
  if(len1 == len2){
    len1 = len1 >> 2;
    if(*(s+len1 + 6) == check_cs(s+6,len1) && *(s+len1+7) == 0x16){
      return len1+8;
    }
  }
  return 0;
}

void replace_str00(uint8_t * start,uint8_t * end){
  uint8_t * s = start;
  while(*s == 0x00 && s < end){
    *s++ = '\r';
  }
}


/**
  * ���� �������к�  ��������ǰ�����к�
  * �������� ��������ʱ����
  */
uint8_t add_server_seq(void){
  uint8_t seq_ = 0;
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  seq_ = local_server_seq;
  local_server_seq++;
  local_server_seq = local_server_seq & 0x0F;
  CPU_CRITICAL_EXIT();
  return seq_;
}

uint8_t add_cjq_seq(void){
  uint8_t seq_ = 0;
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  seq_ = local_cjq_seq;
  local_cjq_seq++;
  local_cjq_seq = local_cjq_seq & 0x0F;
  CPU_CRITICAL_EXIT();
  return seq_;
}



uint8_t delayms(uint32_t timeout){
  OS_ERR err;
  OSTimeDly(timeout,
            OS_OPT_TIME_DLY,
            &err);
  return 1;
}

uint8_t lock_gprs(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_GPRS,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_OWNER){
    return 1;
  }
  return 0;
}

uint8_t unlock_gprs(void){
  OS_ERR err;
  OSMutexPost(&MUTEX_GPRS,OS_OPT_POST_NONE,&err);
  return 1;
}

uint8_t lock_lora(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_LORA,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_OWNER){
    return 1;
  }
  return 0;
}

uint8_t unlock_lora(void){
  OS_ERR err;
  OSMutexPost(&MUTEX_LORA,OS_OPT_POST_NONE,&err);
  return 1;
}

uint8_t lock_cjq(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_CJQ,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_OWNER){
    return 1;
  }
  return 0;
}

uint8_t unlock_cjq(void){
  OS_ERR err;
  OSMutexPost(&MUTEX_CJQ,OS_OPT_POST_NONE,&err);
  return 1;
}

uint8_t lock_meter(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_METER,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_OWNER){
    return 1;
  }
  return 0;
}

uint8_t unlock_meter(void){
  OS_ERR err;
  OSMutexPost(&MUTEX_METER,OS_OPT_POST_NONE,&err);
  return 1;
}

uint8_t lock_mem4k(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_MEM_4K,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_OWNER){
    return 1;
  }
  return 0;
}

uint8_t unlock_mem4k(void){
  OS_ERR err;
  OSMutexPost(&MUTEX_MEM_4K,OS_OPT_POST_NONE,&err);
  return 1;
}

uint8_t * get_mem4k(void){
  return mem4k;
}


uint8_t * get_membuf(void){
  OS_ERR err;
  uint8_t * p_buf = 0;
  p_buf = OSMemGet(&MEM_Buf, &err);
  if(err == OS_ERR_NONE){
    //get the buf
    Mem_Set(p_buf,0x00,256); //clear the buf
  }else{
    //didn't get the buf
    p_buf = 0;
  }
  return p_buf;
}

uint8_t put_membuf(uint8_t * p_buf){
  OS_ERR err;
  OSMemPut(&MEM_Buf,p_buf,&err);
  return 1;
}

uint8_t * get_memisr(void){
  OS_ERR err;
  uint8_t * p_buf = 0;
  p_buf = OSMemGet(&MEM_ISR, &err);
  if(err == OS_ERR_NONE){
    //get the buf
    Mem_Set(p_buf,0x00,4); //clear the buf
  }else{
    //didn't get the buf
    p_buf = 0;
  }
  return p_buf;
}

uint8_t put_memisr(uint8_t * p_buf){
  OS_ERR err;
  OSMemPut(&MEM_ISR,p_buf,&err);
  return 1;
}


uint8_t wait_lora_ok(uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  OSSemPend(&SEM_LORA_OK,
            timeout,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  if(err == OS_ERR_NONE){
    return 1;
  }
  return 0;
}

uint8_t signal_lora_ok(void){
  OS_ERR err;
  OSSemPost(&SEM_LORA_OK,
            OS_OPT_POST_1,
            &err);
  return 1;
}

uint8_t wait_heartbeat(uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  OSSemPend(&SEM_HEART_BEAT,
            timeout,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  if(err == OS_ERR_NONE){
    return 1;
  }
  return 0;
}

uint8_t signal_heartbeat(void){
  OS_ERR err;
  OSSemPost(&SEM_HEART_BEAT,
            OS_OPT_POST_1,
            &err);
  return 1;
}


uint8_t wait_serverack(uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  OSSemPend(&SEM_SERVER_ACK,
            timeout,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  if(err == OS_ERR_NONE){
    return 1;
  }
  return 0;
}

uint8_t signal_serverack(void){
  OS_ERR err;
  OSSemPost(&SEM_SERVER_ACK,
            OS_OPT_POST_1,
            &err);
  return 1;
}


uint8_t wait_sendgprs(uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  OSSemPend(&SEM_SEND_GPRS,
            timeout,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  if(err == OS_ERR_NONE){
    return 1;
  }
  return 0;
}

uint8_t signal_sendgprs(void){
  OS_ERR err;
  OSSemPost(&SEM_SEND_GPRS,
            OS_OPT_POST_1,
            &err);
  return 1;
}


uint8_t wait_cjqack(uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  OSSemPend(&SEM_CJQ_ACK,
            timeout,
            OS_OPT_PEND_BLOCKING,
            &ts,
            &err);
  if(err == OS_ERR_NONE){
    return 1;
  }
  return 0;
}

uint8_t signal_cjqack(void){
  OS_ERR err;
  OSSemPost(&SEM_CJQ_ACK,
            OS_OPT_POST_1,
            &err);
  return 1;
}





uint8_t check_lora_data2frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  uint8_t lora_model = 0;  //1~tran 2~api 3~ok
  
  switch(*(p_buf_start)){
  case 0x68:
    lora_model = 1;
    break;
  case 0x0D:
    lora_model = 2;
  default:
    result = 2;
    break;
  }
  if(result == 2){
    return result;
  }
  
  switch(lora_model){
  case 1:
    result = check_xintian_frame(p_buf_start,p_buf_end);
    break;
  case 2:
    result = check_lora_ok_frame(p_buf_start,p_buf_end);
    break;
  }
  
  return result;
  
}

uint8_t check_xintian_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  uint8_t header = 0;  //�Ƿ���յ�֡ͷ
  uint16_t frame_len = 0;  //���յ�֡���ܳ���
  uint16_t frame_data_len = 0;  //���յ���֡�е����ݳ���  frame_len=frame_data_len+8
  uint8_t msg_length = p_buf_end - p_buf_start;  //��ǰҪ������ݳ���
  
  if(header == 0){
    if(msg_length > 5){
      if(p_buf_start[0] == 0x68 && p_buf_start[5] == 0x68){
        if(p_buf_start[1] == p_buf_start[3] && p_buf_start[2] == p_buf_start[4]){
          frame_data_len = (p_buf_start[1]&0xFF) | ((p_buf_start[2]&0xFF)<<8);
          frame_data_len = frame_data_len >> 2;
          header = 1;
        }
      }
      if(header == 0){
        //give up the data
        result = 2;
      }
    }
  }
  if(header == 1){
    if(msg_length >= (frame_data_len + 8)){
      frame_len = frame_data_len+8;
      
      if(*(p_buf_start+frame_data_len + 6) == check_cs(p_buf_start+6,frame_data_len) && 
         *(p_buf_start+frame_data_len + 7) == 0x16){
           //��һ֡OK
           result = 1;
         }else{
           //��һ֡�д���  ����
           result = 2;
         }
    }else{
      //�յ������ݻ�����
      result = 0;
    }
  }
  return result;
}

uint8_t check_lora_ok_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  uint8_t msg_length = p_buf_end - p_buf_start;  //��ǰҪ������ݳ���
  
  if(msg_length >= 6){
    if(*(p_buf_start+5) == 0x0A){
      if(*(p_buf_start+2) ==0x4F && *(p_buf_start+3) ==0x4B){
        //get the OK  
        //+++/AT+ESC return
        result = 1;
      }else{
        result = 2;
      }
    }else{
      result = 2;
    }
  }else{
    result = 0;
  }
  return result;
}


uint8_t check_188_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  uint8_t msg_length = p_buf_end - p_buf_start;  //��ǰҪ������ݳ���
  uint8_t header = 0;  //�Ƿ���յ�֡ͷ
  uint16_t frame_len= 0;  //���յ�֡���ܳ���
  
  if(header == 0){
    if(p_buf_start[0] == 0x68){
      header = 1;
    }
    if(header == 0){
      //give up the data
      result = 2;
    }
  }
  if(header == 1){
    if(msg_length >= 11){
      frame_len = *(p_buf_start+10)+13;
      
      if(frame_len > 0 && msg_length >= frame_len){
        if(*(p_buf_end - 2) == check_cs(p_buf_start,frame_len-2) && *(p_buf_end - 1) == 0x16){
          //��һ֡OK
          result = 1;
        }else{
          //��һ֡�д���  ����
          result = 2;
        }
      }
    }else{
      //�յ������ݻ�����
      result = 0;
    }
  }
  return result;
}

uint8_t check_meter_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  
  switch(get_protocol()){
  case 0xFF: //188
  case 0xEE: //188 bad
    result = check_188_frame(p_buf_start,p_buf_end);
    break;
  }
  
  return result;
}

uint8_t cjq_data_tome(uint8_t * p_frame,uint8_t frame_len){  
  uint8_t * p_cjqaddr;
  p_cjqaddr = get_cjq_addr();
  //�ɼ������͹�����ACK ���ݻ�Ӧ֡  c0 [c1 c2 c3 c4]
  if(p_cjqaddr[1] == *(p_frame + ADDR_POSITION + 1) && 
     p_cjqaddr[2] == *(p_frame + ADDR_POSITION + 2) && 
     p_cjqaddr[3] == *(p_frame + ADDR_POSITION + 3) && 
     p_cjqaddr[4] == *(p_frame + ADDR_POSITION + 4)){
    return 1;
  }
  //Programmer�����ҵ�����ָ��
  if(0xFF == *(p_frame + ADDR_POSITION + 1) && 
     0xFF == *(p_frame + ADDR_POSITION + 2) && 
     0xFF == *(p_frame + ADDR_POSITION + 3) && 
     0xFF == *(p_frame + ADDR_POSITION + 4)){
    return 2;
  }
  
  return 0;
}



uint8_t wait_q_cjq_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_CJQ_USART,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_cjq_usart(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_CJQ_USART,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t wait_q_meter_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_METER_USART,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_meter_usart(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_METER_USART,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t wait_q_lora_usart(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_LORA_USART,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_lora_usart(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_LORA_USART,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t wait_q_cjq(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_CJQ,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_cjq(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_CJQ,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}


uint8_t wait_q_meter(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_METER,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_meter(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_METER,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t wait_q_read(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_READ,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_read(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_READ,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t wait_q_conf(uint8_t ** p_mem, uint16_t * p_msg_size, uint32_t timeout){
  OS_ERR err;
  CPU_TS ts;
  
  *p_mem = OSQPend(&Q_CONFIG,timeout,OS_OPT_PEND_BLOCKING,p_msg_size,&ts,&err);
  
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

uint8_t post_q_conf(uint8_t * p_mem, uint16_t msg_size){
  OS_ERR err;
  
  OSQPost((OS_Q *)&Q_CONFIG,
          (void *)p_mem,
          msg_size,
          OS_OPT_POST_FIFO,
          &err);
  if(err == OS_ERR_NONE){
    return 1;
  }else{
    return 0;
  }
}

