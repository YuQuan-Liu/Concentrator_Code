
#include "utils.h"
#include "device_params.h"
#include "os.h"

uint8_t local_seq = 0;  //�������к�

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

/**
 * ���CS  + 
 */
uint8_t check_cs(uint8_t * start,uint16_t len){
  uint16_t i;
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
  uint16_t i;
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
  uint8_t * s;
  s = start;
  while(*s == 0x00 && s < end){
    *s++ = '\r';
  }
}


/**
  * ���� �������к�  ��������ǰ�����к�
  * �������� ��������ʱ����
  */
uint8_t addSEQ(void){
  uint8_t seq_ = 0;
  CPU_SR_ALLOC();
  CPU_CRITICAL_ENTER();
  seq_ = local_seq;
  local_seq++;
  local_seq = local_seq & 0x0F;
  CPU_CRITICAL_EXIT();
  return seq_;
}


uint8_t delayms(uint32_t timeout){
  OS_ERR err;
  OSTimeDly(1000,
            OS_OPT_TIME_DLY,
            &err);
  return 1;
}

uint8_t lock_gprs(void){
  OS_ERR err;
  CPU_TS ts;
  OSMutexPend(&MUTEX_GPRS,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_NESTING){
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
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_NESTING){
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
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_NESTING){
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
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_NESTING){
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
  if(err == OS_ERR_NONE || err == OS_ERR_MUTEX_NESTING){
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
  uint8_t lora_model;  //1~tran 2~api 3~ok
  uint8_t msg_length = p_buf_end - p_buf_start;  //��ǰҪ������ݳ���
  
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

uint8_t check_eg_meter_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  uint8_t msg_length = p_buf_end - p_buf_start;  //��ǰҪ������ݳ���
  
  if(msg_length >= 9){
    if(*(p_buf_start) == 0x0E && *(p_buf_start+1) ==0x0D && *(p_buf_start+2) ==0x0B){
      if(0x00 == check_eor(p_buf_start,9)){
        //����Э�� ֡OK
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

uint8_t check_meter_frame(uint8_t * p_buf_start,uint8_t * p_buf_end){
  uint8_t result = 0;   //2~����   0~���ݲ���  1~֡��ȷ
  
  switch(get_protocol()){
  case 0x11:
    //EG
    result = check_eg_meter_frame(p_buf_start,p_buf_end);
    break;
  case 0xFF:
    //188
    result = check_188_frame(p_buf_start,p_buf_end);
    break;
  }
  
  return result;
}
