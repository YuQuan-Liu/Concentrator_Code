

#include "configs.h"
#include "serial.h"
#include "frame.h"
#include "gprs.h"
#include "os.h"
#include "spi_flash.h"
#include "utils.h"
#include "device_params.h"


void param_config(uint8_t * p_buf,uint8_t desc){
  
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  
  uint16_t port = 0;
  uint8_t temp_u8 = 0;
  uint8_t * mem4k = 0;
  
  if(lock_mem4k()){
    mem4k = get_mem4k();
  }else{
    return;
  }
  
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    
    set_ip(p_buf + DATA_POSITION);
    set_port(*((uint16_t *)(p_buf + DATA_POSITION +4)));
    
    port = get_port();
    
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),get_ip(),4);
    Mem_Copy(mem4k + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    device_ack(desc,server_seq_);
    
    break;
  case FN_ADDR:
    
    set_device_addr(p_buf + DATA_POSITION);
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),get_device_addr(),5);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    
    device_ack(desc,server_seq_);
    break;
  case FN_METER:
    
    break;
  case FN_CJQ:
    if(*(p_buf + DATA_POSITION) == 0xAA){  //ɾ��ȫ���ɼ���  �����³�ʼ��FLASH POOL
      sFLASH_PoolInit();
      device_ack(desc,server_seq_);
    }
    
    if(*(p_buf + DATA_POSITION) == 0x55){  //��Ӳɼ���
      if(search_cjq(p_buf + DATA_POSITION + 1) == 0xFFFFFF){
        
      }
      if(add_cjq(p_buf + DATA_POSITION + 1)){
        device_ack(desc,server_seq_);
      }
    }
    break;
  case FN_MBUS:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xBB:
    case 0xCC:
    case 0xFF:
      set_slave(*(p_buf + DATA_POSITION));
      temp_u8 = get_slave();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_DI_SEQ:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xFF:
      set_di_seq(*(p_buf + DATA_POSITION));
      temp_u8 = get_di_seq();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_READMETER_DI_SEQ - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_ACK_ACTION:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xFF:
      set_ack_valve(*(p_buf + DATA_POSITION));
      temp_u8 = get_ack_valve();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_ACK_ACTION - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_PROTOCOL:
    switch(*(p_buf + DATA_POSITION)){
    case 0x11:
    case 0xFF:
      set_protocol(*(p_buf + DATA_POSITION));
      temp_u8 = get_protocol();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
      
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_ERASE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      sFLASH_PoolInit();
      device_ack(desc,server_seq_);
      break;
    }
    break;
  case FN_RESET:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      device_ack(desc,server_seq_);
      device_cmd(DISABLE);
      *((uint8_t *)0) = 0x00;  //��ʹϵͳ����
      break;
    }
    break;
  case FN_LORA_SEND:
    switch(*(p_buf + DATA_POSITION)){
    case 0x01:
      set_lora_test(1);
      break;
    default:
      set_lora_test(0);
      break;
    }
    device_ack(desc,server_seq_);
    break;
  }
  
  unlock_mem4k();
}

void param_query(uint8_t * p_buf,uint8_t desc){
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(desc,server_seq_);
    break;
  case FN_ADDR: 
    ack_query_addr(desc,server_seq_);
    break;
  case FN_METER:
    
    break;
  case FN_CJQ:
    
    break;
  case FN_MBUS:
    ack_query_mbus(desc,server_seq_);
    break;
  case FN_DI_SEQ:
    ack_query_di_seq(desc,server_seq_);
    break;
  case FN_ACK_ACTION:
    ack_query_ack_action(desc,server_seq_);
    break;
  case FN_PROTOCOL:
    ack_query_protocol(desc,server_seq_);
    break;
  case FN_VERSION:
    ack_query_version(desc,server_seq_);
    break;
  }
}


void device_ack(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_ACK;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK;
    
    *p_buf++ = check_cs(p_buf_+6,9);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      if(lock_gprs()){
        send_server(p_buf_,17);
        unlock_gprs();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17);
        lock_cjq();
      }
      break;
    }
    put_membuf(p_buf_);
  }
}

void device_ack_lora(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x3B;//(14 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x3B;//(14 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_ACK;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK;
    
    //TODO...
//    if(protocol == 0x01){
//      *buf_frame++ = cjqaddr_eg[0];
//      *buf_frame++ = cjqaddr_eg[1];
//      *buf_frame++ = 0x00;
//      *buf_frame++ = 0x00;
//      *buf_frame++ = 0x00;
//    }else{
//      *buf_frame++ = cjqaddr[0];
//      *buf_frame++ = cjqaddr[1];
//      *buf_frame++ = cjqaddr[2];
//      *buf_frame++ = cjqaddr[3];
//      *buf_frame++ = cjqaddr[4];
//    }
    
    *p_buf++ = check_cs(p_buf_+6,14);
    *p_buf++ = FRAME_END;
    
    if(lock_lora()){
      write_lora(p_buf_,22);
      unlock_lora();
    }
    put_membuf(p_buf_);
  }
}


void ack_query_addr(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x27;//(9 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ADDR;
    
    *p_buf++ = check_cs(p_buf_+6,9);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,17);
      break;
    default:
      write_cjq(p_buf_,17);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_ip(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x3F;//(15 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x3F;//(15 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_IP_PORT;
    
    p_temp = get_ip();
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[0];
    
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = port_;
    p_buf = (uint8_t *)p_buf_16;
    
    *p_buf++ = check_cs(p_buf+6,15);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,23);
      break;
    default:
      write_cjq(p_buf_,23);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_mbus(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_MBUS;
    
    *p_buf++ = get_slave();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_di_seq(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_DI_SEQ;
    
    *p_buf++ = get_di_seq();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}


void ack_query_ack_action(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_ACK_ACTION;
    
    *p_buf++ = get_ack_valve();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}


void ack_query_protocol(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_PROTOCOL;
    
    *p_buf++ = get_protocol();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}

void ack_query_version(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    
    *p_buf++ = FRAME_HEAD;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = 0x2B;//(10 << 2) | 0x03;
    *p_buf++ = 0x00;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];
    
    *p_buf++ = AFN_QUERY;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = FN_VERSION;
    
    *p_buf++ = get_version();
    *p_buf++ = check_cs(p_buf_+6,10);
    *p_buf++ = FRAME_END;
    
    switch(desc){
    case 0x01:
      send_server(p_buf_,18);
      break;
    default:
      write_cjq(p_buf_,18);
      break;
    }
    put_membuf(p_buf_);
  }
}




//�вɼ���  ���ش˲ɼ�����block��ַ   û���򷵻�0x000000
uint32_t search_cjq(uint8_t * p_cjqaddr){
  uint32_t block_current = 0;
  uint16_t cjq_count = 0;
  uint8_t cjq_addr[5];
  
  uint16_t i = 0;
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
  sFLASH_ReadBuffer((uint8_t *)&block_current,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ
  
  for(i = 0;i < cjq_count;i++){
    
    sFLASH_ReadBuffer(cjq_addr,block_current+CJQ_FLASH_INDEX_ADDR,5);
    if(Mem_Cmp(p_cjqaddr,cjq_addr,5)){
      
      return block_current;
    }
    
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+3,3);
  }
  return 0x000000;
}

uint32_t add_cjq(uint8_t * p_cjqaddr){
  
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t cjq_count = 0;
  uint32_t meter_count = 0;
  uint8_t * mem4k = 0;
  
  block_last = search_cjq(p_cjqaddr);
  if(block_last){  //�Ѿ�������ɼ�����
    return block_last;
  }
  //û������ɼ���
  
  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,sFLASH_CJQ_Q_LAST,3);
  
  cjq_count++;  //�ɼ�������++
  
  //��ȡһ��flash��  ��������Ӧ��Ϣ
  block_new = GetFlash();  
  if(block_new){  //�õ�FLASH BLOCK
    
    sFLASH_WritePage(p_cjqaddr,block_new + CJQ_FLASH_INDEX_ADDR,5);  //�ɼ�����ַ
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_METERCOUNT,2);  //�ɼ�������  (uint8_t *)0 �ǵ�ַ0x00000000����ֵ��
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_FIRSTMETER,3);  
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_LASTMETER,3);  
    //��һ�������һ���ָ����0x000000
    
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    
    if(block_last == 0x000000){
      //this is the first cjq
      //cjq Q �Ŀ�ʼ�ͽ�β��ָ������ӵĲɼ�����
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
    }else{
      //���ǵ�һ��
      //��cjq Q �Ľ�βָ������ӵĲɼ�����
      Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),(uint8_t *)&block_new,3);
      Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&cjq_count,2);
      //��ԭ�����һ���ɼ�������һ���ɼ���ָ������ӵĲɼ�����
      sFLASH_WritePage((uint8_t *)&block_new,block_last + 3,3);
      //������ӵĲɼ������е���һ���ɼ���  ָ��ԭ�������һ���ɼ���
      sFLASH_WritePage((uint8_t *)&block_last,block_new + CJQ_FLASH_INDEX_PREVCJQ,3);  //��һ���ɼ���
    }
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    return block_new;
  }
  //û�еõ�FLASH BLOCK
  return 0x000000;
}

