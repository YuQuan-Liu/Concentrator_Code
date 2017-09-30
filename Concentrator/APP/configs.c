

#include "utils.h"
#include "device_params.h"
#include "configs.h"
#include "serial.h"
#include "frame.h"
#include "gprs.h"
#include "os.h"
#include "spi_flash.h"


void param_config(uint8_t * p_buf,uint16_t msg_size){
  uint8_t * p_temp;
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
    p_temp = get_ip();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_CON_IP1 - sFLASH_CON_START_ADDR),p_temp,4);
    Mem_Copy(mem4k + (sFLASH_CON_PORT_ - sFLASH_CON_START_ADDR),&port,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_ADDR:

    set_device_addr(p_buf + DATA_POSITION);
    p_temp = get_device_addr();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_DEVICE_ADDR - sFLASH_CON_START_ADDR),p_temp,5);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_METER:
    if(*(p_buf + DATA_POSITION) == 0x00){//ɾ����
      if(delete_meters(p_buf)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    if(*(p_buf + DATA_POSITION) == 0x01){//��ӱ�
      if(add_meters(p_buf)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }
    break;
  case FN_CJQ:
    if(*(p_buf + DATA_POSITION) == 0xAA){  //ɾ��ȫ���ɼ���  �����³�ʼ��FLASH POOL
      if(delete_cjqs()){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    if(*(p_buf + DATA_POSITION) == 0x55){  //��Ӳɼ���
      if(add_cjq(p_buf + DATA_POSITION + 1)){
        device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      }
    }

    //ɾ��ĳһ���ɼ����Ĺ����Ȳ���
    break;
  case FN_MBUS:
    switch(*(p_buf + DATA_POSITION)){
    case 0xAA:
    case 0xBB:
    case 0xFF:
      set_slave(*(p_buf + DATA_POSITION));
      temp_u8 = get_slave();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_MBUS - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_PROTOCOL:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //188
    case 0xEE: //188 bad
      set_protocol(*(p_buf + DATA_POSITION));
      temp_u8 = get_protocol();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_PROTOCOL - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_BAUD:
    //96H����9600 bps;      48H����4800 bps;    24H����2400 bps;     12H����1200 bps;
    switch(*(p_buf + DATA_POSITION)){
    case 0x12:
    case 0x24:
    case 0x48:
    case 0x96:
      set_meter_baud(*(p_buf + DATA_POSITION));
      temp_u8 = get_meter_baud();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_METER_BAUD - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_DEVICE_MODE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //����
    case 0xAA: //����
      set_device_mode(*(p_buf + DATA_POSITION));
      temp_u8 = get_device_mode();
      sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
      Mem_Copy(mem4k + (sFLASH_DEVICE_MODE - sFLASH_CON_START_ADDR),&temp_u8,1);
      sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_ERASE:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      sFLASH_PoolInit();
      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      break;
    }
    break;
  case FN_RESET:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF:
      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
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
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    break;
  case FN_SYN:
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
    sync_data2cjq();//TODO... CJQ JZQ ͬ��
    break;
  }

  unlock_mem4k();
}

void param_query(uint8_t * p_buf,uint16_t msg_size){
  uint8_t server_seq_ = *(p_buf + SEQ_POSITION) & 0x0F;
  uint32_t block_cjq = 0;
  uint8_t temp = 0;
  switch(*(p_buf + FN_POSITION)){
  case FN_IP_PORT:
    ack_query_ip(*(p_buf+msg_size),server_seq_);
    break;
  case FN_METER:
    switch(*(p_buf + DATA_POSITION)){
    case 0xFF: //ȫ����
      ack_query_meter_all(*(p_buf+msg_size),server_seq_);
      break;
    case 0xAA: //�����ɼ���ͨ��
      block_cjq = search_cjq(p_buf + DATA_POSITION+1);
      if(block_cjq){
        ack_query_meter_channel(block_cjq,0,0,*(p_buf+msg_size),server_seq_);
      }
      break;
    case 0x11: //������
      ack_query_meter_single(p_buf + DATA_POSITION+1,p_buf + DATA_POSITION + 6,*(p_buf+msg_size),server_seq_);
      break;
    }
    break;
  case FN_CJQ:
    ack_query_cjq(*(p_buf+msg_size),server_seq_);
    break;
  case FN_ADDR:
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_QUERY,FN_ADDR);
    break;
  case FN_MBUS:
    temp = get_slave();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_MBUS);
    break;
  case FN_DI_SEQ:
    temp = get_di_seq();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_DI_SEQ);
    break;
  case FN_ACK_ACTION:
    temp = get_ack_valve();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_ACK_ACTION);
    break;
  case FN_PROTOCOL:
    temp = get_protocol();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_PROTOCOL);
    break;
  case FN_VERSION:
    temp = get_version();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_VERSION);
    break;
  case FN_BAUD:
    temp = get_meter_baud();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_BAUD);
    break;
  case FN_DEVICE_MODE:
    temp = get_device_mode();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_DEVICE_MODE);
    break;
  case FN_READING:
    temp = get_readding();
    device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)&temp,1,AFN_QUERY,FN_READING);
    break;
  case FN_SYN:
      device_ack(*(p_buf+msg_size),server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
      check_sync_data2cjq();//TODO... ��� CJQ JZQ ͬ��
      break;
  }
}

void device_ack(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint8_t i;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;

    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_device_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];

    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;

    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }

    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;

    switch(desc){
    case 0x01:
      if(lock_gprs()){
        send_server(p_buf_,17+data_len);
        unlock_gprs();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        lock_cjq();
      }
      break;
    }
    put_membuf(p_buf_);
  }
}



void device_ack_cjq(uint8_t desc,uint8_t server_seq_,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint8_t i;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((14+data_len) << 2) | 0x03;
    *p_buf_16++ = ((14+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;

    *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_ACK;
    /**/
    p_temp = get_cjq_addr();
    *p_buf++ = p_temp[0];
    *p_buf++ = p_temp[1];
    *p_buf++ = p_temp[2];
    *p_buf++ = p_temp[3];
    *p_buf++ = p_temp[4];

    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;

    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }

    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;

    switch(desc){
    case 0x01:
      if(lock_lora()){
        write_lora(p_buf_,17+data_len);
        unlock_lora();
      }
      break;
    default:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        lock_cjq();
      }
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
    *p_buf_16++ = get_port();
    p_buf = (uint8_t *)p_buf_16;

    *p_buf++ = check_cs(p_buf_+6,15);
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


void ack_query_cjq(uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;

  uint16_t cjq_count;
  uint32_t block_cjq;
  uint8_t cjq_addr[5];
  uint16_t i;
  uint8_t j;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);
    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);

    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+cjq_count*5) << 2) | 0x03;
    *p_buf_16++ = ((9+cjq_count*5) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
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
    *p_buf++ = FN_CJQ;

    for(i = 0;i < cjq_count;i++){
      sFLASH_ReadBuffer(cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
      for(j = 0;j < 5;j++){
        *p_buf++ = cjq_addr[j];
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }

    *p_buf++ = check_cs(p_buf_+6,9+cjq_count*5);
    *p_buf++ = FRAME_END;

    switch(desc){
    case 0x01:
      send_server(p_buf_,17+cjq_count*5);
      break;
    default:
      write_cjq(p_buf_,17+cjq_count*5);
      break;
    }
    put_membuf(p_buf_);
  }
}


uint8_t * ack_mulit_header(uint8_t *p_buf,uint8_t frame_type,uint16_t len,uint8_t afn,uint8_t seq_,uint8_t fn){
  uint8_t * p_temp;
  uint16_t * p_buf_16;

  *p_buf++ = FRAME_HEAD;
  p_buf_16 = (uint16_t *)p_buf;
  *p_buf_16++ = len;
  *p_buf_16++ = len;
  p_buf = (uint8_t *)p_buf_16;
  *p_buf++ = FRAME_HEAD;

  *p_buf++ = ZERO_BYTE | DIR_TO_SERVER | PRM_SLAVE | SLAVE_FUN_DATA;
  /**/
  p_temp = get_device_addr();
  *p_buf++ = p_temp[0];
  *p_buf++ = p_temp[1];
  *p_buf++ = p_temp[2];
  *p_buf++ = p_temp[3];
  *p_buf++ = p_temp[4];

  *p_buf++ = afn;
  switch(frame_type){
  case 0://��֡ 0
    *p_buf++ = ZERO_BYTE | SINGLE | CONFIRM | seq_;
    break;
  case 1://��֡ 1
    *p_buf++ = ZERO_BYTE | MUL_FIRST | CONFIRM | seq_;
    break;
  case 2://�м�֡ 2
    *p_buf++ = ZERO_BYTE | MUL_MIDDLE | CONFIRM | seq_;
    break;
  case 3://β֡ 3
    *p_buf++ = ZERO_BYTE | MUL_LAST | CONFIRM | seq_;
    break;
  }
  *p_buf++ = fn;
  return p_buf;
}


void ack_query_meter_all(uint8_t desc,uint8_t server_seq_){
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;
  uint16_t meter_count = 0;

  uint16_t send_times = 0;  //���б� �����ܴ���
  uint16_t sended = 0;  //�Ѿ����͵�
  uint8_t remains = 0;
  uint16_t c = 0;

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

  //��ȡ����ͨ�����еı������һ��Ҫ���Ͷ��ٴ�
  for(c = 0;c < cjq_count;c++){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    send_times = send_times + meter_count/5;
    if(remains){
      send_times = send_times + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

  for(c = 0;c < cjq_count;c++){
    ack_query_meter_channel(block_cjq,send_times,sended,desc,server_seq_);

    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    sended = sended + meter_count/5;
    if(remains){
      sended = sended + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }
}

void ack_query_meter_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint16_t i;
  uint8_t j;
  uint8_t k;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  uint8_t cjq_addr[5];

  uint32_t block_cjq = block_cjq_;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;

  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //һ��Ҫ���Ͷ���֡

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
    if(cjqmeter_count > 0){
      times = cjqmeter_count/5;
      remain = cjqmeter_count%5;
      times_ = times;
      if(remain > 0){
        times_ = times_ + 1;
      }

      for(i=0;i< times_;i++){
        p_buf = p_buf_;
        frame_data_len = 0;
        frame_meter_count = 0;
        if(times_==1){//��֡ 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+7*frame_meter_count+5+4;
          p_buf = ack_mulit_header(p_buf,0,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
        }else{
          if(i==0){//��֡ 1
            frame_meter_count = 5;
            frame_data_len = 9+7*frame_meter_count+5+4;
            p_buf = ack_mulit_header(p_buf,1,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
          }else{
            if(i==times-1){//β֡ 3
              frame_meter_count = remain;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,3,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
            }else{//�м�֡ 2
              frame_meter_count = 5;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,2,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
            }
          }
        }

        //֡��������
        p_buf_16 = (uint16_t *)p_buf;
        if(frame_times){
          *p_buf_16++ = frame_times;  //�ܴ���
        }else{
          *p_buf_16++ = times_;
        }
        *p_buf_16++ = i+1+frame_times_start;  //��ǰ����
        p_buf = (uint8_t *)p_buf_16;
        //�ɼ�����ַ
        for(j=0;j<5;j++){
          *p_buf++ = cjq_addr[j];
        }
        //������
        for(k=0;k<frame_meter_count;k++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
          for(j=0;j<7;j++){
            *p_buf++ = meter_addr[j];
          }
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
        }

        *p_buf++ = check_cs(p_buf_+6,frame_data_len);
        *p_buf++ = FRAME_END;

        switch(desc){
        case 0x01:
          if(lock_lora()){
            write_lora(p_buf_,p_buf - p_buf_);
            unlock_lora();
          }
          break;
        case 0x00:
          if(lock_cjq()){
            write_cjq(p_buf_,p_buf - p_buf_);
            lock_cjq();
          }
          break;
        }
        delayms(100);
      }
    }
    put_membuf(p_buf_);
  }
}

void ack_query_meter_single(uint8_t *p_cjqaddr,uint8_t * p_meteraddr,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t i;

  uint32_t block_cjq = 0;


  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      if(search_meter(block_cjq,p_meteraddr)){  //get the meter
        *p_buf++ = FRAME_HEAD;
        *p_buf++ = 0x67;//((9+12+4) << 2) | 0x03;
        *p_buf++ = 0x00;
        *p_buf++ = 0x67;//((9+12+4) << 2) | 0x03;
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
        *p_buf++ = FN_METER;

        *p_buf++ = 0x00;
        *p_buf++ = 0x00;
        *p_buf++ = 0x00;
        *p_buf++ = 0x00;
        for(i=0;i<5;i++){
          *p_buf++ = p_cjqaddr[i];
        }
        for(i=0;i<7;i++){
          *p_buf++ = p_meteraddr[i];
        }

        *p_buf++ = check_cs(p_buf_+6,25);
        *p_buf++ = FRAME_END;

        switch(desc){
        case 0x01:
          if(lock_lora()){
            write_lora(p_buf_,17+5+7+4);
            unlock_lora();
          }
          break;
        case 0x00:
          if(lock_cjq()){
            write_cjq(p_buf_,17+5+7+4);
            lock_cjq();
          }
          break;
        }
      }
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

    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+FLASH_POOL_NEXT_INDEX,3);
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
    //**************���òɼ�����**************//
    sFLASH_WritePage(p_cjqaddr,block_new + CJQ_FLASH_INDEX_ADDR,5);  //�ɼ�����ַ
    sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_METERCOUNT,2);  //�ɼ�������  (uint8_t *)0 �ǵ�ַ0x00000000����ֵ��
    //sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_FIRSTMETER,3);
    //sFLASH_WritePage((uint8_t *)&meter_count,block_new + CJQ_FLASH_INDEX_LASTMETER,3);   //��һ�������һ���ָ����0xFFFFFF

    //**************���������ÿ�**************//
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    if(block_last == sFLASH_POINT_END){
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
    }
    //������ӵĲɼ������е���һ���ɼ���  ָ��ԭ�������һ���ɼ���
    sFLASH_WritePage((uint8_t *)&block_last,block_new + CJQ_FLASH_INDEX_PREVCJQ,3);  //��һ���ɼ���
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
  }
  //û�еõ�FLASH BLOCK
  return block_new;
}

uint8_t delete_cjqs(void){

  uint8_t * mem4k = 0;
  uint32_t temp32 = 0;
  mem4k = get_mem4k();

  sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);//������ǰ������
  sFLASH_PoolInit();

  //the pool start
  temp32 = sFLASH_POOL_START_ADDR;
  Mem_Copy(mem4k + (sFLASH_POOL - sFLASH_CON_START_ADDR),&temp32,3);
  //pool free
  temp32 = 2044;
  Mem_Copy(mem4k + (sFLASH_POOL_FREE - sFLASH_CON_START_ADDR),&temp32,2);
  //pool used
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_POOL_USED - sFLASH_CON_START_ADDR),&temp32,2);
  //pool all
  temp32 = 2044;
  Mem_Copy(mem4k + (sFLASH_POOL_ALL - sFLASH_CON_START_ADDR),&temp32,2);

  //CJQ Q
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_CJQ_Q_START - sFLASH_CON_START_ADDR),&temp32,3);
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_CJQ_COUNT - sFLASH_CON_START_ADDR),&temp32,2);
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_CJQ_Q_LAST - sFLASH_CON_START_ADDR),&temp32,3);

  //Meter Q
  temp32 = sFLASH_POINT_END;
  Mem_Copy(mem4k + (sFLASH_METER_Q_START - sFLASH_CON_START_ADDR),&temp32,3);
  temp32 = 0x000000;
  Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),&temp32,2);

  temp32 = 0xABCDAA;
  Mem_Copy(mem4k + (sFLASH_POOL_INIT - sFLASH_CON_START_ADDR),&temp32,1);

  sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);

  return 1;
}

uint32_t add_meters(uint8_t * p_buf){
  uint8_t metercount = 0;
  uint16_t frame_len = 0;
  uint8_t i = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0; //������
  uint8_t result = 0;

  frame_len = check_frame(p_buf);
  meter_type = *(p_buf+18);
  //frame �� 8~֡ͷ+֡β  6~�������ַ�� 3~AFN+SEQ+FN
  //frame ������ : ���б�־(1) ������(1) �ɼ�����ַ(5)  ���ַ(7)...
  metercount = (frame_len - 17 - 7)/7;
  block_cjq = search_cjq(p_buf + 19);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(add_single_meter(block_cjq,p_buf+24+i*7,meter_type)){
        result++;
      }
    }
    if(result != metercount){
      result = 0;
    }
  }
  return result;
}

uint32_t add_single_meter(uint32_t block_cjq, uint8_t * p_meteraddr, uint8_t meter_type){
  uint32_t block_last = 0;
  uint32_t block_new = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint32_t meter_read = 0;
  uint8_t * mem4k = 0;

  block_last = search_meter(block_cjq,p_meteraddr);
  if(block_last){  //�Ѿ����������
    return block_last;
  }

  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_last,block_cjq+CJQ_FLASH_INDEX_LASTMETER,3);

  meter_count++;  //�ɼ����µı�����++
  meter_all++;  //���б�����++

  //��ȡһ��flash��  ��������Ӧ��Ϣ
  block_new = GetFlash();
  if(block_new){  //�õ�FLASH BLOCK
    //**************���ñ��**************//
    sFLASH_WritePage(p_meteraddr,block_new + METER_FLASH_INDEX_ADDR,7);  //���ַ
    sFLASH_WritePage((uint8_t *)&meter_type,block_new + METER_FLASH_INDEX_TYPE,1);  //������
    sFLASH_WritePage((uint8_t *)&meter_read,block_new + METER_FLASH_INDEX_READ,4);  //�����  meter_read = 0
    sFLASH_WritePage((uint8_t *)&meter_read,block_new + METER_FLASH_INDEX_METERSTATE,2);  //��״̬  meter_read = 0

    //**************���òɼ�����**************//
    mem4k = get_mem4k();
    sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //��ȡ�ɼ�������Sector
    if(block_last == sFLASH_POINT_END){//first meter
      //�ɼ�����Ŀ�ʼ�ͽ�β��ָ������ӵı�Ŀ�
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++

    }else{//���ɼ�����Ľ�βָ������ӵı�Ŀ�
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_new,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);  //�ɼ����µ���Ŀ++

      //ԭ�����һ�������һ����ָ������ӵı�Ŀ�
      sFLASH_WritePage((uint8_t *)&block_new,block_last + FLASH_POOL_NEXT_INDEX,3);
    }
    //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
    sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    //����ӵı�Ŀ����һ���� ָ��ԭ�������һ����
    sFLASH_WritePage((uint8_t *)&block_last,block_new + METER_FLASH_INDEX_PREVMETER,3);

    //**************���������ÿ�**************//
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);//all meter
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
  }
  //û�еõ�FLASH BLOCK
  return block_new;
}

uint32_t search_meter(uint32_t block_cjq,uint8_t * p_meteraddr){
  uint32_t block_current = 0;
  uint16_t meter_count = 0;
  uint8_t meter_addr[7];
  uint16_t i;

  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_current,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);

  for(i = 0;i < meter_count;i++){
    sFLASH_ReadBuffer(meter_addr,block_current+METER_FLASH_INDEX_ADDR,7);
    if(Mem_Cmp(p_meteraddr,meter_addr,7)){
      return block_current;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_current,block_current+FLASH_POOL_NEXT_INDEX,3);
  }
  return 0x000000;
}


uint32_t delete_meters(uint8_t * p_buf){
  uint8_t metercount = 0;
  uint16_t frame_len = 0;
  uint8_t i = 0;
  uint32_t block_cjq = 0;
  uint8_t result = 0;

  frame_len = check_frame(p_buf);
  //frame �� 8~֡ͷ+֡β  6~�������ַ�� 3~AFN+SEQ+FN
  //frame ������ : ���б�־(1) ������(1) �ɼ�����ַ(5)  ���ַ(7)...
  metercount = (frame_len - 17 - 7)/7;
  block_cjq = search_cjq(p_buf + 19);
  if(block_cjq){
    for(i = 0;i < metercount;i++){
      if(delete_single_meter(block_cjq,p_buf+24+i*7)){
        result++;
      }
    }
    if(result != metercount){
      result = 0;
    }
  }
  return result;
}
uint32_t delete_single_meter(uint32_t block_cjq,uint8_t * p_meteraddr){
  uint32_t block_current = 0;
  uint32_t block_after = 0;
  uint32_t block_before = 0;
  uint16_t meter_count = 0;
  uint16_t meter_all = 0;
  uint8_t * mem4k = 0;

  block_current = search_meter(block_cjq,p_meteraddr);
  if(!block_current){  //û�������
    return 1;
  }

  sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&meter_all,sFLASH_METER_COUNT,2);
  sFLASH_ReadBuffer((uint8_t *)&block_after,block_current+FLASH_POOL_NEXT_INDEX,3);
  sFLASH_ReadBuffer((uint8_t *)&block_before,block_current+METER_FLASH_INDEX_PREVMETER,3);

  if(PutFlash(block_current)){
    meter_count--;
    meter_all--;

    mem4k = get_mem4k();
    if(meter_count == 0){
      //�ɼ�����Ψһ�ı�  block_before��block_after  ��Ϊ0xFFFFFF
      sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //��ȡ����Sector
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_after,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_before,3);
      Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
      sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
      sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
    }else{
      if(block_before == sFLASH_POINT_END || block_after == sFLASH_POINT_END){//Ҫɾ��������ǵ�һ��  ���������һ��
        if(block_before == sFLASH_POINT_END){
          //�޸ĺ�һ����before Ϊblock_before
          sFLASH_ReadBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);  //��ȡ����Sector
          Mem_Copy(mem4k+block_after%0x1000 + METER_FLASH_INDEX_PREVMETER,(uint8_t *)&block_before,3);
          sFLASH_EraseSector((block_after/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);
          //�޸Ĳɼ�����һ�����ַΪ  block_after  ���²ɼ�������Ŀ
          sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //��ȡ����Sector
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_FIRSTMETER,(uint8_t *)&block_after,3);
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
          sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
        }
        if(block_after == sFLASH_POINT_END){
          //�޸�ǰһ����next Ϊblock_after
          sFLASH_ReadBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);  //��ȡ����Sector
          Mem_Copy(mem4k+block_before%0x1000 + FLASH_POOL_NEXT_INDEX,(uint8_t *)&block_after,3);
          sFLASH_EraseSector((block_before/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);
          //�޸Ĳɼ��������һ�����ַΪ  block_before  ���²ɼ�������Ŀ
          sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //��ȡ����Sector
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_LASTMETER,(uint8_t *)&block_before,3);
          Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
          sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
          sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
        }
      }else{
        //Ҫɾ����������м�
        //�޸�ǰһ����next Ϊblock_after
        sFLASH_ReadBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);  //��ȡ����Sector
        Mem_Copy(mem4k+block_before%0x1000 + FLASH_POOL_NEXT_INDEX,(uint8_t *)&block_after,3);
        sFLASH_EraseSector((block_before/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_before/0x1000)*0x1000,0x1000);
        //�޸ĺ�һ����before Ϊblock_before
        sFLASH_ReadBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);  //��ȡ����Sector
        Mem_Copy(mem4k+block_after%0x1000 + METER_FLASH_INDEX_PREVMETER,(uint8_t *)&block_before,3);
        sFLASH_EraseSector((block_after/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_after/0x1000)*0x1000,0x1000);
        //���²ɼ�������Ŀ
        sFLASH_ReadBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);  //��ȡ����Sector
        Mem_Copy(mem4k+block_cjq%0x1000 + CJQ_FLASH_INDEX_METERCOUNT,(uint8_t *)&meter_count,2);
        sFLASH_EraseSector((block_cjq/0x1000)*0x1000);
        sFLASH_WriteBuffer(mem4k,(block_cjq/0x1000)*0x1000,0x1000);
      }
    }
    //����ȫ������Ŀ
    sFLASH_ReadBuffer(mem4k,sFLASH_CON_START_ADDR,0x100);
    Mem_Copy(mem4k + (sFLASH_METER_COUNT - sFLASH_CON_START_ADDR),(uint8_t *)&meter_all,2);
    sFLASH_EraseWritePage(mem4k,sFLASH_CON_START_ADDR,0x100);
    return 1;
  }else{
    return 0;
  }
}




void sync_data2cjq(void){

}

void check_sync_data2cjq(void){

}
