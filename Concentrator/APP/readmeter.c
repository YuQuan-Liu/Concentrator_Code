
#include "readmeter.h"
#include "utils.h"
#include "device_params.h"

void meter_control(uint8_t * p_buf,uint16_t msg_size){
  switch(get_slave()){
  case 0xAA:
  case 0xFF:
    meter_control_meter(p_buf,msg_size);
    break;
  case 0xBB:
  case 0xCC:
    meter_control_cjq(p_buf,msg_size);
    break;
  }
}

void meter_read(uint8_t * p_buf,uint16_t msg_size){
  switch(get_slave()){
  case 0xAA:
  case 0xFF:
    meter_read_meters(p_buf,msg_size);
    break;
  case 0xBB:
  case 0xCC:
    meter_read_cjqs(p_buf,msg_size);
    break;
  }
}

void meter_read_meters(uint8_t * p_buf,uint16_t msg_size){
  switch(get_protocol()){
  case 0xFF://188
    meter_read_188(p_buf,msg_size);
    break;
  case 0x01://EG  
    meter_read_eg(p_buf,msg_size);
    break;
  }
}

void meter_read_cjqs(uint8_t * p_buf,uint16_t msg_size){
  switch(get_protocol()){
  case 0xFF://188
    meter_read_188_cjq(p_buf,msg_size);
    break;
  case 0x01://EG  
    meter_read_eg_cjq(p_buf,msg_size);
    break;
  }
}

void meter_read_188(uint8_t * p_buf,uint16_t msg_size){
  
}

void meter_read_eg(uint8_t * p_buf,uint16_t msg_size){
  
}

void meter_read_188_cjq(uint8_t * p_buf,uint16_t msg_size){
  
}

void meter_read_eg_cjq(uint8_t * p_buf,uint16_t msg_size){
  
}

void meter_control_meter(uint8_t * p_buf,uint16_t msg_size){
  
}

void meter_control_cjq(uint8_t * p_buf,uint16_t msg_size){
  
}



/**
 * 抄海大协议表

void meter_read_eg_cjq(uint8_t * buf_frame,uint8_t frame_len,uint8_t desc){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * lora_data;
  uint8_t i;
  uint8_t cjq_ok;
  uint8_t recv_ok=0;
  uint8_t all_single = *(buf_frame + DATA_POSITION);  //0x00~全部   0xAA~单个
  uint8_t cjq_h = *(buf_frame + DATA_POSITION + 1);
  uint8_t cjq_l = *(buf_frame + DATA_POSITION + 2);
  uint8_t allmeters = *(buf_frame + DATA_POSITION + 3);  //全部表时表示总表数  单个表时表示表的地址
  uint8_t lora_seq_ = 0;
  uint8_t lora_seq = 0;
  uint8_t tmr_count =0;
  uint8_t meter_recv = 0;
  uint16_t data_len = 0;
  uint8_t metercnt = 0;
  
  //标示正在抄的采集器
  cjqaddr_eg[0] = cjq_l;
  cjqaddr_eg[1] = cjq_h;
  
  for(i = 0;i < 3;i++){
    cjq_ok = 0;
    Write_LORA(buf_frame,frame_len);
    
    OSSemPend(&SEM_CJQLORAACK,
              10000,
              OS_OPT_PEND_BLOCKING,
              &ts,
              &err);
    
    if(err != OS_ERR_NONE){
      continue;
    }
    cjq_ok = 1;
    break;
  }
  
  
  //获取config_flash的使用权
  OSMutexPend(&MUTEX_CONFIGFLASH,1000,OS_OPT_PEND_BLOCKING,&ts,&err);
  if(err != OS_ERR_NONE){
    //获取MUTEX过程中 出错了...
    //return 0xFFFFFF;
    return;
  }
  meterdata = config_flash; //将表返回的所有信息存放在config_flash
  Mem_Set(config_flash,0x00,0x1000); //clear the buf
  
  if(cjq_ok){
    //cjq is ok wait the data
    //when recv the data send the ack
    
    recv_ok=0;
    switch(all_single){
    case 0xAA:
      for(i = 0;i < 3;i++){
        lora_data = OSQPend(&Q_ReadData_LORA,
                            5000,
                            OS_OPT_PEND_BLOCKING,
                            &msg_size,
                            &ts,
                            &err);
        if(err != OS_ERR_NONE){
          continue;
        }
        
        lora_seq_ = *(lora_data + SEQ_POSITION);
        //device_ack_lora(desc,lora_seq_);
        //get the data
        recv_ok=1;
        //判断采集器地址  判断表地址
        if(cjq_h == *(lora_data + DATA_POSITION) && cjq_l == *(lora_data + DATA_POSITION + 1) && allmeters == *(lora_data + DATA_POSITION + 3)){
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0x00;
          meterdata[3] = *(lora_data + DATA_POSITION + 3);
          meterdata[4] = *(lora_data + DATA_POSITION + 4);
          meterdata[5] = *(lora_data + DATA_POSITION + 5);
        }else{
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0xFF;
        }
        OSMemPut(&MEM_Buf,lora_data,&err);
        break;
      }
      if(!recv_ok){
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = 0xFF;
      }
      break;
    case 0x00:
      while(tmr_count < 60){
        lora_data = OSQPend(&Q_ReadData_LORA,
                            1500,
                            OS_OPT_PEND_BLOCKING,
                            &msg_size,
                            &ts,
                            &err);
        if(err != OS_ERR_NONE){
          tmr_count++;
          continue;
        }
        
        
        lora_seq_ = *(lora_data + SEQ_POSITION);
        if(meter_recv == 0){
          lora_seq = lora_seq_-1; //确保第一帧被接收
          meterdata[0] = cjq_h;
          meterdata[1] = cjq_l;
          meterdata[2] = 0x00;  
        }
        //device_ack_lora(desc,lora_seq_);
        
        data_len = (lora_data[1]&0xFF) | ((lora_data[2]&0xFF)<<8);
        data_len = data_len >> 2;
        
        if(lora_seq != lora_seq_){
          lora_seq = lora_seq_;
          metercnt = (data_len-12)/3;
          
          if(0xFF == lora_data[17]){
            //采集器超时~~~~返回指令0xFF
            tmr_count=60;
          }else{
            //处理Frame 
            //正常的数据帧  
            for(i = 0;i < metercnt;i++){
              meterdata[meter_recv*3+i*3+3] = lora_data[18+3*i];
              meterdata[meter_recv*3+i*3+1+3] = lora_data[18+3*i+1];
              meterdata[meter_recv*3+i*3+2+3] = lora_data[18+3*i+2];
            }
          }
          //放在往meterdata copy前不可以  导致数组错位
          meter_recv += metercnt;
        }
        
        OSMemPut(&MEM_Buf,lora_data,&err);
        if(meter_recv == allmeters){
          recv_ok=1;
          break;
        }
      }
      if(!recv_ok){
        //sorry 采集器故障
        meterdata[0] = cjq_h;
        meterdata[1] = cjq_l;
        meterdata[2] = 0xFF;
      }
      break;
    }
  }else{
    //cjq is error send the overtime
    meterdata[0] = cjq_h;
    meterdata[1] = cjq_l;
    meterdata[2] = 0xFF;
  }
  
  
  //send the data to Server TODO...
  switch (all_single){
    case 0xAA:
      send_data_eg(1,desc);
    break;
    case 0x00:
      if(meterdata[2] != 0xFF){
        send_data_eg(allmeters,desc);
      }else{
        send_cjqtimeout_eg(desc);
      }
    break;
  }
  
  OSMutexPost(&MUTEX_CONFIGFLASH,OS_OPT_POST_NONE,&err);
}
 */