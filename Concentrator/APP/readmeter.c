
#include "readmeter.h"
#include "utils.h"
#include "device_params.h"
#include "serial.h"
#include "frame.h"
#include "frame_188.h"
#include "spi_flash.h"
#include "configs.h"
#include "bsp.h"

void meter_control(uint8_t * p_frame,uint16_t frame_len){
  switch(get_slave()){
  case 0xAA:
  case 0xFF:
    meter_control_meter(p_frame,frame_len);
    break;
  case 0xBB:
  case 0xCC:
    meter_control_cjq(p_frame,frame_len);
    break;
  }
}

void meter_control_meter(uint8_t * p_frame,uint16_t frame_len){
  
}

void meter_control_cjq(uint8_t * p_frame,uint16_t frame_len){
  
}



void meter_read(uint8_t * p_frame,uint16_t frame_len){
  switch(get_slave()){
  case 0xAA:
  case 0xFF://底层设备 表
    switch(*(p_frame+DATA_POSITION)){  //单个表  单个采集器(通道)  全部表(全部通道)
    case 0xFF: //全部表
      meter_read_m_all(p_frame,frame_len);
      break;
    case 0xAA: //单个采集器通道
      meter_read_m_channel(p_frame,frame_len);
      break;
    case 0x11: //单个表  
      meter_read_m_meter(p_frame,frame_len);
      break;
    }
    break;
  case 0xBB:
  case 0xCC://底层设备CJQ
    switch(*(p_frame+DATA_POSITION)){  //单个表  单个采集器(通道)  全部表(全部通道)
    case 0xFF: //全部表
      meter_read_c_all(p_frame,frame_len);
      break;
    case 0xAA: //单个采集器通道
      meter_read_c_channel(p_frame,frame_len);
      break;
    case 0x11: //单个表  
      meter_read_c_meter(p_frame,frame_len);
      break;
    }
    break;
  }
}



/**
 * 抄全部表还是抄单个表
 * 底层直接是表
 */
void meter_read_m_all(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  uint8_t i = 0;
  uint8_t c = 1;  //采集器通道计数
  uint16_t meter_count = 0;
  
  uint8_t meter_read[4];
  uint8_t meter_status[2];
  uint8_t meter_addr[7];
  
  
  uint16_t send_times = 0;  //采集器所有通道表发送时  发送总次数
  uint8_t remains = 0;
    
  p_cjqaddr = p_frame+DATA_POSITION+1;
  
  for(c = 1;c < 4;c++){
    *(p_cjqaddr) = c;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
      sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
      
      remains = meter_count%10;
      send_times = meter_count/10;
      if(remains){
        send_times = send_times + 1;
      }
      
      cjq_relay_control(1,*(p_cjqaddr));  //开采集器通道
      for(i = 0;i < meter_count;i++){   //遍历抄采集器通道下的所有的表
        sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
        sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
        
        meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
      }
      cjq_relay_control(0,*(p_cjqaddr)); //关采集器通道
    }
  }
  //send the data out  send_times  TODO...
  
  
}

void meter_read_m_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  uint8_t i = 0;
  uint16_t meter_count = 0;
  
  uint8_t meter_read[4];
  uint8_t meter_status[2];
  uint8_t meter_addr[7];
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){  //find the cjq
    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    
    cjq_relay_control(1,*(p_cjqaddr+4));  //开采集器通道
    for(i = 0;i < meter_count;i++){   //遍历抄采集器通道下的所有的表
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
      
      meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);
      
      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
    }
    cjq_relay_control(0,*(p_cjqaddr+4)); //关采集器通道
    
    //send the data out  TODO...
    
    
  }
}

void meter_read_m_meter(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  
  uint8_t meter_read[4];
  uint8_t meter_status[2];
  
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;
  
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
      if(meter_read_single(block_meter, p_meteraddr, meter_type, meter_read, meter_status)){
        //send the data out  TODO...
        
        
      }
    }
  }
}


void meter_read_c_all(uint8_t * p_frame,uint16_t frame_len){
  
}

void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len){
  
  uint32_t block_cjq = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  
  uint8_t meter_read[4];
  uint8_t meter_status[2];
  
  uint8_t * p_cjq_response = 0;
  uint16_t msg_size = 0;
  
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;
  
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){
    //find the cjq
    //将此抄表指令发送给采集器
    if(meter_read_c_write_frame(p_frame,frame_len)){
      if(wait_q_cjq(&p_cjq_response,&msg_size,10000)){  
        //看看是不是给我的数据
        
        //保存数据 
        
        //send the data out  TODO...
      }
    }
  }
}

void meter_read_c_meter(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  
  uint8_t meter_read[4];
  uint8_t meter_status[2];
  
  uint8_t * p_cjq_response = 0;
  uint16_t msg_size = 0;
  
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;
  
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      //将此抄表指令发送给采集器
      if(meter_read_c_write_frame(p_frame,frame_len)){
        if(wait_q_cjq(&p_cjq_response,&msg_size,10000)){  
          //看看是不是给我的数据
          
          //保存数据 
          
          //send the data out  TODO...
        }
      }
    }
  }
}


uint8_t meter_read_c_write_frame(uint8_t * p_frame,uint16_t frame_len){
  uint8_t * p_cjq_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_ack = 0;
  
  for(i = 0;i < 3;i++){
    cjq_ack = 0;
    write_cjq_lora(p_frame,frame_len);
    
    if(wait_q_cjq(&p_cjq_response,&msg_size,10000)){  //看看是不是给我的ACK
      
      
    }
  }
  return cjq_ack;
}


uint8_t meter_read_single(uint8_t block_meter, uint8_t *p_meteraddr,uint8_t meter_type,uint8_t * meter_read,uint8_t * meter_status){
  uint8_t success = 0;
  uint8_t * p_meter_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  
  for(i =0;i<4;i++){
    meter_read[i] = 0x00;
  }
  meter_status[0] = 0x00;
  meter_status[1] = 0x00;
  
  for(i = 0;i < 3;i++){
    success = 0;
    if(meter_read_frame_send(p_meteraddr,meter_type)){
      if(wait_q_meter(&p_meter_response,&msg_size,1200)){
        switch(get_protocol()){ //根据协议  判断地址
        case 0xFF:
        case 0xEE:
          if(Mem_Cmp(p_meteraddr,p_meter_response+2,7)){
            success = 1;
            meter_status[0] = *(p_meter_response + 31);//获取ST L
            meter_status[1] = *(p_meter_response + 32);//获取ST H
            for(j = 0;j < 4;j++){
              meter_read[j] = *(p_meter_response + 14 + j);
            }
          }
          break;
        }
        if(success){
          break;
        }
      }else{   //接收数据失败
        delayms(200);
      }
    }else{   //发送数据失败
      delayms(200);
    }
  }
  
  if(!success){
    meter_status[0] = 0x40;
  }
  
  meter_read_save(block_meter,meter_read,meter_status);   //如果是188协议表  保存表信息
  if(success){
    return 1;
  }else{
    return 0;
  }
}

uint8_t meter_read_save(uint32_t block_meter,uint8_t * meter_read,uint8_t * meter_status){
  uint8_t * mem4k = 0;
  
  mem4k = get_mem4k();
  switch(get_protocol()){
  case 0xFF:
  case 0xEE:
    sFLASH_ReadBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);  //读取所在Sector
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_METERSTATE,meter_status,2);  //记录st信息
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_READ,meter_read,4);        //读数
    
    sFLASH_EraseSector((block_meter/0x1000)*0x1000);  //将配置好的Flash块重新写入到Flash中。
    sFLASH_WriteBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);
    break;
  }
}

uint8_t meter_read_frame_send(uint8_t * p_meteraddr,uint8_t meter_type){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t i = 0;
  
  p_buf = get_membuf();
  if(p_buf > 0){  //get the buf
    p_buf_ = p_buf;
    
    switch(get_protocol()){
    case 0xFF:
    case 0xEE:
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = 0xFE;
      *p_buf++ = FRAME_HEAD;
      *p_buf++ = meter_type;
      for(i=0;i<7;i++){
        *p_buf++ = *(p_meteraddr + i);
      }
      *p_buf++ = 0x01; //C
      *p_buf++ = 0x03; //len
      switch(get_di_seq()){
      case 0xFF: //默认低位在前
        *p_buf++ = DATAFLAG_RD_L;
        *p_buf++ = DATAFLAG_RD_H;
        break;
      case 0xAA:
        *p_buf++ = DATAFLAG_RD_H;
        *p_buf++ = DATAFLAG_RD_L;
        break;
      }      
      *p_buf++ = 0x01;
      *p_buf++ = check_cs(p_buf,11+3);
      *p_buf++ = FRAME_END;
      break;
    }
    write_meter(p_buf_,p_buf-p_buf_);
    return 1;
  }else{
    return 0;
  }
}


uint8_t cjq_relay_control(uint8_t cmd,uint8_t cjq){
  switch(cmd){
  case 1: //开继电器
    switch(cjq){
    case 1:
      RELAY1_ON();
      break;
    case 2:
      RELAY2_ON();
      break;
    case 3:
      RELAY3_ON();
      break;
    }
    break;
  case 0: //关继电器
    switch(cjq){
    case 1:
      RELAY1_OFF();
      break;
    case 2:
      RELAY2_OFF();
      break;
    case 3:
      RELAY3_OFF();
      break;
    }
    break;
  }
}


void send_meter_data(uint8_t *p_cjqaddr,uint8_t * p_meteraddr,uint8_t desc,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint16_t i;
  uint8_t j;
  uint8_t k;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  
  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //一共要发送多少帧
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    block_cjq = search_cjq(p_cjqaddr);
    if(block_cjq){
      if(Mem_Cmp(p_meteraddr,"\xFF\xFF\xFF\xFF\xFF\xFF\xFF",7)){  //all meter under this cjq
        
        sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
        sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
        if(cjqmeter_count > 0){
          times = cjqmeter_count/10;
          remain = cjqmeter_count%10;
          times_ = times;
          if(remain > 0){
            times_ = times_ + 1;
          }
          
          for(i=0;i< times_;i++){
            p_buf = p_buf_;
            frame_data_len = 0;
            frame_meter_count = 0;
            if(times_==1){//单帧 0
              frame_meter_count = cjqmeter_count;
              frame_data_len = 9+7*frame_meter_count+5+4;
              p_buf = ack_mulit_header(p_buf,0,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
            }else{
              if(i==0){//首帧 1
                frame_meter_count = 10;
                frame_data_len = 9+7*frame_meter_count+5+4;
                p_buf = ack_mulit_header(p_buf,1,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
              }else{
                if(i==times-1){//尾帧 3
                  frame_meter_count = remain;
                  frame_data_len = 9+7*frame_meter_count+5+4;
                  p_buf = ack_mulit_header(p_buf,3,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
                }else{//中间帧 2
                  frame_meter_count = 10;
                  frame_data_len = 9+7*frame_meter_count+5+4;
                  p_buf = ack_mulit_header(p_buf,2,(frame_data_len << 2) | 0x03,AFN_QUERY,server_seq_,FN_METER);
                }
              }
            }
            
            //帧的数据域
            p_buf_16 = (uint16_t *)p_buf;
            *p_buf_16++ = times_;  //总次数
            *p_buf_16++ = i+1;  //当前次数
            p_buf = (uint8_t *)p_buf_16;
            //采集器地址
            for(j=0;j<5;j++){
              *p_buf++ = p_cjqaddr[j];
            }
            //表数据
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
              send_server(p_buf_,frame_data_len+8);
              break;
            default:
              write_cjq(p_buf_,frame_data_len+8);
              break;
            }
            delayms(100);
          }
        }
      }else{  //the single meter under the cjq
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
            send_server(p_buf_,17+5+7+4);
            break;
          default:
            write_cjq(p_buf_,17+5+7+4);
            break;
          }
        }
      }
    }
    put_membuf(p_buf_);
  }
  
  
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