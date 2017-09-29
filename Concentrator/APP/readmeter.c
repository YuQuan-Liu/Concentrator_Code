
#include "readmeter.h"
#include "utils.h"
#include "device_params.h"
#include "serial.h"
#include "frame.h"
#include "frame_188.h"
#include "spi_flash.h"
#include "configs.h"
#include "bsp.h"
#include "lib_str.h"
#include "lib_mem.h"

void meter_control(uint8_t * p_frame,uint16_t frame_len){
  switch(get_slave()){
  case 0xAA:
  case 0xFF:
    meter_control_meter(p_frame,frame_len);
    break;
  case 0xBB:
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
  case 0xFF://底层设备 表  TODO...
//    switch(*(p_frame+DATA_POSITION)){  //单个表  单个采集器(通道)  全部表(全部通道)
//    case 0xFF: //全部表
//      meter_read_m_all(p_frame,frame_len);
//      break;
//    case 0xAA: //单个采集器通道
//      meter_read_m_channel(p_frame,frame_len);
//      break;
//    case 0x11: //单个表  
//      meter_read_m_meter(p_frame,frame_len);
//      break;
//    }
    break;
  case 0xBB://底层设备CJQ
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


uint8_t search_cjq_in_membuf(uint8_t * p_buf_cjq,uint8_t * p_cjqaddr){
  uint8_t cjq_count_inbuf = 0;
  uint8_t i = 0;
  uint8_t * p_cjqaddr_inbuf = 0;
  
  cjq_count_inbuf = p_buf_cjq[255];
  
  for(i = 0;i < cjq_count_inbuf;i++){
    p_cjqaddr_inbuf = p_buf_cjq + i*10 + 1;
    if(Mem_Cmp(p_cjqaddr_inbuf,p_cjqaddr+1,4)){
      return 1;
    }
  }
  return 0;
}

void meter_read_c_all(uint8_t * p_frame,uint16_t frame_len){
  uint16_t cjq_count = 0;  //FLASH 一共有多少采集器
  uint32_t block_cjq = 0;
  uint16_t c = 0;
  uint8_t * p_all_cjq = 0;
  
  uint8_t cjq_addr[5];
  uint8_t cjq_count_inbuf = 0;  //MEMBUF中一共有多少个采集器
  
  p_all_cjq = get_membuf();
  if(p_all_cjq > 0){
    /*
    p_all_cjq   256存储一共有多少个采集器  255存储已经读完了几个采集器
    每个采集器占10个字节  12345~采集器地址   6~是否完成~[00~未完成  11~完成  22~超时]
    */
    Mem_Set(p_buf_,0x00,256);
    
    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头
    
    //获取所有通道所有的表加起来一共要发送多少次
    for(c = 0;c < cjq_count;c++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
      
      if(!search_cjq_in_membuf(p_buf_cjq,cjq_addr)){
        cjq_count_inbuf = p_all_cjq[255];
        Mem_Copy(p_all_cjq + cjq_count_inbuf*10 + 1, cjq_addr + 1,4);
        p_all_cjq[255] = cjq_count_inbuf + 1;
      }
      
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }
    
    //TODO...
    
    
    
    put_membuf(p_all_cjq);
  }
  
  
  
  
  
  
}

void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  
  uint8_t frame_data[1];
  uint8_t * p_meter_read;
  uint8_t * p_meter_status;
  
  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;
  
  uint8_t timeout_count = 10; //如果出现10次超时就判定为失败
  uint16_t all_frames = 10;  //表示一共会返回多少帧
  uint16_t this_frame = 0;   //表示这是所有帧中的第几帧
  uint8_t frame_metercount = 0;  //接收到的帧中有几块表的数据
  
  
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){
    //find the cjq
    //将此抄表指令发送给采集器
    frame_data[0] = *(p_frame+DATA_POSITION);
    
    cjq_seq = add_cjq_seq();
    set_cjq_data_seq(cjq_seq);
    if(write_frame_cjq(p_cjqaddr, frame_data, 1,AFN_CURRENT,FN_METER,cjq_seq)){ //给采集器发送抄表指令
      if(wait_cjqack(10000)){  //等待10s采集器对抄表指令ACK
        while(timeout_count > 0){
          if(wait_q_cjq(&p_response,&msg_size,10000)){  
            //帧的长度msg_size  计算此帧中一共有多少表 frame_metercount  
            //一共有多少帧all_frames  这是第几帧this_frame
            all_frames = *(p_response + DATA_POSITION) + *(p_response +DATA_POSITION+1)<<8;
            this_frame = *(p_response + DATA_POSITION+2) + *(p_response + DATA_POSITION+3)<<8;
            frame_metercount = (msg_size-8-9-5)/14;
            
            for(i = 0;i < frame_metercount;i++){
              //从帧中获取  表地址 表的读数  表的状态
              p_meteraddr = p_response + DATA_POSITION + 5 + 14 * i;
              p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * i;
              p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * i; 
              
              block_meter = search_meter(block_cjq,p_meteraddr);
              if(block_meter){  //保存数据
                meter_read_save(block_meter,p_meter_read,p_meter_status);
              }
            }
            put_membuf(p_response);
            if(all_frames == this_frame){
              break;
            }
          }else{
            timeout_count--;
          }
        }
        
        //send the data out  TODO...
        if(timeout_count > 0){ //接收所有的帧OK
          
        }else{ //接收所有的表数据超时
          
        }
      }
    }
  }
}

void meter_read_c_meter(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  
  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;
  
  uint8_t frame_data[8];
  uint8_t * p_meter_read;
  uint8_t * p_meter_status;
  
  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;
  
  block_cjq = search_cjq(p_cjqaddr);
  
  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      //将此抄表指令发送给采集器
      frame_data[0] = *(p_frame+DATA_POSITION);
      for(i = 0;i < 7;i++){
        frame_data[i+1] = p_meteraddr[i];
      }
      
      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);
      if(write_frame_cjq(p_cjqaddr, frame_data, 8,AFN_CURRENT,FN_METER,cjq_seq)){ //给采集器发送抄表指令
        if(wait_cjqack(10000)){  //等待10s采集器对抄表指令ACK
          if(wait_q_cjq(&p_response,&msg_size,10000)){  
            //从帧中获取 表的读数  表的状态
            p_meter_read = p_response + DATA_POSITION + 5 + 8;
            p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4; 
            //保存数据 
            meter_read_save(block_meter,p_meter_read,p_meter_status);
            //send the data out  TODO...
            
            put_membuf(p_response);
          }
        }
      }
    }
  }
}


uint8_t write_frame_cjq(uint8_t * p_cjqaddr,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn,uint8_t server_seq_){
  uint8_t * p_temp;
  uint8_t * p_buf;
  uint8_t * p_buf_;
  uint16_t * p_buf_16;
  uint8_t i;
  uint8_t result = 0;
  
  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;
    *p_buf++ = FRAME_HEAD;
    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    *p_buf_16++ = ((9+data_len) << 2) | 0x03;
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = FRAME_HEAD;
    
    *p_buf++ = ZERO_BYTE | DIR_TO_DEVICE | PRM_START | SLAVE_FUN_DATA;
    /**/
    *p_buf++ = p_cjqaddr[0];
    *p_buf++ = p_cjqaddr[1];
    *p_buf++ = p_cjqaddr[2];
    *p_buf++ = p_cjqaddr[3];
    *p_buf++ = p_cjqaddr[4];
    
    *p_buf++ = afn;
    *p_buf++ = ZERO_BYTE |SINGLE | server_seq_;
    *p_buf++ = fn;
        
    for(i = 0;i < data_len;i++){
      *p_buf++ = p_data[i];
    }
    
    *p_buf++ = check_cs(p_buf_+6,9+data_len);
    *p_buf++ = FRAME_END;
    
    switch(get_device_mode()){
    case 0xFF:
      if(lock_lora()){
        write_lora(p_buf_,17+data_len);
        unlock_lora();
        result = 1;
      }else{
        result = 0;
      }
      break;
    case 0xAA:
      if(lock_cjq()){
        write_cjq(p_buf_,17+data_len);
        unlock_cjq();
        result = 1;
      }else{
        result = 0;
      }
      break;
    }
    put_membuf(p_buf_);
  }else{
    result = 0;
  }
  return result;
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
  return 1;
}
