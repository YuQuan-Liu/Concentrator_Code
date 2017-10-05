
#include "readmeter.h"
#include "utils.h"
#include "device_params.h"
#include "serial.h"
#include "frame.h"
#include "frame_188.h"
#include "spi_flash.h"
#include "configs.h"
#include "bsp.h"
#include "lib_mem.h"

/**
 * 阀门控制
 * 底层直接是表
 */
void meter_control(uint8_t * p_frame,uint16_t frame_len){
  meter_control_meter(p_frame,frame_len);
}

//阀门开关控制
void meter_control_meter(uint8_t * p_frame,uint16_t frame_len){

}

/**
 * 抄全部表还是抄单个表
 * 底层直接是表
 */
void meter_read(uint8_t * p_frame,uint16_t frame_len){
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
}


//抄采集器所有通道表
void meter_read_m_all(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;

  uint8_t i = 0;
  uint8_t c = 1;  //采集器通道计数
  uint16_t meter_count = 0;
  uint16_t cjq_count = 0;

  uint8_t meter_read[4];
  uint8_t meter_status[2];
  uint8_t meter_addr[7];
  uint8_t cjq_addr[5];

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  for(c = 0;c < cjq_count;c++){

    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);

    cjq_relay_control(1,*(cjq_addr));  //开采集器通道
    for(i = 0;i < meter_count;i++){   //遍历抄采集器通道下的所有的表
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);

      meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);

      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
    }
    cjq_relay_control(0,*(cjq_addr)); //关采集器通道

    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  //这个地方表示抄表完成   等待集中器来取表
}

//抄采集器单个通道
void meter_read_m_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;
  uint8_t meter_type = 0;

  uint8_t * p_cjqaddr = 0;
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

    cjq_relay_control(1,*(p_cjqaddr));  //开采集器通道
    for(i = 0;i < meter_count;i++){   //遍历抄采集器通道下的所有的表
      sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);

      meter_read_single(block_meter, meter_addr, meter_type, meter_read, meter_status);

      sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
    }
    cjq_relay_control(0,*(p_cjqaddr)); //关采集器通道

    //send the data out
    send_meter_data_channel(block_cjq,0,0,meter_type,*(p_frame+frame_len));
  }
}

//抄单个表
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
        //send the data out
        send_meter_data_single(p_meteraddr,meter_read,meter_status,meter_type,*(p_frame+frame_len));
      }
    }
  }
}

//只管抄表
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
        put_membuf(p_meter_response);
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

//将抄表结果保存到FLASH
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

//根据协议发送抄表帧
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
      *p_buf++ = check_cs(p_buf_,11+3);
      *p_buf++ = FRAME_END;
      break;
    }
    write_meter(p_buf_,p_buf-p_buf_);
    put_membuf(p_buf_);
    return 1;
  }else{
    return 0;
  }
}

//采集器通道开关控制
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
  return 1;
}

//抄单个表时  发送单个表数据
void send_meter_data_single(uint8_t * p_meteraddr,uint8_t * meter_read ,uint8_t * meter_status,uint8_t meter_type,uint8_t desc){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;
  uint8_t frame_seq = 0;
  uint8_t frame_data_len = 0;

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;

    frame_seq = addSEQ();
    set_data_seq(frame_seq);
    frame_data_len = 9+14+5;
    p_buf = ack_mulit_header(p_buf,0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);


    p_buf_16 = (uint16_t *)p_buf;
    *p_buf_16++ = 1;    //总共多少帧
    *p_buf_16++ = 1;  //第几帧
    p_buf = (uint8_t *)p_buf_16;
    *p_buf++ = meter_type;

    for(i=0;i<7;i++){
      *p_buf++ = p_meteraddr[i];
    }
    *p_buf++ = 0x01;
    for(i=0;i<4;i++){
      *p_buf++ = meter_read[i];
    }
    *p_buf++ = meter_status[0];
    *p_buf++ = meter_status[1];

    *p_buf++ = check_cs(p_buf_+6,frame_data_len);
    *p_buf++ = FRAME_END;

    for(i = 0;i < 3;i++){
      switch(desc){
      case 0x01:
        if(lock_lora()){
          write_lora(p_buf_,p_buf-p_buf_);
          unlock_lora();
        }
        break;
      case 0x00:
        if(lock_cjq()){
          write_cjq(p_buf_,p_buf-p_buf_);
          unlock_cjq();
        }
        break;
      }

      if(wait_jzqack(10000)){ //wait ack
        break;
      }
    }
    put_membuf(p_buf_);
  }
}

void send_meter_data_all(uint8_t desc){
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;
  uint16_t meter_count = 0;

  uint16_t send_times = 0;  //所有表 发送总次数
  uint16_t sended = 0;  //已经发送的
  uint8_t remains = 0;
  uint16_t c = 0;

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  //获取所有通道所有的表加起来一共要发送多少次
  for(c = 0;c < cjq_count;c++){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    send_times = send_times + meter_count/5;
    if(remains){
      send_times = send_times + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  for(c = 0;c < cjq_count;c++){
    send_meter_data_channel(block_cjq,send_times,sended,0x10,desc);

    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%5;
    sended = sended + meter_count/5;
    if(remains){
      sended = sended + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }
}


//发送采集器单个通道的数据    TODO... 帧的地址域为当前采集器地址
void send_meter_data_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t meter_type,uint8_t desc){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint16_t i = 0;
  uint8_t j = 0;
  uint8_t k = 0;
  uint8_t frame_meter_count = 0;
  uint8_t meter_addr[7];
  uint8_t meter_read[4];
  uint8_t meter_status[2];

  uint32_t block_cjq = block_cjq_;
  uint32_t block_meter = 0;
  uint16_t cjqmeter_count = 0;
  uint8_t frame_data_len = 0;
  uint8_t frame_seq = 0;

  uint16_t times = 0;
  uint8_t remain = 0;
  uint16_t times_ = 0;      //一共要发送多少帧

  uint8_t ack = 0;  //发送数据后是否得到ack

  p_buf = get_membuf();
  if(p_buf > 0){
    p_buf_ = p_buf;

    sFLASH_ReadBuffer((uint8_t *)&block_meter,block_cjq+CJQ_FLASH_INDEX_FIRSTMETER,3);
    sFLASH_ReadBuffer((uint8_t *)&cjqmeter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);

    if(cjqmeter_count > 0){
      times = cjqmeter_count/5;
      remain = cjqmeter_count%5;
      times_ = times;
      if(remain > 0){
        times_ = times_ + 1;
      }

      for(i=0;i< times_;i++){
        p_buf = p_buf_;
        frame_seq = addSEQ();
        set_data_seq(frame_seq);
        frame_data_len = 0;
        frame_meter_count = 0;
        if(times_==1){//单帧 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+14*frame_meter_count+5;
          p_buf = ack_mulit_header(p_buf,0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
        }else{
          if(i==0){//首帧 1
            frame_meter_count = 5;
            frame_data_len = 9+14*frame_meter_count+5;
            p_buf = ack_mulit_header(p_buf,1,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
          }else{
            if(i==times_-1){//尾帧 3
              frame_meter_count = remain;
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,3,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
            }else{//中间帧 2
              frame_meter_count = 5;
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,2,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
            }
          }
        }

        //帧的数据域
        p_buf_16 = (uint16_t *)p_buf;
        if(frame_times){
          *p_buf_16++ = frame_times;  //总次数
        }else{
          *p_buf_16++ = times_;
        }
        *p_buf_16++ = i+1+frame_times_start;  //当前次数
        p_buf = (uint8_t *)p_buf_16;
        *p_buf++ = meter_type;

        //表数据
        for(k=0;k<frame_meter_count;k++){
          sFLASH_ReadBuffer((uint8_t *)&meter_addr,block_meter+METER_FLASH_INDEX_ADDR,7);
          sFLASH_ReadBuffer((uint8_t *)&meter_read,block_meter+METER_FLASH_INDEX_READ,4);
          sFLASH_ReadBuffer((uint8_t *)&meter_status,block_meter+METER_FLASH_INDEX_METERSTATE,2);
          for(j=0;j<7;j++){
            *p_buf++ = meter_addr[j];
          }
          *p_buf++ = 0x01;
          for(j=0;j<4;j++){
            *p_buf++ = meter_read[j];
          }
          *p_buf++ = meter_status[0];
          *p_buf++ = meter_status[1];
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
        }

        *p_buf++ = check_cs(p_buf_+6,frame_data_len);
        *p_buf++ = FRAME_END;


        for(k = 0;k < 3;k++){
          ack = 0;
          switch(desc){
          case 0x01:
            if(lock_lora()){
              write_lora(p_buf_,p_buf-p_buf_);
              unlock_lora();
            }
            break;
          case 0x00:
            if(lock_cjq()){
              write_cjq(p_buf_,p_buf-p_buf_);
              lock_cjq();
            }
            break;
          }

          if(wait_jzqack(10000)){ //wait ack
            ack = 1;
            break;
          }
          delayms(100);
        }
        if(!ack){
          break;
        }
      }
    }
    put_membuf(p_buf_);
  }
}
