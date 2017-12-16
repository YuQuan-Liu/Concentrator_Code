
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
#include "gprs.h"

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
  uint32_t block_meter = 0;
  uint16_t c = 0;
  uint8_t * p_all_cjq = 0;

  uint8_t cjq_addr[5];
  uint8_t cjq_count_inbuf = 0;  //MEMBUF中一共有多少个采集器

  uint8_t frame_data[2];
  uint8_t cjq_seq = 0;
  uint8_t ack = 0;
  uint8_t i = 0;

  uint8_t timeout_count = 30;
  uint8_t read_over_cjq = 0;   //抄表完成的采集器(包括超时)
  uint8_t all_read_over = 0;  //所有的采集器是否都抄表完成了
  uint8_t cjq_state = 0;   //当前采集器的抄表状态
  uint8_t read_over = 2;  //0~抄表中  1~抄表结束  2~超时  查询采集器是否在抄表 对应的状态

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;

  uint16_t all_frames = 0;
  uint16_t this_frame = 0;
  uint8_t frame_metercount = 0;


  uint8_t * p_meteraddr = 0;
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t m = 0;
  uint8_t z = 0;
  uint8_t * p_cjqaddr = 0;
  
  uint32_t send_ts = 0;  //发送指令的时钟
  
  p_all_cjq = get_membuf();
  if(p_all_cjq > 0){
    /*
    p_all_cjq   256存储一共有多少个采集器  255存储已经读完了几个采集器
    每个采集器占10个字节  12345~采集器地址   6~是否完成~[00~未完成  11~完成  22~超时]
    7~是否接收到采集器数据  00~没有接收到   11~接收到
    */
    Mem_Set(p_all_cjq,0x00,256);

    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

    for(c = 0;c < cjq_count;c++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);

      if(!search_cjq_in_membuf(p_all_cjq,cjq_addr)){
        cjq_count_inbuf = p_all_cjq[255];
        Mem_Copy(p_all_cjq + cjq_count_inbuf*10 + 1, cjq_addr + 1,4);
        p_all_cjq[255] = cjq_count_inbuf + 1;
      }

      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }

    //集中器下挂的采集器设备的个数
    cjq_count_inbuf = p_all_cjq[255];

    read_over_cjq = 0;   //抄表完成的采集器(包括超时)
    //依次给采集器发送抄表指令
    for(c = 0; c < cjq_count_inbuf;c++){
      frame_data[0] = 0xFF;  //抄采集器全部表
      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);

      ack = 0;
      for(i = 0;i < 3;i++){  //给采集发送指令 尝试3次
        set_cjq_addr(p_all_cjq+10*c);
        if(write_frame_cjq(p_all_cjq+10*c, frame_data, 1,AFN_CURRENT,FN_METER,cjq_seq)){ //给采集器发送抄表指令
          if(wait_cjqack(10000)){  //等待10s采集器对抄表指令ACK
            ack = 1;
            break;
          }
        }
      }
      if(ack){
        p_all_cjq[10*c+5] = 0x00;
      }else{
        p_all_cjq[10*c+5] = 0x22;
        read_over_cjq++;
      }
    }

    //每10s遍历一遍所有的采集器  看是否抄表完成  等待5min
    timeout_count = 30;
    all_read_over = 0;  //所有的采集器是否都抄表完成了
    while(timeout_count > 0){
      timeout_count++;
      delayms(10000);

      for(c = 0; c < cjq_count_inbuf;c++){
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x00){   //判断采集器状态  还在抄表...
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          read_over = 2;  //0~抄表中  1~抄表结束  2~超时
          for(i = 0;i < 3;i++){  //给采集发送指令 尝试3次
            set_cjq_addr(p_all_cjq+10*c);
            if(write_frame_cjq(p_all_cjq+10*c, (uint8_t *)0, 0,AFN_QUERY,FN_READING,cjq_seq)){ //给采集器发送询问是否在抄表
              send_ts = get_timestamp();
              if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
                //判断采集器是否抄表完成
                if(*(p_response + DATA_POSITION)){  //还在抄表
                  read_over = 0;
                }else{  //抄表结束
                  read_over = 1;
                }
                put_membuf(p_response);
                break;
              }
            }
          }
          switch(read_over){
          case 0:
            p_all_cjq[10*c+5] = 0x00;
            break;
          case 1:
            p_all_cjq[10*c+5] = 0x11;
            read_over_cjq++;
            break;
          case 2:
            read_over_cjq++;
            p_all_cjq[10*c+5] = 0x22;
            break;
          }
        }
      }
      
      //往上层发送抄表进度帧
      frame_data[0] = read_over_cjq;
      frame_data[1] = cjq_count_inbuf;
      device_ack(*(p_frame+frame_len),add_server_seq(),frame_data,2,AFN_FAKE,FN_ACK);
      
      if(read_over_cjq == cjq_count_inbuf){
        all_read_over = 1;
        break;
      }
    }

    if(all_read_over){//全部抄表完成
      //依次遍历所有抄表完成的采集器  获取最新的数据
      for(c = 0; c < cjq_count_inbuf;c++){
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x11){   //判断采集器状态为抄表完成
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          for(i = 0;i < 3;i++){  //给采集发送指令 尝试3次
            set_cjq_addr(p_all_cjq+10*c);
            if(write_frame_cjq(p_all_cjq+10*c, (uint8_t *)0, 0,AFN_QUERY,FN_ALL_READDATA,cjq_seq)){ //给采集器发送把全部表的抄表结果交出来
              send_ts = get_timestamp();
              timeout_count = 10;
              while(timeout_count > 0){
                if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
                  //帧的长度msg_size  计算此帧中一共有多少表 frame_metercount
                  //一共有多少帧all_frames  这是第几帧this_frame
                  all_frames = *(p_response + DATA_POSITION) | *(p_response +DATA_POSITION+1)<<8;
                  this_frame = *(p_response + DATA_POSITION+2) | *(p_response + DATA_POSITION+3)<<8;
                  frame_metercount = (msg_size-8-9-5)/14;

                  for(m = 0;m < frame_metercount;m++){
                    //从帧中获取  表地址 表的读数  表的状态
                    p_meteraddr = p_response + DATA_POSITION + 5 + 14 * m;
                    p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * m;
                    p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * m;

                    //根据当前查询的采集器地址  搜索采集器块 表块
                    p_cjqaddr = get_cjq_addr();
                    for(z = 1;z <= 4;z++){
                      *(p_cjqaddr) = z;
                      block_cjq = search_cjq(p_cjqaddr);
                      if(block_cjq){
                        block_meter = search_meter(block_cjq,p_meteraddr);
                        if(block_meter){  //保存数据
                          meter_read_save(block_meter,p_meter_read,p_meter_status);
                          break;
                        }
                      }
                    }
                    *(p_cjqaddr) = 0;
                  }
                  put_membuf(p_response);
                  if(all_frames == this_frame){
                    break;
                  }
                }else{
                  timeout_count--;
                }
              }

              if(timeout_count > 0){ //接收所有的帧OK
                p_all_cjq[10*c+6] = 0x11;
                break;
              }
            }
          }
        }
        //往上层发送读采集器抄表结果进度帧
        frame_data[0] = c+1;
        frame_data[1] = cjq_count_inbuf;
        device_ack(*(p_frame+frame_len),add_server_seq(),frame_data,2,AFN_FAKE,FN_ACK);
      }

      //所有最新的采集器的数据  全部接收到了集中器中
      //将数据发送出去
      send_meter_data_all(*(p_frame + frame_len),p_all_cjq);
      
    }else{  //抄表超时  这个的可能性不大  暂且不管

    }
    put_membuf(p_all_cjq);
  }
}

void meter_read_c_channel(uint8_t * p_frame,uint16_t frame_len){
  uint32_t block_meter = 0;
  uint32_t block_cjq = 0;

  uint8_t * p_cjqaddr = 0;
  uint8_t * p_meteraddr = 0;

  uint8_t frame_data[6];
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;

  uint8_t timeout_count = 10; //如果出现10次超时就判定为失败
  uint16_t all_frames = 10;  //表示一共会返回多少帧
  uint16_t this_frame = 0;   //表示这是所有帧中的第几帧
  uint8_t frame_metercount = 0;  //接收到的帧中有几块表的数据
  uint8_t meter_type = 0;

  uint32_t send_ts = 0;  //发送指令的时钟
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){  //find the cjq
    //将此抄表指令发送给采集器
    frame_data[0] = *(p_frame+DATA_POSITION);
    for(i = 0;i < 5;i++){
      frame_data[i+1] = p_cjqaddr[i];
    }

    cjq_seq = add_cjq_seq();
    set_cjq_data_seq(cjq_seq);
    set_cjq_addr(p_cjqaddr);
    if(write_frame_cjq(p_cjqaddr, frame_data, 6,AFN_CURRENT,FN_METER,cjq_seq)){ //给采集器发送抄表指令
      send_ts = get_timestamp();
      if(wait_cjqack(10000)){  //等待10s采集器对抄表指令ACK
        while(timeout_count > 0){
          if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
            //帧的长度msg_size  计算此帧中一共有多少表 frame_metercount
            //一共有多少帧all_frames  这是第几帧this_frame
            all_frames = *(p_response + DATA_POSITION) + *(p_response +DATA_POSITION+1)<<8;
            this_frame = *(p_response + DATA_POSITION+2) + *(p_response + DATA_POSITION+3)<<8;
            frame_metercount = (msg_size-8-9-5)/14;
            
            meter_type = *(p_response + DATA_POSITION + 4);
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

        //send the data out  
        if(timeout_count > 0){ //接收所有的帧OK
          send_meter_data_channel(block_cjq,0,0,meter_type,*(p_frame+frame_len),0);
        }else{ //接收所有的表数据超时   暂且不管 TODO...
          
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

  uint8_t frame_data[13];
  uint8_t * p_meter_read = 0;
  uint8_t * p_meter_status = 0;

  uint8_t * p_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t cjq_seq = 0;
  uint8_t meter_type = 0;

  uint32_t send_ts = 0;  //发送指令的时钟
  
  p_cjqaddr = p_frame+DATA_POSITION+1;
  p_meteraddr = p_cjqaddr + 5;

  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){
    block_meter = search_meter(block_cjq,p_meteraddr);
    if(block_meter){
      //find the meter under the cjq
      //将此抄表指令发送给采集器
      frame_data[0] = *(p_frame+DATA_POSITION);
      for(i = 0;i < 5;i++){
        frame_data[i+1] = p_cjqaddr[i];
      }
      for(i = 0;i < 7;i++){
        frame_data[i+6] = p_meteraddr[i];
      }

      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);
      set_cjq_addr(p_cjqaddr);
      if(write_frame_cjq(p_cjqaddr, frame_data, 13,AFN_CURRENT,FN_METER,cjq_seq)){ //给采集器发送抄表指令
        send_ts = get_timestamp();
        if(wait_cjqack(10000)){  //等待10s采集器对抄表指令ACK
          if(wait_q_cjq_ts(&p_response,&msg_size,10000,send_ts)){
            //从帧中获取 表的读数  表的状态
            meter_type = *(p_response + DATA_POSITION + 4);
            p_meter_read = p_response + DATA_POSITION + 5 + 8;
            p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4;
            //保存数据
            meter_read_save(block_meter,p_meter_read,p_meter_status);
            //send the data out  
            send_meter_data_single(p_meteraddr,p_meter_read,p_meter_status,meter_type,*(p_frame+frame_len));
            put_membuf(p_response);
          }
        }
      }
    }
  }
}


uint8_t write_frame_cjq(uint8_t * p_cjqaddr,uint8_t * p_data,uint8_t data_len,uint8_t afn,uint8_t fn,uint8_t server_seq_){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint16_t * p_buf_16 = 0;
  uint8_t i = 0;
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

    frame_seq = add_server_seq();
    set_server_data_seq(frame_seq);
    frame_data_len = 9+14+5;
    p_buf = ack_mulit_header(p_buf,get_device_addr(),0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);


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
        if(lock_gprs()){
          send_server(p_buf_,p_buf-p_buf_);
          unlock_gprs();
        }
        break;
      case 0x00:
        if(lock_cjq()){
          write_cjq(p_buf_,p_buf-p_buf_);
          unlock_cjq();
        }
        break;
      }

      if(wait_serverack(10000)){ //wait ack
        break;
      }
    }
    put_membuf(p_buf_);
  }
}

//检查采集器的数据是否接收到了
uint8_t check_cjq_timeout(uint8_t * p_all_cjq,uint8_t * cjq_addr){
  uint8_t i = 0;
  uint8_t cjq_result = 0;
  uint8_t cjq_count_inbuf = p_all_cjq[255];
  
  //依次给采集器发送抄表指令
  for(i = 0; i < cjq_count_inbuf;i++){
    if(Mem_Cmp(p_all_cjq+10*i+1, cjq_addr+1, 4)){
      cjq_result = *(p_all_cjq+10*i+6);
      if(cjq_result == 0x00){
        cjq_result = 1;
      }
      if(cjq_result == 0x11){
        cjq_result = 0;
      }
      break;
    }
  }
  
  return cjq_result;
}

void send_meter_data_all(uint8_t desc,uint8_t * p_all_cjq){
  uint16_t cjq_count = 0;
  uint32_t block_cjq = 0;
  uint16_t meter_count = 0;

  uint16_t send_times = 0;  //所有表 发送总次数
  uint16_t sended = 0;  //已经发送的
  uint8_t remains = 0;
  uint16_t c = 0;
  uint8_t cjq_addr[5];

  sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //采集器数量
  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头

  //获取所有通道所有的表加起来一共要发送多少次
  for(c = 0;c < cjq_count;c++){
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%10;
    send_times = send_times + meter_count/10;
    if(remains){
      send_times = send_times + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
  }

  sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //采集器队列头
  sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);  //采集器地址
  
  for(c = 0;c < cjq_count;c++){
    //判断采集器是否超时
    if(p_all_cjq == 0){
      send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,0);
    }else{
      if(check_cjq_timeout(p_all_cjq,cjq_addr)){//采集器超时
        send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,1);
      }else{  //没有超时
        send_meter_data_channel(block_cjq,send_times,sended,0x10,desc,0);
      }
    }
    
    sFLASH_ReadBuffer((uint8_t *)&meter_count,block_cjq+CJQ_FLASH_INDEX_METERCOUNT,2);
    remains = meter_count%10;
    sended = sended + meter_count/10;
    if(remains){
      sended = sended + 1;
    }
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);  //采集器地址
  }
}


//发送采集器单个通道的数据     帧的地址域为当前采集器地址
void send_meter_data_channel(uint32_t block_cjq_,uint16_t frame_times,uint16_t frame_times_start,uint8_t meter_type,uint8_t desc,uint8_t cjq_timeout){
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
      times = cjqmeter_count/10;
      remain = cjqmeter_count%10;
      times_ = times;
      if(remain > 0){
        times_ = times_ + 1;
      }

      for(i=0;i< times_;i++){
        p_buf = p_buf_;
        frame_seq = add_server_seq();
        set_server_data_seq(frame_seq);
        frame_data_len = 0;
        frame_meter_count = 0;
        if(times_==1){//单帧 0
          frame_meter_count = cjqmeter_count;
          frame_data_len = 9+14*frame_meter_count+5;
          p_buf = ack_mulit_header(p_buf,get_device_addr(),0,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
        }else{
          if(i==0){//首帧 1
            frame_meter_count = 10;
            frame_data_len = 9+14*frame_meter_count+5;
            p_buf = ack_mulit_header(p_buf,get_device_addr(),1,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
          }else{
            if(i==times_-1){//尾帧 3
              if(remain == 0){
                frame_meter_count = 10;
              }else{
                frame_meter_count = remain;
              }
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),3,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
            }else{//中间帧 2
              frame_meter_count = 10;
              frame_data_len = 9+14*frame_meter_count+5;
              p_buf = ack_mulit_header(p_buf,get_device_addr(),2,(frame_data_len << 2) | 0x03,AFN_CURRENT,frame_seq,FN_CURRENT_METER);
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
          if(cjq_timeout){  //采集器超时
            *p_buf++ = 0x40;
            *p_buf++ = 0x40;
          }else{
            *p_buf++ = meter_status[0];
            *p_buf++ = meter_status[1];
          }
          sFLASH_ReadBuffer((uint8_t *)&block_meter,block_meter+FLASH_POOL_NEXT_INDEX,3);
        }

        *p_buf++ = check_cs(p_buf_+6,frame_data_len);
        *p_buf++ = FRAME_END;


        for(k = 0;k < 3;k++){
          ack = 0;
          switch(desc){
          case 0x01:
            if(lock_gprs()){
              send_server(p_buf_,p_buf-p_buf_);
              unlock_gprs();
            }
            break;
          case 0x00:
            if(lock_cjq()){
              write_cjq(p_buf_,p_buf-p_buf_);
              unlock_cjq();
            }
            break;
          }

          if(wait_serverack(10000)){ //wait ack
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

/*
 * 下面为集中器直接抄表
 */

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

  send_meter_data_all(*(p_frame + frame_len),0);
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
    send_meter_data_channel(block_cjq,0,0,meter_type,*(p_frame+frame_len),0);
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
      cjq_relay_control(1,*(p_cjqaddr));
      sFLASH_ReadBuffer((uint8_t *)&meter_type,block_meter+METER_FLASH_INDEX_TYPE,1);
      if(meter_read_single(block_meter, p_meteraddr, meter_type, meter_read, meter_status)){
        //send the data out
        send_meter_data_single(p_meteraddr,meter_read,meter_status,meter_type,*(p_frame+frame_len));
      }
      cjq_relay_control(0,*(p_cjqaddr));
    }
  }
}

//只管抄表
uint8_t meter_read_single(uint32_t block_meter, uint8_t *p_meteraddr,uint8_t meter_type,uint8_t * meter_read,uint8_t * meter_status){
  uint8_t success = 0;
  uint8_t * p_meter_response = 0;
  uint16_t msg_size = 0;
  uint8_t i = 0;
  uint8_t j = 0;
  uint32_t send_ts = 0;  //发送指令的时钟
  uint32_t recv_ts = 0;  //接收映带时的时钟
  uint8_t wait_cnt = 0;
  
  for(i =0;i<4;i++){
    meter_read[i] = 0x00;
  }
  meter_status[0] = 0x00;
  meter_status[1] = 0x00;

  for(i = 0;i < 3;i++){
    success = 0;
    if(meter_read_frame_send(p_meteraddr,meter_type)){
      send_ts = get_timestamp();
      if(wait_q_meter_ts(&p_meter_response,&msg_size,1200,send_ts)){
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
      }else{  //接收数据超时
        delayms(100);
      }
    }else{   //发送数据失败
      delayms(100);
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
      *p_buf++ = check_cs(p_buf_+4,11+3);
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
    case 4:
      RELAY4_ON();
      break;
    }
    delayms(2000);
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
    case 4:
      RELAY4_OFF();
      break;
    }
    break;
  }
  return 1;
}






