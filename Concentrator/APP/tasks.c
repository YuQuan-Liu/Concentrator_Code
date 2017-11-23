

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "gprs.h"
#include "frame.h"
#include "frame_188.h"
#include "utils.h"
#include "configs.h"
#include "bsp.h"
#include "device_params.h"
#include "readmeter.h"


extern uint8_t * volatile p_server;      //中断中保存GPRS 返回来的数据
extern uint8_t * volatile p_server_;     //记录中断的开始指针


/**
 * 处理meter usart接收到的数据
 */
void task_meter_raw(void *p_arg){
  uint8_t * p_mem = 0;    //the ptr get from the queue
  uint16_t msg_size = 0;    //the message's size 
  uint8_t msg_data = 0;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_ = 0;       //keep the buf's ptr  used to release the buf
  uint8_t post_q_result = 0;
  uint32_t * p_tmp_32 = 0;
  
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    if(!wait_q_meter_usart(&p_mem,&msg_size,200)){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){ //get the buf
        p_buf_ = p_buf;
      }else{ //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_meter_frame(p_buf_,p_buf)){
    case 2: //当前帧错误
      p_buf = p_buf_;
      break;
    case 0: //当前帧没有接收完
      break;
    case 1: //当前帧接收完毕
      if(get_readding()){
        
        p_tmp_32 = (uint32_t *)p_buf;
        *p_tmp_32 = get_timestamp();
        post_q_result = post_q_meter(p_buf_,p_buf-p_buf_);
        
        if(post_q_result){
          p_buf_ = 0;
          p_buf = 0;
        }else{
          p_buf = p_buf_;
        }
      }else{
        p_buf = p_buf_;
      }
      break;
    }
  }
}

/**
 * 有线采集器接口接收到的原始数据
 */
void task_cjq_raw(void *p_arg){
  uint8_t * p_mem = 0;    //the ptr get from the queue
  uint16_t msg_size = 0;    //the message's size 
  uint8_t msg_data = 0;         //the data get from the queue
  
  uint16_t frame_len = 0;
  uint8_t server_seq_ = 0; //服务器发送过来的数据 的序列号
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_ = 0;       //keep the buf's ptr  used to release the buf
  uint32_t * p_tmp_32 = 0;
  
  uint8_t post_q_result = 0;
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    if(!wait_q_cjq_usart(&p_mem,&msg_size,200)){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){  //get the buf
        p_buf_ = p_buf;
      }else{  //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_xintian_frame(p_buf_,p_buf)){
    case 2: //当前帧错误
      p_buf = p_buf_;
      break;
    case 0: //当前帧没有接收完
      break;
    case 1:  //当前帧接收完毕
      //抄表状态下 采集器返回过来的数据帧  或者 应答帧
      frame_len = check_frame(p_buf_);
      server_seq_ = *(p_buf_+SEQ_POSITION) & 0x0F;  //获得该帧的序列号
      *p_buf = 0x00;//标识这一帧数据是来自485的
      
      switch(cjq_data_tome(p_buf_,frame_len)){
      case 1:  //我正在处理的采集器   除ACK外 全部post到q_cjq
        switch(*(p_buf_+AFN_POSITION)){
        case AFN_ACK:
          if(server_seq_ == get_cjq_data_seq()){
            signal_cjqack();
          }
          p_buf = p_buf_;
          break;
        default:
          //TODO...这个地方应该差一个ACK  告诉采集器 哥收到了  
          //到这个地方的有采集器返回的抄表数据  读取采集器所有表的数据
          if(*(p_buf_+AFN_POSITION) == AFN_QUERY && *(p_buf_+FN_POSITION)==FN_READING){
            //NO ACK
          }else{
            if(*(p_buf_+AFN_POSITION) == AFN_QUERY && *(p_buf_+FN_POSITION)==FN_METER){
              //NO ACK
            }else{
              device_ack_cjq(0,server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
            }
          }
          p_tmp_32 = (uint32_t *)p_buf;
          *p_tmp_32 = get_timestamp();
          post_q_result = post_q_cjq(p_buf_, p_buf-p_buf_);  
          if(post_q_result){
            p_buf_ = 0;
            p_buf = 0;
          }else{
            p_buf = p_buf_;
          }
          break;
        }
        break;
      case 2:  //Programmer 发送过来的配置指令
        switch(*(p_buf_+AFN_POSITION)){
        case AFN_CONFIG:
        case AFN_QUERY:
          post_q_result = post_q_conf(p_buf_, frame_len);
          if(post_q_result){
            p_buf_ = 0;
            p_buf = 0;
          }else{
            p_buf = p_buf_;
          }
          break;
        case AFN_CONTROL:
        case AFN_CURRENT:
          device_ack(0,server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //ACK
          if(!get_readding()){
            post_q_result = post_q_read(p_buf_, frame_len);
            if(post_q_result){
              p_buf_ = 0;
              p_buf = 0;
            }else{
              p_buf = p_buf_;
            }
          }else{
            p_buf = p_buf_;  //我在抄表  放弃此帧
          }
          break;
        case AFN_ACK:
          if(server_seq_ == get_server_data_seq()){
            signal_serverack();
          }
          p_buf = p_buf_;
          break;
        default:
          p_buf = p_buf_;
          break;
        }
        break;
      default:
        p_buf = p_buf_;
        break;
      }
      break;
    }
  }
}

/**
 * 处理LORA接收到的指令
 */
void task_lora_raw(void *p_arg){
  uint8_t * p_mem = 0;    //the ptr get from the queue
  uint16_t msg_size = 0;    //the message's size 
  uint8_t msg_data = 0;         //the data get from the queue
  
  uint16_t frame_len = 0;
  uint8_t server_seq_ = 0; //服务器发送过来的数据 的序列号
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_ = 0;       //keep the buf's ptr  used to release the buf
  uint32_t * p_tmp_32 = 0;
  
  uint8_t post_q_result = 0;
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    if(!wait_q_lora_usart(&p_mem,&msg_size,200)){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){ //get the buf
        p_buf_ = p_buf;
      }else{  //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_lora_data2frame(p_buf_,p_buf)){
    case 2: //当前帧错误
      p_buf = p_buf_;
      break;
    case 0: //当前帧没有接收完
      break;
    case 1: //当前帧接收完毕
      if(*(p_buf_) == 0x68){
        server_seq_ = *(p_buf_+SEQ_POSITION) & 0x0F;
        switch(cjq_data_tome(p_buf_,frame_len)){
        case 1:  //我正在处理的采集器   除ACK外 全部post到q_cjq
          switch(*(p_buf_+AFN_POSITION)){
          case AFN_ACK:
            if(server_seq_ == get_cjq_data_seq()){
              signal_cjqack();
            }
            p_buf = p_buf_;
            break;
          default:
            //TODO...这个地方应该差一个ACK  告诉采集器 哥收到了 
            //到这个地方的有采集器返回的抄表数据  读取采集器所有表的数据
            if(*(p_buf_+AFN_POSITION) == AFN_QUERY && *(p_buf_+FN_POSITION)==FN_READING){
              //NO ACK
            }else{
              if(*(p_buf_+AFN_POSITION) == AFN_QUERY && *(p_buf_+FN_POSITION)==FN_METER){
                //NO ACK
              }else{
                device_ack_cjq(1,server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);
              }
            }
            
            p_tmp_32 = (uint32_t *)p_buf;
            *p_tmp_32 = get_timestamp();
            post_q_result = post_q_cjq(p_buf_, p_buf-p_buf_);  
            if(post_q_result){
              p_buf_ = 0;
              p_buf = 0;
            }else{
              p_buf = p_buf_;
            }
            break;
          }
          break;
        case 2:  //Programmer 发送过来的配置指令  LORA不支持
          p_buf = p_buf_;
          break;
        default:
          p_buf = p_buf_;
          break;
        }
      }
      if(*(p_buf_) == 0x0D){
        //+++/AT+ESC return
        signal_lora_ok();
        p_buf = p_buf_;
      }
      break;
    }
  }
}




/**
 * 连接服务器 处理GPRS模块发送过来的指令
 */
void task_server(void *p_arg){
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t getbuffail = 0;
  uint8_t connectfail = 0;
  
  while(DEF_TRUE){
    if(get_connect_state()){
      if(p_buf == 0){
        p_buf = get_membuf();
        if(p_buf > 0){  //get the buf
          getbuffail = 0;
          p_buf_ = p_buf;
          server_2buf(p_buf);
        }else{ //do not get the buf
          getbuffail ++;
//          if(getbuffail >= 20){
//            //20次没有获取到buf  
//            device_cmd(0);
//            *((uint8_t *)0) = 0x00;  //迫使系统重启
//          }
          delayms(200);
          continue;
        }
      }
      
      if(p_server - p_server_ > 0){
        delayms(4);
        
        p_buf = p_server;
        replace_str00(p_buf_,p_buf);  //屏蔽掉数据前的0x00
        if(Str_Str(p_buf_,"\n>")){
          
          p_buf = p_buf_;
          Mem_Set(p_buf_,0x00,256); //clear the buf
          server_2buf(p_buf);
          
          signal_sendgprs();
          continue;
        }
        
        if(Str_Str(p_buf_,"RECEIVE")){
          //oh it's the data  \r\n+TCPRECV:0,**,0x68 L L 0x68 C A     \r\n
          if(post_q_server_action(p_buf_,p_buf - p_buf_)){
            p_buf = 0;
            p_buf_ = 0;
            server_2buf(0);
          }else{
            p_buf = p_buf_;
            Mem_Set(p_buf_,0x00,256); //clear the buf
            server_2buf(p_buf);
          }
          continue;
        }
        
        if(Str_Str(p_buf_,"CLOSE") || Str_Str(p_buf_,"DEACT") || Str_Str(p_buf_,"ERROR")){
          //CLOSED   +PDP: DEACT\r\n
          Mem_Set(p_buf_,0x00,256); //clear the buf
          put_membuf(p_buf_);
          p_buf = 0;
          p_buf_ = 0;
          server_2buf(0);
          set_connect_state(0);
          continue;
        }
        
        //do not know what's that
        p_buf = p_buf_;
        Mem_Set(p_buf_,0x00,256); //clear the buf
        server_2buf(p_buf_);
      }else{
        delayms(4);
      }
    }else{
      
      if(p_buf_ != 0){
        Mem_Set(p_buf_,0x00,256); //clear the buf
        put_membuf(p_buf_);
        p_buf = 0;
        p_buf_ = 0;
        server_2buf(0);
      }
      
      device_cmd(0);
      device_cmd(1);
      if(connect()){
        connectfail = 0;
      }else{
        connectfail++;
//        if(connectfail > 20){
//          device_cmd(0);
//          *((uint8_t *)0) = 0x00;  //迫使系统重启
//        }
      }
    }
  }
}

/**
 *
 */
void task_server_action(void *p_arg){
  uint16_t msg_size = 0;
  uint8_t * p_buf = 0;
    
  uint8_t *frame_start = 0; //接收到的帧的起始地址
  uint8_t frame_len = 0;  //接收到帧的总长度
  uint8_t server_seq_ = 0; //服务器发送过来的数据 的序列号
  
  uint8_t * p_buf_copy = 0;  //复制配置帧 抄表帧 post 到队列
  uint8_t post_q_result = 0;
  while(DEF_TRUE){
    wait_q_server_action(&p_buf,&msg_size,0);
    
    frame_start = Str_Str(p_buf,"\r\n\x68") + 2;
    frame_len = check_frame(frame_start);  //check the frame get the length
    if(frame_len){ //the frame is ok
      server_seq_ = *(frame_start + SEQ_POSITION) & 0x0F;  //获得该帧的序列号
      switch(*(frame_start+AFN_POSITION)){
      case AFN_ACK:  //the ack of the server
        if(server_seq_ == get_server_data_seq()){
          signal_serverack();
        }
        break;
      case AFN_CONFIG:
      case AFN_QUERY:
        p_buf_copy =  get_membuf();
        if(p_buf_copy > 0){
          Mem_Copy(p_buf_copy,frame_start,frame_len);
          *(p_buf_copy + frame_len) = 0x01;  //标识这一帧来自服务器
          post_q_result = post_q_conf(p_buf_copy, frame_len);
          if(!post_q_result){
            put_membuf(p_buf_copy);
          }
        }
        break;
      case AFN_CONTROL:
      case AFN_CURRENT:
        device_ack(0x01,server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //ACK
        if(!get_readding()){
          p_buf_copy =  get_membuf();
          if(p_buf_copy > 0){
            Mem_Copy(p_buf_copy,frame_start,frame_len);
            *(p_buf_copy + frame_len) = 0x01;  //标识这一帧来自服务器
            post_q_result = post_q_read(p_buf_copy, frame_len);
            if(!post_q_result){
              put_membuf(p_buf_copy);
            }
          }
        }
        break;
      case AFN_LINK_TEST: //never will come to here
      case AFN_HISTORY://don't support
        break;
      default:
        break;
      }
    }
    
    put_membuf(p_buf);
  }
}

/**
 * GPRS 心跳任务
 */
void task_heartbeat(void *p_arg){
  uint8_t beat[17];
  uint8_t * p_beat = 0;
  uint8_t heart_ack = 0;
  uint8_t i = 0;
  uint8_t frame_seq = 0;
  uint8_t * p_deviceaddr = 0;
  
  while(DEF_TRUE){
    if(get_connect_state()){
      if(lock_gprs()){
        for(i = 0;get_connect_state() && i < 3;i++){
          heart_ack = 0;
          
          p_beat = beat;
          *p_beat++ = FRAME_HEAD;
          *p_beat++ = 0x27;//(9 << 2) | 0x03;
          *p_beat++ = 0x00;
          *p_beat++ = 0x27;//(9 << 2) | 0x03;
          *p_beat++ = 0x00;
          *p_beat++ = FRAME_HEAD;
          
          *p_beat++ = ZERO_BYTE | DIR_TO_SERVER | PRM_START | START_FUN_TEST;
          
          p_deviceaddr = get_device_addr();
          *p_beat++ = p_deviceaddr[0];
          *p_beat++ = p_deviceaddr[1];
          *p_beat++ = p_deviceaddr[2];
          *p_beat++ = p_deviceaddr[3];
          *p_beat++ = p_deviceaddr[4];
          
          *p_beat++ = AFN_LINK_TEST;
          
          frame_seq = add_server_seq();
          set_server_data_seq(frame_seq);
          *p_beat++ = ZERO_BYTE |SINGLE | CONFIRM | frame_seq;
          
          *p_beat++ = FN_HEARTBEAT;
          
          *p_beat++ = check_cs(beat+6,9);
          *p_beat++ = FRAME_END;
          if(send_server(beat,17)){
            if(wait_serverack(5000)){
              heart_ack = 1;
              break;
            }
          }
        }
        unlock_gprs();
        if(heart_ack){  //接收到心跳ACK
          delayms(120000);
        }else{   //3次都没有收到心跳ACK
          set_connect_state(0);
        }
      }else{
        delayms(1000);  //没有得到GPRS锁  1s后再试
      }
    }else{
      delayms(1000);  //不在线 1s后再试
    }
  }
}


/**
 * 定时检测LORA模块是否OK  
 */
void task_lora_check(void *p_arg){
  uint8_t inat = 0;
  uint8_t outat = 0;
  uint8_t i = 0;
  
  while(DEF_TRUE){
    //2min  每2分钟检测一次
    delayms(120000);
    if(get_readding() || get_syning()){
      continue;
    }
    
    if(get_device_mode() != 0xFF){  
      continue;//不是无线
    }
        
    if(lock_lora()){
      //尝试3次
      for(i = 0; i < 3;i++){
        inat = 0;
        outat = 0;
        write_lora("+++",3); //进入AT模式
        if(wait_lora_ok(1000)){
          inat = 1;
        }
        
        if(inat){
          write_lora("AT+ESC\r\n",8);  //离开AT模式
          if(wait_lora_ok(1000)){
            outat = 1;
          }
        }
        
        if(inat == 1 && outat == 1){
          break;
        }
      }
    }
    unlock_lora();
    
    //检测失败
    if(inat == 0 || outat == 0){
      //restart LORA
      PWR_LORA_OFF();
      delayms(10000);
      PWR_LORA_ON();
    }
  }
}


/**
 * 具体的抄表任务
 */
void task_read(void *p_arg){
  uint16_t msg_size = 0;
  uint8_t * p_buf = 0;
  
  while(DEF_TRUE){
    wait_q_read(&p_buf,&msg_size,0);
    
    set_readding(1);
    if(lock_mem4k()){
      switch(*(p_buf+msg_size)){
      case 0x01:
        if(lock_gprs()){
          switch(*(p_buf+AFN_POSITION)){
          case AFN_CONTROL:
            meter_control(p_buf,msg_size);
            break;
          case AFN_CURRENT:
            meter_read(p_buf,msg_size);
            break;
          }
          unlock_gprs();
        }
        break;
      default:
        if(lock_cjq()){
          switch(*(p_buf+AFN_POSITION)){
          case AFN_CONTROL:
            meter_control(p_buf,msg_size);
            break;
          case AFN_CURRENT:
            meter_read(p_buf,msg_size);
            break;
          }
          unlock_cjq();
        }
        break;
      }
      unlock_mem4k();
    }
    set_readding(0);
    
    put_membuf(p_buf);
  }
}

/**
 * config and query the parameter
 */
void task_config(void *p_arg){
  uint16_t msg_size = 0;
  uint8_t * p_buf = 0;
  
  while(DEF_TRUE){
    wait_q_conf(&p_buf,&msg_size,0);
    
    switch(*(p_buf+msg_size)){
    case 0x01:
      if(lock_gprs()){
        switch(*(p_buf+AFN_POSITION)){
        case AFN_CONFIG:
          param_config(p_buf,msg_size);
          break;
        case AFN_QUERY:
          param_query(p_buf,msg_size);
          break;
        }
        unlock_gprs();
      }
      break;
    default:
      if(lock_cjq()){
        switch(*(p_buf+AFN_POSITION)){
        case AFN_CONFIG:
          param_config(p_buf,msg_size);
          break;
        case AFN_QUERY:
          param_query(p_buf,msg_size);
          break;
        }
        unlock_cjq();
      }
      break;
    }
    
    put_membuf(p_buf);
  }
}


/**
 * LED2
 * 抄表指示  
 * 安装时测试LORA信号
 */
void task_led(void *p_arg){
  
  while(DEF_TRUE){
    while(get_lora_test()){
      LED2_OFF();
      LED3_ON();
      write_lora("TEST",4); //定时发送供采集器测试信号使用
      delayms(3500);
      LED3_OFF();
      delayms(500);
    }
    
    //LED2
    if(get_readding()){
      LED2_ON();
      delayms(100);
      LED2_OFF();
      delayms(100);
    }else{
      LED2_ON();
      delayms(1000);
      LED2_OFF();
      delayms(1000);
    }
  }
}

extern OS_FLAG_GRP FLAG_Event;
//MBUS过载
void task_overload(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t cnt = 0;
  
  while(DEF_TRUE){
    
    OSFlagPend(&FLAG_Event,
               MBUSOVERLOAD,
               0,
               OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME + OS_OPT_PEND_BLOCKING,
               &ts,
               &err);
    
    delayms(300);
    if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12)){
      cjq_relay_control(0,1); //关采集器通道
      cjq_relay_control(0,2);
      cjq_relay_control(0,3);
      cjq_relay_control(0,4);
      
      while(cnt < 100){
        LED3_ON();
        delayms(100);
        LED3_OFF();
        delayms(100);
        cnt++;
      }
      cnt = 0;
    }
    
  }
}


void task_syn(void *p_arg){
  
  while(DEF_TRUE){
    wait_syn(0);
    set_syning(1);
    switch(get_device_mode()){
    case 0xFF:
      if(lock_lora()){
        sync_data2cjq(get_cjq_addr());//同步CJQ所有通道数据到 采集器
        unlock_lora();
      }
      break;
    case 0xAA:
      if(lock_cjq()){
        sync_data2cjq(get_cjq_addr());//同步CJQ所有通道数据到 采集器
        unlock_cjq();
      }
      break;
    }
    set_syning(0);
  }
}
