

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "gprs.h"
#include "frame.h"
#include "frame_188.h"
#include "readeg.h"
#include "utils.h"
#include "configs.h"
#include "bsp.h"
#include "device_params.h"
#include "readmeter.h"

extern OS_Q Q_CJQ_USART;  //CJQ USART接收数据
extern OS_Q Q_METER_USART; //METER USART接收数据
extern OS_Q Q_LORA_USART;  //LORA USART接收数据
extern OS_Q Q_CJQ;  //LORA 485-CJQ 接收发送的数据  Q_CJQ_USART  Q_LORA_USART  处理后的帧
extern OS_Q Q_METER;   //Q_METER_USART处理后的帧
extern OS_Q Q_READ;    //Q_SERVER  Q_CJQ 处理后去抄表的帧
extern OS_Q Q_CONFIG;  //Q_SERVER  Q_CJQ 处理后去设置的帧

extern OS_TMR TMR_CJQTIMEOUT;    //打开采集器之后 20分钟超时 自动关闭通道

extern uint8_t * volatile p_server;      //中断中保存GPRS 返回来的数据
extern uint8_t * volatile p_server_;     //记录中断的开始指针


uint8_t data_seq = 0;  //记录数据的序列号 等待ack
uint8_t server_seq = 0;  //服务器端序列号  抄表时  会同步此序列号


/**
 * 处理meter usart接收到的数据
 */
void task_meter_raw(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    p_mem = OSQPend(&Q_METER_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_meter_frame(p_buf_,p_buf)){
    case 2:
      //当前帧错误
      p_buf = p_buf_;
      break;
    case 0:
      //当前帧没有接收完
      break;
    case 1:
      //当前帧接收完毕
      if(get_readding()){
        
        switch(get_protocol()){
        case 0x11:
          //EG
          frame_len = 9;
          OSQPost(&Q_METER,
                  p_buf_,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          break;
        case 0xFF:
          //188
          frame_len = *(p_buf_+10)+13;
          OSQPost(&Q_METER,
                  p_buf_,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          break;
        }
        
        if(err == OS_ERR_NONE){
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
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  uint8_t forme = 0;  //是否是找我
  
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    p_mem = OSQPend(&Q_CJQ_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_xintian_frame(p_buf_,p_buf)){
    case 2:
      //当前帧错误
      p_buf = p_buf_;
      break;
    case 0:
      //当前帧没有接收完
      break;
    case 1:
      //当前帧接收完毕
      //抄表状态下 采集器返回过来的数据帧  或者 应答帧
      frame_len = check_frame(p_buf_);
      if(get_readding()){
        forme = 0;
//        switch(get_protocol()){  //TODO...
//        case 0x11:
//          //EG
//          if(cjqaddr_eg[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr_eg[1] == *(p_buf_ + DATA_POSITION)){
//            forme = 1;
//          }
//          break;
//        case 0xFF:
//          //188
//          if(cjqaddr[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr[1] == *(p_buf_ + DATA_POSITION)){
//            forme = 1;
//          }
//          break;
//        }
        if(forme){
          if(*(p_buf_+AFN_POSITION) == AFN_ACK){
            //ACK
            signal_cjqack();
            p_buf = p_buf_;
          }else{
            //DATA
            //当前采集器   ACK   DATA to Queue  TODO...
            device_ack_lora(0,*(p_buf_ + SEQ_POSITION));
            OSQPost(&Q_CJQ,
                    p_buf_,
                    frame_len,
                    OS_OPT_POST_FIFO,
                    &err);
            if(err == OS_ERR_NONE){
              p_buf_ = 0;
              p_buf = 0;
            }else{
              p_buf = p_buf_;
            }
          }
        }else{
          p_buf = p_buf_;  //不是找我的
        }
      }else{
        //我不在抄表模式下  配置
        switch(*(p_buf_+AFN_POSITION)){
        case AFN_CONFIG:
        case AFN_QUERY:
          *p_buf = 0x00;//标识这一帧数据是来自485的
          OSQPost(&Q_CONFIG,
                  p_buf,
                  frame_len,
                  OS_OPT_POST_FIFO,
                  &err);
          if(err == OS_ERR_NONE){
            p_buf_ = 0;
            p_buf = 0;
          }else{
            p_buf = p_buf_;
          }
          break;
        default:
          p_buf = p_buf_;
          break;
        }
      }
      break;
    }
  }
}

/**
 * 处理LORA接收到的指令
 */
void task_lora_raw(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
  uint8_t forme = 0;  //是否是找我
  
  while(DEF_TRUE){
    //收到0x68之后  如果200ms 没有收到数据  就认为超时了
    p_mem = OSQPend(&Q_LORA_USART,200,OS_OPT_PEND_BLOCKING,&msg_size,&ts,&err);
    
    if(err == OS_ERR_TIMEOUT){
      p_buf = p_buf_;
      continue;
    }
    
    msg_data = *p_mem;
    put_memisr(p_mem);
    
    if(p_buf == 0){
      p_buf = get_membuf();
      if(p_buf > 0){
        //get the buf
        p_buf_ = p_buf;
      }else{
        //do not get the buf
        continue;
      }
    }
    *p_buf++ = msg_data;
    
    //检查接收到帧格式
    switch(check_lora_data2frame(p_buf_,p_buf)){
    case 2:
      //当前帧错误
      p_buf = p_buf_;
      break;
    case 0:
      //当前帧没有接收完
      break;
    case 1:
      //当前帧接收完毕
      if(*(p_buf_) == 0x68){
        //抄表状态下 采集器返回过来的数据帧  或者 应答帧
        if(get_readding()){
          forme = 0;
//          switch(get_protocol()){  //TODO...
//          case 0x11:
//            //EG
//            if(cjqaddr_eg[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr_eg[1] == *(p_buf_ + DATA_POSITION)){
//              forme = 1;
//            }
//            break;
//          case 0xFF:
//            //188
//            if(cjqaddr[0] == *(p_buf_ + DATA_POSITION + 1) && cjqaddr[1] == *(p_buf_ + DATA_POSITION)){
//              forme = 1;
//            }
//            break;
//          }
          if(forme){
            if(*(p_buf_+AFN_POSITION) == AFN_ACK){
              //ACK
              signal_cjqack();
              p_buf = p_buf_;
            }else{
              //DATA
              //当前采集器   ACK   DATA to Queue  TODO...
              device_ack_lora(0,*(p_buf_ + SEQ_POSITION));
              frame_len = check_frame(p_buf_);
              OSQPost(&Q_CJQ,
                      p_buf_,
                      frame_len,
                      OS_OPT_POST_FIFO,
                      &err);
              if(err == OS_ERR_NONE){
                p_buf_ = 0;
                p_buf = 0;
              }else{
                p_buf = p_buf_;
              }
            }
          }else{
            p_buf = p_buf_;  //不是找我的
          }
        }else{
          p_buf = p_buf_; //我不在抄表模式下  
        }
      }
      if(*(p_buf_) == 0x0D){
        //+++/AT+ESC return
        signal_lora_ok();
      }
      break;
    }
  }
}




/**
 * 连接服务器 处理GPRS模块发送过来的指令
 */
void task_server(void *p_arg){
  OS_ERR err;
  
  uint8_t * p_buf = 0;
  uint8_t * p_buf_ = 0;
  uint8_t getbuffail = 0;
  uint8_t connectfail = 0;
  
  uint8_t *frame_start = 0; //接收到的帧的起始地址
  uint8_t frame_len = 0;  //接收到帧的总长度
  uint8_t server_seq_ = 0; //服务器发送过来的数据 的序列号
  
  uint8_t * p_buf_copy = 0;  //复制配置帧 抄表帧 post 到队列
  
  while(DEF_TRUE){
    if(get_connect_state()){
      if(p_buf == 0){
        p_buf = get_membuf();
        if(p_buf > 0){
          //get the buf
          getbuffail = 0;
          p_buf_ = p_buf;
          server_2buf(p_buf);
        }else{
          //do not get the buf
          getbuffail ++;
          //if(getbuffail >= 20){
          //  //20次没有获取到buf  
          //  device_cmd(0);
          //  *((uint8_t *)0) = 0x00;  //迫使系统重启
          //}
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
          //oh it's the data  TODO...  \r\n+TCPRECV:0,**,0x68 L L 0x68 C A     \r\n
          frame_start = Str_Str(p_buf_,"\r\n\x68") + 2;
          frame_len = check_frame(frame_start);  //check the frame get the length
          if(frame_len){
            //the frame is ok
            server_seq_ = *(frame_start + SEQ_POSITION) & 0x0F;  //获得该帧的序列号
            switch(*(frame_start+AFN_POSITION)){
            case AFN_ACK:  //the ack of the server
              if(server_seq_ == data_seq){
                signal_serverack();
              }
              break;
            case AFN_CONFIG:
            case AFN_QUERY:
              p_buf_copy =  get_membuf();
              if(p_buf_copy > 0){
                Mem_Copy(p_buf_copy,frame_start,frame_len);
                *(p_buf_copy + frame_len) = 0x01;  //标识这一帧来自服务器
                OSQPost(&Q_CONFIG,p_buf_copy,frame_len,OS_OPT_POST_1,&err);
                if(err != OS_ERR_NONE){
                  put_membuf(p_buf_copy);
                }
              }
              break;
            case AFN_CONTROL:
            case AFN_CURRENT:
              device_ack(0x01,server_seq_);  //ACK
              if(*(frame_start+FN_POSITION) == 0x05){//匹配序列号
                server_seq = server_seq_;
              }else{
                if(server_seq != server_seq_){  //新的抄表指令  read
                  p_buf_copy =  get_membuf();
                  if(p_buf_copy > 0){
                    Mem_Copy(p_buf_copy,frame_start,frame_len);
                    *(p_buf_copy + frame_len) = 0x01;  //标识这一帧来自服务器
                    server_seq = server_seq_;
                    OSQPost(&Q_READ,p_buf_copy,frame_len,OS_OPT_POST_1,&err);
                    if(err != OS_ERR_NONE){
                      put_membuf(p_buf_copy);
                    }
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
          p_buf =p_buf_;
          Mem_Set(p_buf_,0x00,256); //clear the buf
          server_2buf(p_buf_);
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
        //if(connectfail > 20){
        //  device_cmd(0);
        //  *((uint8_t *)0) = 0x00;  //迫使系统重启
        //}
      }
    }
  }
}

/**
 * GPRS 心跳任务
 */
void task_heartbeat(void *p_arg){
  uint8_t beat[17];
  uint8_t * p_beat;
  uint8_t heart_ack = 0;
  uint8_t i;
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
          
          data_seq = addSEQ();
          *p_beat++ = ZERO_BYTE |SINGLE | CONFIRM | data_seq;
          
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
  uint8_t i;
  
  while(DEF_TRUE){
    //2min  每2分钟检测一次
    delayms(120000);
    if(get_readding()){
      continue;
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
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * p_buf;
  
  while(DEF_TRUE){
    p_buf = OSQPend(&Q_READ,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    &msg_size,
                    &ts,
                    &err);
    set_readding(1);
    switch(*(p_buf+AFN_POSITION)){
    case AFN_CONTROL:
      meter_control(p_buf,msg_size);
      break;
    case AFN_CURRENT:
      meter_read(p_buf,msg_size);
      break;
    }
    set_readding(0);
    
    put_membuf(p_buf);
  }
}

/**
 * config and query the parameter
 */
void task_config(void *p_arg){
  OS_ERR err;
  CPU_TS ts;
  uint16_t msg_size;
  uint8_t * p_buf;
  
  while(DEF_TRUE){
    p_buf = OSQPend(&Q_CONFIG,
                    0,
                    OS_OPT_PEND_BLOCKING,
                    &msg_size,
                    &ts,
                    &err);
    
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
