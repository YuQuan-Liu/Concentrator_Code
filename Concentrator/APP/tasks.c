

#include "os.h"
#include "stm32f10x_conf.h"
#include "tasks.h"
#include "lib_str.h"
#include "serial.h"
#include "spi_flash.h"
#include "frame.h"
#include "frame_188.h"
#include "utils.h"
#include "configs.h"
#include "bsp.h"
#include "device_params.h"
#include "readmeter.h"


/**
 * 处理meter usart接收到的数据
 */
void task_meter_raw(void *p_arg){
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  uint8_t post_q_result = 0;
  
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
        switch(get_protocol()){
        case 0xFF: //188
          frame_len = *(p_buf_+10)+13;
          post_q_result = post_q_meter(p_buf_,frame_len);
        case 0xEE: //188 bad
          frame_len = *(p_buf_+10)+13;
          post_q_result = post_q_meter(p_buf_,frame_len);
          break;
        }
        
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
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  uint8_t server_seq_;
  
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
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
      case 1:
      case 2:
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
          if(server_seq_ == get_data_seq()){
            signal_jzqack();
          }
          p_buf = p_buf_;
          break;
        default:
          p_buf = p_buf_;
          break;
        }
        break;
      default :
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
  uint8_t * p_mem;    //the ptr get from the queue
  uint16_t msg_size;    //the message's size 
  uint8_t msg_data;         //the data get from the queue
  
  uint8_t server_seq_;
  uint16_t frame_len = 0;
  
  uint8_t * p_buf = 0;   //the buf used put the data in 
  uint8_t * p_buf_;       //keep the buf's ptr  used to release the buf
  
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
        //抄表状态下 采集器返回过来的数据帧  或者 应答帧
        frame_len = check_frame(p_buf_);
        server_seq_ = *(p_buf_+SEQ_POSITION) & 0x0F;  //获得该帧的序列号
        *p_buf = 0x01;//标识这一帧数据是来自LORA
        switch(cjq_data_tome(p_buf_,frame_len)){
        case 1:
        case 2:
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
            device_ack(1,server_seq_,(uint8_t *)0,0,AFN_ACK,FN_ACK);  //ACK
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
            if(server_seq_ == get_data_seq()){
              signal_jzqack();
            }
            p_buf = p_buf_;
            break;
          default:
            p_buf = p_buf_;
            break;
          }
          break;
        default :
          p_buf = p_buf_;
          break;
        }
        
      }
      if(*(p_buf_) == 0x0D){  //+++/AT+ESC return
        signal_lora_ok();
        p_buf = p_buf_;
      }
      break;
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
  uint16_t msg_size;
  uint8_t * p_buf;
  
  while(DEF_TRUE){
    wait_q_read(&p_buf,&msg_size,0);
    
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
  uint16_t msg_size;
  uint8_t * p_buf;
  
  while(DEF_TRUE){
    wait_q_conf(&p_buf,&msg_size,0);
    
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
