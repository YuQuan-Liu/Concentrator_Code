
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
  case 0xFF://�ײ��豸 ��  TODO...
//    switch(*(p_frame+DATA_POSITION)){  //������  �����ɼ���(ͨ��)  ȫ����(ȫ��ͨ��)
//    case 0xFF: //ȫ����
//      meter_read_m_all(p_frame,frame_len);
//      break;
//    case 0xAA: //�����ɼ���ͨ��
//      meter_read_m_channel(p_frame,frame_len);
//      break;
//    case 0x11: //������
//      meter_read_m_meter(p_frame,frame_len);
//      break;
//    }
    break;
  case 0xBB://�ײ��豸CJQ
    switch(*(p_frame+DATA_POSITION)){  //������  �����ɼ���(ͨ��)  ȫ����(ȫ��ͨ��)
    case 0xFF: //ȫ����
      meter_read_c_all(p_frame,frame_len);
      break;
    case 0xAA: //�����ɼ���ͨ��
      meter_read_c_channel(p_frame,frame_len);
      break;
    case 0x11: //������
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
  uint16_t cjq_count = 0;  //FLASH һ���ж��ٲɼ���
  uint32_t block_cjq = 0;
  uint32_t block_meter = 0;
  uint16_t c = 0;
  uint8_t * p_all_cjq = 0;

  uint8_t cjq_addr[5];
  uint8_t cjq_count_inbuf = 0;  //MEMBUF��һ���ж��ٸ��ɼ���
  
  uint8_t frame_data[1];
  uint8_t cjq_seq;
  uint8_t ack = 0;
  uint8_t i = 0;
  
  uint8_t timeout_count = 30;
  uint8_t read_over_cjq = 0;   //������ɵĲɼ���(������ʱ)
  uint8_t all_read_over = 0;  //���еĲɼ����Ƿ񶼳��������
  uint8_t cjq_state = 0;   //��ǰ�ɼ����ĳ���״̬
  uint8_t read_over = 2;  //0~������  1~�������  2~��ʱ  ��ѯ�ɼ����Ƿ��ڳ��� ��Ӧ��״̬
  
  uint8_t * p_response;
  uint16_t msg_size;
  
  uint16_t all_frames;
  uint16_t this_frame;
  uint8_t frame_metercount;
  
  
  uint8_t * p_meteraddr = 0;
  uint8_t * p_meter_read;
  uint8_t * p_meter_status;
  
  p_all_cjq = get_membuf();
  if(p_all_cjq > 0){
    /*
    p_all_cjq   256�洢һ���ж��ٸ��ɼ���  255�洢�Ѿ������˼����ɼ���
    ÿ���ɼ���ռ10���ֽ�  12345~�ɼ�����ַ   6~�Ƿ����~[00~δ���  11~���  22~��ʱ]
    7~�Ƿ���յ��ɼ�������  00~û�н��յ�   11~���յ�
    */
    Mem_Set(p_all_cjq,0x00,256);

    sFLASH_ReadBuffer((uint8_t *)&cjq_count,sFLASH_CJQ_COUNT,2);  //�ɼ�������
    sFLASH_ReadBuffer((uint8_t *)&block_cjq,sFLASH_CJQ_Q_START,3);  //�ɼ�������ͷ

    for(c = 0;c < cjq_count;c++){
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);

      if(!search_cjq_in_membuf(p_all_cjq,cjq_addr)){
        cjq_count_inbuf = p_all_cjq[255];
        Mem_Copy(p_all_cjq + cjq_count_inbuf*10 + 1, cjq_addr + 1,4);
        p_all_cjq[255] = cjq_count_inbuf + 1;
      }

      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }

    //�������¹ҵĲɼ����豸�ĸ���
    cjq_count_inbuf = p_all_cjq[255];

    //���θ��ɼ������ͳ���ָ��
    for(c = 0; c < cjq_count_inbuf;c++){
      frame_data[0] = 0xFF;  //���ɼ���ȫ����
      sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);

      ack = 0;
      for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
        if(write_frame_cjq(cjq_addr, frame_data, 1,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
          if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
            ack = 1;
            break;
          }
        }
      }
      if(ack){
        p_all_cjq[10*c+5] = 0x00;
      }else{
        p_all_cjq[10*c+5] = 0x22;
      }
      sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
    }

    //ÿ10s����һ�����еĲɼ���  ���Ƿ񳭱����  �ȴ�5min
    timeout_count = 30;
    read_over_cjq = 0;   //������ɵĲɼ���(������ʱ)
    all_read_over = 0;  //���еĲɼ����Ƿ񶼳��������
    while(timeout_count > 0){
      timeout_count++;
      delayms(10000);

      for(c = 0; c < cjq_count_inbuf;c++){
        sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x00){   //�жϲɼ���״̬  ���ڳ���...
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          read_over = 2;  //0~������  1~�������  2~��ʱ
          for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
            if(write_frame_cjq(cjq_addr, (uint8_t *)0, 0,AFN_QUERY,FN_READING,cjq_seq)){ //���ɼ�������ѯ���Ƿ��ڳ���
              if(wait_q_cjq(&p_response,&msg_size,10000)){
                //�жϲɼ����Ƿ񳭱����
                if(*(p_response + DATA_POSITION)){  //���ڳ���
                  read_over = 0;
                }else{  //�������
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
        sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
      }

      if(read_over_cjq == cjq_count_inbuf){
        all_read_over = 1;
        break;
      }
    }

    if(all_read_over){//ȫ���������
      //���α������г�����ɵĲɼ���  ��ȡ���µ�����
      for(c = 0; c < cjq_count_inbuf;c++){
        sFLASH_ReadBuffer((uint8_t *)&cjq_addr,block_cjq+CJQ_FLASH_INDEX_ADDR,5);
        cjq_state = p_all_cjq[10*c+5];
        if(cjq_state == 0x11){   //�жϲɼ���״̬Ϊ�������
          cjq_seq = add_cjq_seq();
          set_cjq_data_seq(cjq_seq);

          for(i = 0;i < 3;i++){  //���ɼ�����ָ�� ����3��
            if(write_frame_cjq(cjq_addr, (uint8_t *)0, 0,AFN_QUERY,FN_ALL_READDATA,cjq_seq)){ //���ɼ������Ͱ�ȫ����ĳ�����������
              timeout_count = 10;
              while(timeout_count > 0){
                if(wait_q_cjq(&p_response,&msg_size,10000)){
                  //֡�ĳ���msg_size  �����֡��һ���ж��ٱ� frame_metercount
                  //һ���ж���֡all_frames  ���ǵڼ�֡this_frame
                  all_frames = *(p_response + DATA_POSITION) + *(p_response +DATA_POSITION+1)<<8;
                  this_frame = *(p_response + DATA_POSITION+2) + *(p_response + DATA_POSITION+3)<<8;
                  frame_metercount = (msg_size-8-9-5)/14;

                  for(i = 0;i < frame_metercount;i++){
                    //��֡�л�ȡ  ���ַ ��Ķ���  ���״̬
                    p_meteraddr = p_response + DATA_POSITION + 5 + 14 * i;
                    p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * i;
                    p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * i;

                    block_meter = search_meter(block_cjq,p_meteraddr);
                    if(block_meter){  //��������
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

              if(timeout_count > 0){ //�������е�֡OK
                p_all_cjq[10*c+6] = 0x11;
                break;
              }
            }
          }
        }
        sFLASH_ReadBuffer((uint8_t *)&block_cjq,block_cjq+FLASH_POOL_NEXT_INDEX,3);
      }

      //�������µĲɼ���������  ȫ�����յ��˼�������
      //TODO... �����ݷ��ͳ�ȥ


    }else{  //����ʱ  ����Ŀ����Բ���  ���Ҳ���
      
    }
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

  uint8_t timeout_count = 10; //�������10�γ�ʱ���ж�Ϊʧ��
  uint16_t all_frames = 10;  //��ʾһ���᷵�ض���֡
  uint16_t this_frame = 0;   //��ʾ��������֡�еĵڼ�֡
  uint8_t frame_metercount = 0;  //���յ���֡���м���������



  p_cjqaddr = p_frame+DATA_POSITION+1;
  block_cjq = search_cjq(p_cjqaddr);

  if(block_cjq){  //find the cjq
    //���˳���ָ��͸��ɼ���
    frame_data[0] = *(p_frame+DATA_POSITION);

    cjq_seq = add_cjq_seq();
    set_cjq_data_seq(cjq_seq);
    if(write_frame_cjq(p_cjqaddr, frame_data, 1,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
      if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
        while(timeout_count > 0){
          if(wait_q_cjq(&p_response,&msg_size,10000)){
            //֡�ĳ���msg_size  �����֡��һ���ж��ٱ� frame_metercount
            //һ���ж���֡all_frames  ���ǵڼ�֡this_frame
            all_frames = *(p_response + DATA_POSITION) + *(p_response +DATA_POSITION+1)<<8;
            this_frame = *(p_response + DATA_POSITION+2) + *(p_response + DATA_POSITION+3)<<8;
            frame_metercount = (msg_size-8-9-5)/14;

            for(i = 0;i < frame_metercount;i++){
              //��֡�л�ȡ  ���ַ ��Ķ���  ���״̬
              p_meteraddr = p_response + DATA_POSITION + 5 + 14 * i;
              p_meter_read = p_response + DATA_POSITION + 5 + 8 + 14 * i;
              p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4 + 14 * i;

              block_meter = search_meter(block_cjq,p_meteraddr);
              if(block_meter){  //��������
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
        if(timeout_count > 0){ //�������е�֡OK

        }else{ //�������еı����ݳ�ʱ   ���Ҳ��� TODO...

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
      //���˳���ָ��͸��ɼ���
      frame_data[0] = *(p_frame+DATA_POSITION);
      for(i = 0;i < 7;i++){
        frame_data[i+1] = p_meteraddr[i];
      }

      cjq_seq = add_cjq_seq();
      set_cjq_data_seq(cjq_seq);
      if(write_frame_cjq(p_cjqaddr, frame_data, 8,AFN_CURRENT,FN_METER,cjq_seq)){ //���ɼ������ͳ���ָ��
        if(wait_cjqack(10000)){  //�ȴ�10s�ɼ����Գ���ָ��ACK
          if(wait_q_cjq(&p_response,&msg_size,10000)){
            //��֡�л�ȡ ��Ķ���  ���״̬
            p_meter_read = p_response + DATA_POSITION + 5 + 8;
            p_meter_status = p_response + DATA_POSITION + 5 + 8 + 4;
            //��������
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
    sFLASH_ReadBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);  //��ȡ����Sector
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_METERSTATE,meter_status,2);  //��¼st��Ϣ
    Mem_Copy(mem4k+block_meter%0x1000 + METER_FLASH_INDEX_READ,meter_read,4);        //����

    sFLASH_EraseSector((block_meter/0x1000)*0x1000);  //�����úõ�Flash������д�뵽Flash�С�
    sFLASH_WriteBuffer(mem4k,(block_meter/0x1000)*0x1000,0x1000);
    break;
  }
  return 1;
}
