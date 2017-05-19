
#ifndef TASK_H
#define TASK_H






void meter_read_188(uint8_t * buf_frame,uint8_t desc);  //抄表
void meter_read_eg(uint8_t * buf_frame,uint8_t desc);  //抄表
void meter_control(uint8_t * buf_frame,uint8_t desc);   //开关阀
void meter_read_single(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc);  //只管读表
void meter_send(uint8_t all,uint32_t block_meter_,uint8_t desc);  //all = 1 发送全部表  all = 0 发送表块对应的表
void meter_open(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_close(uint8_t * meter_addr,uint32_t block_meter,uint8_t meter_type,uint8_t desc,uint8_t server_seq_);
void meter_clean(void); //执行一次开关阀操作

void device_ack(uint8_t desc,uint8_t server_seq_);  //发送确认帧  1 发送给M590E服务器  0 发送给485

/*
如果底层有采集器  底层采集器使用透传模式
集中器直接发送抄表指令
cjq_open   cjq_close  只对4路MBUS继电器进行控制
*/
uint8_t cjq_open(uint8_t * cjq_addr,uint32_t block_cjq);
uint8_t cjq_close(uint8_t * cjq_addr,uint32_t block_cjq);

void cjq_timeout(void *p_tmr,void *p_arg);   //20min 超时 关闭通道

uint8_t relay_485(FunctionalState NewState);
uint8_t mbus_power(FunctionalState NewState);


//tasks
void Task_485_2(void *p_arg);
void Task_LORA(void *p_arg);
void Task_Server(void *p_arg);
void Task_DealServer(void *p_arg);
void Task_Connect(void *p_arg);
void Task_Read(void *p_arg);
void Task_HeartBeat(void *p_arg);
void Task_Config(void *p_arg);
void Task_LED(void *p_arg);

void addSEQ(void);  //增加local_seq



#endif