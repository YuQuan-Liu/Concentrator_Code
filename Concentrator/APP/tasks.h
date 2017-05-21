
#ifndef TASK_H
#define TASK_H


//tasks
void Task_485_2(void *p_arg);
void Task_LORA(void *p_arg);
void Task_Server(void *p_arg);
void Task_DealServer(void *p_arg);
void Task_HeartBeat(void *p_arg);
void Task_LORA_Check(void *p_arg);
void Task_Read(void *p_arg);
void Task_Config(void *p_arg);


void Task_LED(void *p_arg);

#endif