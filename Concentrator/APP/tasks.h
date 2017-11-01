
#ifndef TASK_H
#define TASK_H


//tasks

void task_meter_raw(void *p_arg);
void task_server(void *p_arg);
void task_server_action(void *p_arg);
void task_lora_raw(void *p_arg);
void task_cjq_raw(void *p_arg);
void task_read(void *p_arg);
void task_heartbeat(void *p_arg);
void task_lora_check(void *p_arg);
void task_config(void *p_arg);
void task_syn(void *p_arg);
void task_led(void *p_arg);

void task_overload(void *p_arg);

#endif