#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"

#define BUFFER_SIZE 32 // 假设每次接收 128 字节
#define MAX_RETRY 3  // 允许的最大重试次数
#define TIMEOUT 100  // UART 超时时间（单位：ms）
extern uint8_t message_LPUART[BUFFER_SIZE];
extern uint8_t message_USART[BUFFER_SIZE];
extern uint8_t ch[50];
extern int if_serve;
extern float kpos;
extern float kspd;

void USART_printf(char *fmt, ...);
void process_received_data(uint8_t *data, uint32_t length);
uint16_t CRC16_CCITT(uint8_t *data, uint16_t length);
HAL_StatusTypeDef Reliable_Send(UART_HandleTypeDef *huart, uint8_t* data, uint8_t len);
typedef struct {
    float number; // 数字部分
    char  letter; // 字符部分
} UartMessage_t;

void Execute_Process_Task(UartMessage_t msgg);


typedef struct {
    uint8_t sourceID;
} TransferMessage_t;

typedef enum {
    TASK_SOURCE_UNKNOWN = 0,  // 未知任务
    TASK_SOURCE_DEVICE1,      // 设备1
    TASK_SOURCE_DEVICE2,      // 设备2
    TASK_SOURCE_DEVICE3,      // 设备3
} TaskSourceID;

extern TaskSourceID currentTask;

#endif
