#ifndef RS485_H
#define RS485_H

#include "main.h"
#include <stdbool.h>
#include <string.h>

// 数据帧格式定义
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2];        // 帧头 0x55 0xAA
    uint8_t device_id;        // 设备地址
    uint8_t ID;              // 会话ID
    uint8_t length;           // 数据长度
	  uint8_t command ; 
    uint8_t data[248];        // 数据域
    uint16_t crc16;           // Modbus CRC-16（2字节）
} RS485_Frame_t;
#pragma pack(pop)

// 命令定义
typedef enum {
    CMD_INFORMATION = 0x01,          // 获取信息
		CMD_CONTROL = 0x02,       // 风机控制命令
    CMD_READ_DATA = 0x03,     // 获取配置
    CMD_WRITE_DATA = 0x04,    // 写入配置
} Command_t;
typedef enum {
    Response_INFORMATION = 0x21,          // 获取信息
		Response_CONTROL = 0x22,       // 风机控制命令
    Response_READ_DATA = 0x23,     // 获取配置
    Response_WRITE_DATA = 0x24,    // 写入配置
} Response_Command_t;
// RS485状态机状态
typedef enum {
    STATE_IDLE = 0,           // 空闲状态
    STATE_HEADER_1,           // 接收帧头1
    STATE_HEADER_2,           // 接收帧头2
    STATE_DEVICE_ID,          // 接收板地址
    STATE_ID,            			// 接收会话ID
    STATE_LENGTH,             // 接收数据长度
		STATE_COMMAND,						//命令格式
    STATE_DATA,               // 接收数据
    STATE_CRC_LOW,            // 接收CRC低字节
    STATE_CRC_HIGH            // 接收CRC高字节
} RS485_State_t;

// RS485句柄结构
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *de_port;
    uint16_t de_pin;
    uint8_t device_id;        // 板地址
    
    // 接收相关
    RS485_State_t state;
    RS485_Frame_t rx_frame;
    uint16_t rx_data_index;
    uint8_t rx_byte;
    bool frame_received;
    bool frame_error;
    
    // 发送相关
    RS485_Frame_t tx_frame;
    
    // 统计
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t error_count;
} RS485_Handle_t;

// 函数声明
void RS485_Init(RS485_Handle_t *hrs485, UART_HandleTypeDef *huart, 
                GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t device_id);
void RS485_StartReceive(RS485_Handle_t *hrs485);
void RS485_UART_RxCpltCallback(RS485_Handle_t *hrs485);
void RS485_ProcessReceivedFrame(RS485_Handle_t *hrs485);
void RS485_SendResponse(RS485_Handle_t *hrs485, uint8_t command, uint8_t *data, uint8_t length,uint8_t ID);


// 数据帧处理函数
void RS485_Frame_Init(RS485_Frame_t *frame, uint8_t device_id, uint8_t command,uint8_t ID);
uint8_t RS485_Frame_CalculateChecksum(RS485_Frame_t *frame);
bool RS485_Frame_ValidateChecksum(RS485_Frame_t *frame);
uint16_t RS485_Frame_Serialize(RS485_Frame_t *frame, uint8_t *buffer);
HAL_StatusTypeDef RS485_SendFrame(RS485_Handle_t *hrs485, RS485_Frame_t *frame);
uint16_t RS485_Frame_CalculateCRC16(uint8_t *data, uint16_t length);
bool RS485_Frame_ValidateCRC16(RS485_Frame_t *frame);
// 命令处理回调函数类型
typedef void (*CommandHandler_t)(RS485_Handle_t *hrs485, RS485_Frame_t *frame);

// 命令处理函数
void RS485_RegisterCommandHandler(RS485_Handle_t *hrs485, uint8_t command, CommandHandler_t handler);





#endif