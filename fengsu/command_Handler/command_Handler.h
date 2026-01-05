#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "rs485.h"
#include "config_flash.h"  // 包含Flash配置头文件

// 注意：Config_Page1_t结构体已移到config_flash.h中
// 全局配置变量config_page1已在config_flash.h中声明为extern

// 函数声明
void CommandHandler_INFORMATION(RS485_Handle_t *hrs485, RS485_Frame_t *frame);
void CommandHandler_ReadData(RS485_Handle_t *hrs485, RS485_Frame_t *frame);
void CommandHandler_WriteData(RS485_Handle_t *hrs485, RS485_Frame_t *frame);
void CommandHandler_Control(RS485_Handle_t *hrs485, RS485_Frame_t *frame);
void RegisterAllCommandHandlers(RS485_Handle_t *hrs485);

#endif