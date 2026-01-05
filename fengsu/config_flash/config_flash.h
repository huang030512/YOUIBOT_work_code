#ifndef CONFIG_FLASH_H
#define CONFIG_FLASH_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// Flash配置定义
#define CONFIG_FLASH_PAGE_ADDR    0x08007C00  // 最后一页地址
#define CONFIG_MAGIC_NUMBER       0x55AA55AA  // 配置标识符

// 配置参数结构体（需要4字节对齐）
typedef struct {
    uint32_t magic_number;         // 配置标识符
    uint8_t device_address;        // 设备地址 (参数0)
    uint8_t sensor_enable;         // 是否启用风速传感器 (参数1)
    uint8_t sample_period;         // 风速传感器采样周期 (参数2)
    uint8_t reserved[57];          // 保留字节，使结构体为64字节（Flash半页大小）
} Config_Page1_t;

// 全局配置变量声明（在config_flash.c中定义）
extern Config_Page1_t config_page1;

// Flash操作函数声明
void Flash_Unlock(void);
void Flash_Lock(void);
HAL_StatusTypeDef Flash_ErasePage(uint32_t page_address);
HAL_StatusTypeDef Flash_WriteWord(uint32_t address, uint32_t data);

// 配置管理函数
void Config_Init(void);
uint8_t Config_ReadPage1(Config_Page1_t *config);
uint8_t Config_WritePage1(Config_Page1_t *config);
void Config_ReadFromFlash(void);
void Config_WriteToFlash(void);
uint8_t Config_IsValid(void);

#endif /* CONFIG_FLASH_H */