#include "config_flash.h"
#include "command_Handler.h"
#include <string.h>

// 全局配置变量定义
Config_Page1_t config_page1;

/**
 * @brief 解锁Flash
 */
void Flash_Unlock(void) {
    HAL_FLASH_Unlock();
}

/**
 * @brief 锁定Flash
 */
void Flash_Lock(void) {
    HAL_FLASH_Lock();
}

/**
 * @brief 擦除Flash页
 * @param page_address 页地址
 */
HAL_StatusTypeDef Flash_ErasePage(uint32_t page_address) {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = page_address;
    erase_init.NbPages = 1;  // 只擦除一页
    
    return HAL_FLASHEx_Erase(&erase_init, &page_error);
}

/**
 * @brief 写入Flash字（32位）
 * @param address 地址
 * @param data 数据
 */
HAL_StatusTypeDef Flash_WriteWord(uint32_t address, uint32_t data) {
    return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data);
}

/**
 * @brief 检查配置是否有效
 */
uint8_t Config_IsValid(void) {
    return (config_page1.magic_number == CONFIG_MAGIC_NUMBER);
}

/**
 * @brief 从Flash读取配置
 */
void Config_ReadFromFlash(void) {
    Config_Page1_t *flash_config = (Config_Page1_t*)CONFIG_FLASH_PAGE_ADDR;
    
    // 复制Flash中的数据到RAM
    memcpy(&config_page1, flash_config, sizeof(Config_Page1_t));
    
    // 如果配置无效，使用默认值
    if (!Config_IsValid()) {
        Config_Init();
    }
}

/**
 * @brief 将配置写入Flash
 */
void Config_WriteToFlash(void) {
    uint32_t *src_data = (uint32_t*)&config_page1;
    uint32_t flash_address = CONFIG_FLASH_PAGE_ADDR;
    uint32_t word_count = sizeof(Config_Page1_t) / 4;  // 计算32位字数量
    
    // 解锁Flash
    Flash_Unlock();
    
    // 擦除最后一页（必须擦除后才能写入）
    if (Flash_ErasePage(CONFIG_FLASH_PAGE_ADDR) != HAL_OK) {
        Flash_Lock();
        return;
    }
    
    // 写入配置数据（按32位字写入）
    for (uint32_t i = 0; i < word_count; i++) {
        if (Flash_WriteWord(flash_address, src_data[i]) != HAL_OK) {
            break;
        }
        flash_address += 4;  // 地址增加4字节
    }
    
    // 锁定Flash
    Flash_Lock();
}

/**
 * @brief 初始化配置参数
 */
void Config_Init(void) {
    // 设置标识符
    config_page1.magic_number = CONFIG_MAGIC_NUMBER;
    
    // 默认配置值
    config_page1.device_address = 0x01;       // 默认设备地址1
    config_page1.sensor_enable = 0x01;        // 默认启用风速传感器
    config_page1.sample_period = 0x0A;        // 默认采样周期10（1秒）单位为100毫秒
    
    // 清除保留区域
    memset(config_page1.reserved, 0, sizeof(config_page1.reserved));
    
    // 写入Flash
    Config_WriteToFlash();
}

/**
 * @brief 读取第一页配置
 */
uint8_t Config_ReadPage1(Config_Page1_t *config) {
    if (config == NULL) return 0x01;
    
    // 从RAM读取配置
    config->device_address = config_page1.device_address;
    config->sensor_enable = config_page1.sensor_enable;
    config->sample_period = config_page1.sample_period;
    
    return 0x00; // 成功
}

/**
 * @brief 写入第一页配置
 */
uint8_t Config_WritePage1(Config_Page1_t *config) {
    if (config == NULL) return 0x01;
    
    // 参数有效性检查
    if (config->device_address < 1 || config->device_address > 255) {
        return 0x01; // 参数无效
    }
    
    if (config->sensor_enable > 0x01) {
        return 0x01; // 参数无效
    }
    
    if (config->sample_period < 1 || config->sample_period > 100) {
        return 0x01; // 参数无效
    }
    
    // 更新RAM中的配置
    config_page1.magic_number = CONFIG_MAGIC_NUMBER;
    config_page1.device_address = config->device_address;
    config_page1.sensor_enable = config->sensor_enable;
    config_page1.sample_period = config->sample_period;
    
    // 写入Flash
    Config_WriteToFlash();
    
    return 0x00; // 成功
}