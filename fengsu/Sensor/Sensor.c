#include "Sensor.h"
#include <string.h>
#include "config_flash.h"
/**
 * @brief 初始化风速传感器
 */
void WindSensor_Init(WindSensor_t *sensor, RS485_Handle_t *rs485, uint8_t slave_addr) {
    sensor->rs485 = rs485;
    sensor->slave_address = slave_addr;
    sensor->wind_speed = 0.0f;
	  sensor->sensor_open=0x01; //默认开启
	 sensor->last_read_time = 0;
    sensor->read_interval = 1000; // 默认1秒读取一次
}

/**
 * @brief 读取风速值
 */
HAL_StatusTypeDef WindSensor_ReadSpeed(WindSensor_t *sensor) {
    if (sensor->rs485 == NULL) {
        return HAL_ERROR;
    }
    
     uint8_t tx_buffer[8];
     uint8_t rx_buffer[7]; // 正常响应: 地址1 + 功能码1 + 长度1 + 数据2 + CRC2 = 7字节
    sensor->sensor_open=0x01;
    // 构建Modbus读取命令: 读取保持寄存器 0x0000 (风速值)
    tx_buffer[0] = sensor->slave_address;  // 设备地址
    tx_buffer[1] = MODBUS_READ_HOLDING_REG; // 功能码
    tx_buffer[2] = (WIND_SPEED_REG >> 8) & 0xFF; // 寄存器地址高字节
    tx_buffer[3] = WIND_SPEED_REG & 0xFF;       // 寄存器地址低字节
    tx_buffer[4] = 0x00; // 寄存器数量高字节
    tx_buffer[5] = 0x01; // 寄存器数量低字节(读取1个寄存器)
    
    // 计算CRC
    uint16_t crc = RS485_Frame_CalculateCRC16(tx_buffer, 6);
    tx_buffer[6] = crc & 0xFF;        // CRC低字节
    tx_buffer[7] = (crc >> 8) & 0xFF; // CRC高字节
    
    // 发送命令并接收响应
    HAL_StatusTypeDef status = RS485_TransmitReceive(sensor->rs485, tx_buffer, sizeof(tx_buffer), rx_buffer, sizeof(rx_buffer));
   
    // 解析风速值 (寄存器值需要除以100得到实际值)
    uint16_t raw_value = (rx_buffer[3] << 8) | rx_buffer[4];
    sensor->wind_speed = (float)raw_value / 100.0f;
    return HAL_OK;
}

/**
 * @brief 更新风速传感器数据
 */
void WindSensor_Update(WindSensor_t *sensor) {
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - sensor->last_read_time >= config_page1.sample_period*100) {
        HAL_StatusTypeDef status = WindSensor_ReadSpeed(sensor);
        
        if (status == HAL_OK) {
          //  printf("风速: %.2f m/s\n", sensor->wind_speed);
        } else {
           // printf("读取风速失败, 状态: %d, 重试次数: %d\n", status, sensor->retry_count);
        }
        
        sensor->last_read_time = current_time;
    }
}

/**
 * @brief 获取当前风速值
 */
float WindSensor_GetSpeed(WindSensor_t *sensor) {
    return sensor->wind_speed;
}
/**
 * @brief 发送和接受风速传感器指令
 *//**
 * @brief Modbus RTU发送接收（针对风速传感器优化）
 * @param hrs485: RS485句柄
 * @param tx_data: 发送数据
 * @param tx_size: 发送数据长度
 * @param rx_data: 接收缓冲区
 * @param rx_size: 接收缓冲区大小
 * @return HAL状态
 */
HAL_StatusTypeDef RS485_TransmitReceive(RS485_Handle_t *hrs485, uint8_t *tx_data, 
                                        uint8_t tx_size, uint8_t *rx_data, uint8_t rx_size) {
    HAL_StatusTypeDef status;
//    if (hrs485->huart == NULL || tx_data == NULL || tx_size == 0 || rx_data == NULL || rx_size == 0) {
//        return HAL_ERROR;
//    }
        HAL_GPIO_WritePin(hrs485->de_port, hrs485->de_pin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 50; i++); // 短暂延时
    
    // 发送数据
    status = HAL_UART_Transmit(hrs485->huart, tx_data,tx_size, HAL_MAX_DELAY);
    
    // 等待发送完成
    HAL_Delay(10);
     
    // 切换回接收模式
    HAL_GPIO_WritePin(hrs485->de_port, hrs485->de_pin, GPIO_PIN_RESET);
   //for(volatile int i = 0; i < 50; i++); // 短暂延时
		  HAL_Delay(10);														
    // 3. 接收数据
     HAL_UART_Receive(hrs485->huart, rx_data, rx_size, 1000);
		return HAL_OK;
}

