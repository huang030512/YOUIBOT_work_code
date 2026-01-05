#ifndef SENSOR_H
#define SENSOR_H

#include "main.h"
#include "rs485.h"

// 风速传感器结构体
typedef struct {
    RS485_Handle_t *rs485;
    uint8_t slave_address;
    float wind_speed;           // 当前风速值 (m/s)
	 uint32_t last_read_time;
    uint32_t read_interval;     // 读取间隔(ms)
    bool sensor_open; //是否开启风速传感器

} WindSensor_t;

// 函数声明
void WindSensor_Init(WindSensor_t *sensor, RS485_Handle_t *rs485, uint8_t slave_addr);
HAL_StatusTypeDef WindSensor_ReadSpeed(WindSensor_t *sensor);
void WindSensor_Update(WindSensor_t *sensor);
float WindSensor_GetSpeed(WindSensor_t *sensor);
HAL_StatusTypeDef RS485_TransmitReceive(RS485_Handle_t *hrs485, uint8_t *tx_data, 
                                        uint8_t tx_size, uint8_t *rx_data, uint8_t rx_size);
// Modbus功能码
#define MODBUS_READ_HOLDING_REG     0x03
#define MODBUS_WRITE_SINGLE_REG     0x06

// 传感器寄存器地址
#define WIND_SPEED_REG              0x0000  //风速寄存器
#define Count_REG                  0x0065   //测点总数

#define DEVICE_ADDR_REG             0x0066  //设备地址


#endif