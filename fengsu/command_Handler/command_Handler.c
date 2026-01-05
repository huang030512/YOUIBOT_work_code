#include "command_Handler.h"
#include "pwm_controller.h"
#include "main.h"
#include "Sensor.h"
extern PWM_Controller pwm1;
extern WindSensor_t  wind_sensor;
#define max_wind_speed 100
/**
 * @brief 获取信息命令处理
 */
void CommandHandler_INFORMATION(RS485_Handle_t *hrs485, RS485_Frame_t *frame) {
    // 返回控制速度、风机反馈速度、当前风速
    float duty_cycle = PWM_GetDutyCycle(&pwm1);
    float wind_speed = WindSensor_GetSpeed(&wind_sensor);
    
    uint8_t response_data[4];
    // 控制速度：0-100%，转换为1字节
    uint8_t control_speed = (uint8_t)(duty_cycle * 100.0f);
    // 风机反馈速度：0-100%，转换为1字节
    uint8_t fan_feedback = control_speed; // 这里假设反馈速度与控制速度相同
    
    // 当前风速：放大100倍，2字节（低字节在前）
    uint16_t wind_speed_int = (uint16_t)(wind_speed * 100.0f);
    
    response_data[0] = control_speed;      // 控制速度（1字节）
    response_data[1] = fan_feedback;       // 风机反馈速度（1字节）
    response_data[2] = wind_speed_int & 0xFF;        // 当前风速低字节
    response_data[3] = (wind_speed_int >> 8) & 0xFF; // 当前风速高字节
    
    RS485_SendResponse(hrs485, Response_INFORMATION, response_data, 4, hrs485->rx_frame.ID);
}
/**
 * @brief 控制命令处理
 */
void CommandHandler_Control(RS485_Handle_t *hrs485, RS485_Frame_t *frame) {
    uint8_t control_cmd = frame->data[0];
		float speed=(float)frame->data[1]/100.0f;
    uint8_t result = 0x00; // 默认成功
    switch (control_cmd) {
        case 0x00: // 停止风机
					PWM_Stop(&pwm1);
            // 执行重启逻辑
            break;
        case 0x01: // 启动风机
					PWM_Start(&pwm1);
				  PWM_SetDutyCycle(&pwm1,speed);
            // 执行参数复位
            break;
        default:
            result = 0x02; // 执行失败
            break;
    }
    if(speed>1.0){
			result=0x01;
		}
		if(PWM_GetDutyCycle(&pwm1)!=speed){
			result=0x03; //设备故障
		}
    // 发送控制结果
    RS485_SendResponse(hrs485, Response_CONTROL, &result, sizeof(result),hrs485->rx_frame.ID);
}


/**
 * @brief 读取数据命令处理
 */
void CommandHandler_ReadData(RS485_Handle_t *hrs485, RS485_Frame_t *frame) {
    uint8_t page = frame->data[0]; // 页号
    uint8_t response_data[132];    // 1(结果) + 1(页) + 128(配置项) + 2(预留)
    uint16_t data_index = 0;
    
    response_data[data_index++] = 0x00; // 结果：成功
    
    // 根据页号处理
    if (page == 0x00 || page == 0x01) { // 全部配置或第一页配置
        response_data[data_index++] = 0x01; // 页：第一页
        
        // 从Flash读取配置
        Config_ReadFromFlash();
        Config_Page1_t config;
        uint8_t read_result = Config_ReadPage1(&config);
        
        if (read_result != 0x00) {
            response_data[0] = 0x01; // 结果：失败
        }
        
        // 填充配置项（每个参数4字节，共32个参数，128字节）
        // 参数0：设备地址
        response_data[data_index++] = config.device_address;
      
        
        // 参数1：是否启用风速传感器
        response_data[data_index++] = config.sensor_enable;
               
        // 参数2：风速传感器采样周期
        response_data[data_index++] = config.sample_period;
        
        // 剩余参数填充0（第3-31参数）
        for (int i = 3; i < 32; i++) {
            response_data[data_index++] = 0x00;
            response_data[data_index++] = 0x00;
            response_data[data_index++] = 0x00;
            response_data[data_index++] = 0x00;
        }
        
        // 预留2字节
        response_data[data_index++] = 0x00;
        response_data[data_index++] = 0x00;
        
    } 
				//else if (page >= 0x02 && page <= 0x08) {
//        // 其他页（目前只支持第一页，其他页返回失败）
//        response_data[0] = 0x01; // 结果：失败
//        response_data[data_index++] = page;
//        
//        // 填充128字节的0
//        for (int i = 0; i < 128; i++) {
//            response_data[data_index++] = 0x00;
//        }
//        
//        // 预留2字节
//        response_data[data_index++] = 0x00;
//        response_data[data_index++] = 0x00;
//    } else {
//        return; // 无效页号，不响应
//    }
    
    // 发送响应数据
    RS485_SendResponse(hrs485, Response_READ_DATA, response_data, 132, hrs485->rx_frame.ID);
}

/**
 * @brief 写入数据命令处理
 */
void CommandHandler_WriteData(RS485_Handle_t *hrs485, RS485_Frame_t *frame) {
     uint8_t page = frame->data[0]; // 页号
    uint8_t response_data[4];      // 1(结果) + 1(页) + 2(预留)
    uint8_t result = 0x00;
    
    if (page == 0x01) { // 只处理第一页
        Config_Page1_t config;
        
        // 解析配置项
        uint16_t data_index = 1; // 跳过页号
        
        // 参数0：设备地址（第一个4字节）
        config.device_address = frame->data[data_index];
        data_index += 1; // 每个参数1字节
        
        // 参数1：是否启用风速传感器
        config.sensor_enable = frame->data[data_index];
        data_index += 1;// 每个参数1字节
        
        // 参数2：风速传感器采样周期
        config.sample_period = frame->data[data_index];
        data_index += 1;// 每个参数1字节
        
        // 写入配置到Flash
        result = Config_WritePage1(&config);
        
        // 如果写入成功，更新设备地址
        if (result == 0x00) {
            hrs485->device_id = config.device_address;
            
            // 重新读取配置到RAM
            Config_ReadFromFlash();
        }
        
    }
//		else if (page >= 0x02 && page <= 0x08) {
//        // 其他页（目前不支持，返回失败）
//        result = 0x01;
//    } else {
//        return; // 无效页号，不响应
//    }
    
    // 准备响应数据
    response_data[0] = result; // 结果
    response_data[1] = page;   // 页
    response_data[2] = 0x00;   // 预留
    response_data[3] = 0x00;   // 预留
		RS485_SendResponse(hrs485, Response_WRITE_DATA , response_data,sizeof(response_data) , hrs485->rx_frame.ID);
}

/**
 * @brief 注册所有命令处理函数
 */
void RegisterAllCommandHandlers(RS485_Handle_t *hrs485) {
    RS485_RegisterCommandHandler(hrs485, CMD_INFORMATION, CommandHandler_INFORMATION);
		RS485_RegisterCommandHandler(hrs485, CMD_CONTROL, CommandHandler_Control);
    RS485_RegisterCommandHandler(hrs485, CMD_READ_DATA, CommandHandler_ReadData);
    RS485_RegisterCommandHandler(hrs485, CMD_WRITE_DATA, CommandHandler_WriteData);
}