#include "rs485.h"
#include "debug.h"

// 帧头定义
#define FRAME_HEADER_1 0x55
#define FRAME_HEADER_2 0xAA

// 命令处理函数指针数组
static CommandHandler_t command_handlers[256] = {0};

// CRC-16计算
uint16_t RS485_Frame_CalculateCRC16(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t i;
    
    while (length--) {
        crc ^= *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}
/**
 * @brief 初始化RS485
 */
void RS485_Init(RS485_Handle_t *hrs485, UART_HandleTypeDef *huart, 
                GPIO_TypeDef *de_port, uint16_t de_pin, uint8_t device_id) {
    hrs485->huart = huart;
    hrs485->de_port = de_port;
    hrs485->de_pin = de_pin;
    hrs485->device_id = device_id;
    
    // 初始化状态
    hrs485->state = STATE_IDLE;
    hrs485->rx_data_index = 0;
    hrs485->frame_received = false;
    hrs485->frame_error = false;
    
    // 初始化统计
    hrs485->rx_frame_count = 0;
    hrs485->tx_frame_count = 0;
    hrs485->error_count = 0;
    
    // 注册默认命令处理器
    
    
    // 初始化为接收模式
    HAL_GPIO_WritePin(hrs485->de_port, hrs485->de_pin, GPIO_PIN_RESET);
}

/**
 * @brief 开始接收数据（中断方式）
 */
void RS485_StartReceive(RS485_Handle_t *hrs485) {
    hrs485->state = STATE_IDLE;
    hrs485->frame_received = false;
    hrs485->frame_error = false;
    hrs485->rx_data_index = 0;
    
    // 启动UART接收中断，每次接收1个字节
    HAL_UART_Receive_IT(hrs485->huart, &hrs485->rx_byte, 1);
	//USART_printf("receive");
}

/**
 * @brief UART接收完成回调函数 - 核心状态机
 */
void RS485_UART_RxCpltCallback(RS485_Handle_t *hrs485) {
    switch (hrs485->state) {
        case STATE_IDLE:
            // 等待帧头1
            if (hrs485->rx_byte == FRAME_HEADER_1) {
								hrs485->rx_frame.header[0] = hrs485->rx_byte; // 保存帧头1
                hrs485->state = STATE_HEADER_2;
            }
            break;
            
        case STATE_HEADER_2:
            // 等待帧头2
            if (hrs485->rx_byte == FRAME_HEADER_2) {
								hrs485->rx_frame.header[1] = hrs485->rx_byte;  // 保存帧头2
                hrs485->state = STATE_DEVICE_ID;
            } 
						else {
                hrs485->state = STATE_IDLE; // 帧头错误，重置
                hrs485->error_count++;
            }
            break;
            
        case STATE_DEVICE_ID:
            // 接收板地址
						hrs485->rx_frame.device_id = hrs485->rx_byte;
            hrs485->state = STATE_ID;
            break;
        case STATE_ID:
            // 接收会话ID
            hrs485->rx_frame.ID = hrs485->rx_byte;
            hrs485->state = STATE_LENGTH;
            break;
            
        case STATE_LENGTH:
            // 接收数据长度
            hrs485->rx_frame.length = hrs485->rx_byte;
            hrs485->rx_data_index = 0;
            
            if (hrs485->rx_frame.length == 0) {
                hrs485->state = STATE_CRC_LOW; // 没有数据，直接到校验和
            } else if (hrs485->rx_frame.length <= sizeof(hrs485->rx_frame.data)) {
                hrs485->state = STATE_COMMAND;
            } else {
                // 数据长度过长，帧错误
                hrs485->state = STATE_IDLE;
                hrs485->error_count++;
            }
            break;
				 case STATE_COMMAND:
            // 将N位数据位拆为一位命令码和N-1位命令值
            hrs485->rx_frame.command = hrs485->rx_byte;
            hrs485->state = STATE_DATA;
            break;
            
        case STATE_DATA:
            // 接收数据
            hrs485->rx_frame.data[hrs485->rx_data_index++] = hrs485->rx_byte;
            if (hrs485->rx_data_index >= hrs485->rx_frame.length-1) {
                hrs485->state = STATE_CRC_LOW;
            }
            break;
            
        case STATE_CRC_LOW:
            // 接收CRC低字节
            hrs485->rx_frame.crc16 = hrs485->rx_byte; // 先存低字节
            hrs485->state = STATE_CRC_HIGH;
            break;
            
        case STATE_CRC_HIGH:
            // 接收CRC高字节
            hrs485->rx_frame.crc16 |= (hrs485->rx_byte << 8); // 合并高字节
            
            // 验证CRC
            if (RS485_Frame_ValidateCRC16(&hrs485->rx_frame)) {
                hrs485->frame_received = true;
                hrs485->rx_frame_count++;
            } else {
                hrs485->frame_error = true;
                hrs485->error_count++;
            }
					 hrs485->state = STATE_IDLE;
            break;
            
        default:
            hrs485->state = STATE_IDLE;
            break;
    }
    
    // 如果不是帧接收完成状态，继续接收下一个字节
    if (!hrs485->frame_received && !hrs485->frame_error) {
        HAL_UART_Receive_IT(hrs485->huart, &hrs485->rx_byte, 1);
    }
}

/**
 * @brief 处理接收到的完整帧
 */
void RS485_ProcessReceivedFrame(RS485_Handle_t *hrs485) {
    if (!hrs485->frame_received) {
        return;
    }
    
    // 检查设备地址是否匹配
    if (hrs485->rx_frame.device_id != hrs485->device_id ) {
        hrs485->frame_received = false;
        return;
    }
    
    // 调用命令处理函数
    if (command_handlers[hrs485->rx_frame.command] != NULL) {
        command_handlers[hrs485->rx_frame.command](hrs485, &hrs485->rx_frame);
    }
    
    hrs485->frame_received = false;
		//处理完一帧接着接受下一帧
		RS485_StartReceive(hrs485);
}

/**
 * @brief 注册命令处理函数
 */
void RS485_RegisterCommandHandler(RS485_Handle_t *hrs485, uint8_t command, CommandHandler_t handler) {
        command_handlers[command] = handler;
}



/**
 * @brief 发送响应帧
 */
void RS485_SendResponse(RS485_Handle_t *hrs485, uint8_t command, uint8_t *data, uint8_t length,uint8_t ID) {
    RS485_Frame_Init(&hrs485->tx_frame, hrs485->device_id, command,ID);
    
    if (data != NULL && length > 0) {
        memcpy(hrs485->tx_frame.data, data, length);
        hrs485->tx_frame.length = length+1;
    }
    
    RS485_SendFrame(hrs485, &hrs485->tx_frame);
}


// ==================== 数据帧处理函数 ====================

/**
 * @brief 初始化数据帧
 */
void RS485_Frame_Init(RS485_Frame_t *frame, uint8_t device_id, uint8_t command,uint8_t ID) {
    frame->header[0] = FRAME_HEADER_1;
    frame->header[1] = FRAME_HEADER_2;
    frame->device_id = device_id;
    frame->ID = ID;
    frame->length = 0;
	  frame->command= command;
    memset(frame->data, 0, sizeof(frame->data));
    frame->crc16 = 0;
}

/**
 * @brief 验证CRC16
 */
bool RS485_Frame_ValidateCRC16(RS485_Frame_t *frame) {
    // 计算CRC的数据范围：从设备地址到数据结束
    uint8_t crc_data[2 + 1 + 1 + 1 + 1 + 248]; // header(2) + device_id(1) + ID(1) + length(1) + command(1) + data(248)
    uint16_t crc_length = 0;
    
    // 构建CRC计算数据
	  crc_data[crc_length++] = frame->header[0];  // 帧头1
    crc_data[crc_length++] = frame->header[1];  // 帧头2
    crc_data[crc_length++] = frame->device_id;
    crc_data[crc_length++] = frame->ID;
    crc_data[crc_length++] = frame->length;
    crc_data[crc_length++] = frame->command;
    for (int i = 0; i < frame->length-1; i++) {
        crc_data[crc_length++] = frame->data[i];
    }
    
    // 计算CRC并与帧中的CRC比较
    uint16_t calculated_crc = RS485_Frame_CalculateCRC16(crc_data, crc_length);
    return (frame->crc16 == calculated_crc);
}

/**
 * @brief 序列化帧数据
 */
uint16_t RS485_Frame_Serialize(RS485_Frame_t *frame, uint8_t *buffer) {
    uint16_t index = 0;
    uint8_t crc_data[2 + 1 + 1 + 1 + 1 + 248]; // 用于CRC计算的数据
    uint16_t crc_length = 0;
    
    // 填充缓冲区头部
    buffer[index++] = frame->header[0];
    buffer[index++] = frame->header[1];
    buffer[index++] = frame->device_id;
    buffer[index++] = frame->ID;
    buffer[index++] = frame->length;
    buffer[index++] = frame->command;
    // 构建CRC计算数据
	   crc_data[crc_length++] = frame->header[0];  // 帧头1
    crc_data[crc_length++] = frame->header[1];  // 帧头2
    crc_data[crc_length++] = frame->device_id;
    crc_data[crc_length++] = frame->ID;
    crc_data[crc_length++] = frame->length;
    crc_data[crc_length++] = frame->command;
    // 填充数据和构建CRC数据
    for (int i = 0; i < frame->length-1; i++) {
        buffer[index++] = frame->data[i];
        crc_data[crc_length++] = frame->data[i];
    }
    
    // 计算CRC（不包括帧头）
    frame->crc16 = RS485_Frame_CalculateCRC16(crc_data, crc_length);
    
    // 填充CRC（低字节在前）
    buffer[index++] = frame->crc16 & 0xFF;        // CRC低字节
    buffer[index++] = (frame->crc16 >> 8) & 0xFF; // CRC高字节
    
    return index; // 返回帧长度
}

/**
 * @brief 发送数据帧
 */
HAL_StatusTypeDef RS485_SendFrame(RS485_Handle_t *hrs485, RS485_Frame_t *frame) {
    HAL_StatusTypeDef status;
    uint8_t tx_buffer[260]; // 最大帧长度
    uint16_t frame_length;
    
    // 序列化帧
    frame_length = RS485_Frame_Serialize(frame, tx_buffer);
    
    // 设置为发送模式
    HAL_GPIO_WritePin(hrs485->de_port, hrs485->de_pin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 50; i++); // 短暂延时
    
    // 发送数据
    status = HAL_UART_Transmit(hrs485->huart, tx_buffer, frame_length, 1000);
    
    // 等待发送完成
    HAL_Delay(5);
    
    // 切换回接收模式
    HAL_GPIO_WritePin(hrs485->de_port, hrs485->de_pin, GPIO_PIN_RESET);
    
    if (status == HAL_OK) {
        hrs485->tx_frame_count++;
    }
    
    return status;
}