/**
 ******************************************************************************
 * @file    ActutorKincoDiff.h
 * @author
 * @version V1.0.0
 * @date
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ACTUATOR_KINCO_DIFF_H__
#define __ACTUATOR_KINCO_DIFF_H__

/* Includes ------------------------------------------------------------------*/
#include "TypeDefine.h"
#include "ActuatorBase.h"
#include "OpenCan_Drv.h"
#include "KincoCmd.h"

/* Exported macro ------------------------------------------------------------*/

#define ACT_KI_HEARTBET_TIME_MS 1000 // 2500ms
#define ACT_KI_SNYC_PERIOD_MS 10     // 10ms
#define DIFF_KI_BOOTUP_TIMEOUT_MS 5000
/* Exported constants --------------------------------------------------------*/
// typedef enum KI_DiffStatus
// {
//     KI_DIFF_INIT = 0,
//     KI_DIFF_BOOTUP,
//     KI_DIFF_STARTUP,
//     KI_DIFF_NORMAL,
//     KI_DIFF_RESET,
//     KI_DIFF_ERROR,
// } KI_DiffStatus_def;

/* Exported types ------------------------------------------------------------*/
typedef struct Act_KI_DriveParam
{
    uint8_t nodeId;
    uint8_t nodeState;
    KI_Error1_def errorCode1; // 错误代码1
    KI_Error2_def errorCode2; // 错误代码2
    KI_Status_def status;     // 状态字
    int16_t current_Dec;      // 电机电流
    uint16_t inputIOState;    // 驱动器输入IO状态
    int16_t temperature;      // 电机温度
    
    uint8_t staReadyPowerOn;  // 准备上电
    uint8_t staSwitchOn;      // 已经上电
    uint8_t staOpertEnable;   // 伺服运行使能
    uint8_t staFault;         // 故障标志
    uint8_t staEmStop;        // 急停标志;        

    uint16_t emcyError;
    uint8_t regError;
    uint8_t mrfsSpecErr[5];
    float Ipeak_A; // 峰值电流
    float current; // 单位A
    uint16_t voltage;
    uint8_t swVer[9];
    int32_t lastEncPosTick;

    uint32_t heartbeatTick;
} Act_KI_DriveParam_def;

typedef struct Act_KI_DiffCtrlParam
{
    uint8_t enable;
    pWalkModelCfgParam_def pCfgParam;
    WalkCtrlRunState_def runStatus;
    uint8_t step;
    uint32_t nextStatusStartTick;
    uint32_t tickCount;
    uint8_t emergentStop;
    uint8_t emStopType;
    uint8_t invalidEmStopDetectEnable;
    float emStopDecRps; // 快速停止减速度，单位rps
    uint8_t emcRecoverFlag;
	uint32_t encResolu; //编码器分辨率
    uint8_t isDetSpeedCmdTimeout;

    Act_KI_DriveParam_def drive[DIFF_WALK_NODE_COUNT];
    ActAxisData_def axisData[DIFF_WALK_AXIS_COUNT];
    float cmdVelX; // 设置线速度X方向
    float cmdVelY; // 设置线速度Y方向
    float cmdVelW; // 设置角速度
    float realVelX;
    float realVelY;
    float realVelW;

    DiffWalkOutputMsg_def outputMsg;
    DiffWalkOutputMsgCallBackDef diffWalkOutputMsgCallBack;
} Act_KI_DiffCtrlParam_def;

/* Exported functions ------------------------------------------------------- */

void ActKI_Init(pWalkModelCfgParam_def pCfgParam);

void ActKI_Node1RxCallBack(OpenCan_FrameType_def rxFrame);

void ActKI_Node2RxCallBack(OpenCan_FrameType_def rxFrame);

void ActKI_unpackPDO1Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackPDO2Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackPDO3Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackPDO4Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackSDORsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackNodeState(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_unpackEMCY(uint8_t nodeId, OpenCan_FrameType_def rxFrame);

void ActKI_RequestNodeState(void);

uint8_t ActKI_checkNodeFault(uint8_t nodeId);

uint8_t ActKI_sendRpm(uint8_t nodeId, float rpm);

uint8_t ActKI_ctrlCmd(uint8_t nodeId, uint16_t cmd);

void ActKI_DetectEmStopPin(void);

void ActKI_setRpm(uint8_t nodeId, float rpm);

void ActKI_WalkCtrl(float velX, float velY, float velW); // 线速度，角速度

int32_t ActKI_OptCtrl(uint8_t cmd, uint8_t *dataBuf, uint16_t dataLen);

void ActKI_setStatus(WalkCtrlRunState_def status);

void ActKI_ReleaseBrakeCtrl(uint8_t enable);

void ActKI_setOutputDiffMsgCallBack(DiffWalkOutputMsgCallBackDef callBack);

void ActKI_OutputMsg(void);

void ActKI_setTickMeter(uint8_t nodeId, uint32_t tickMeter);

uint32_t ActKI_getTickMeter(uint8_t nodeId);

void ActKI_SendCmdSpeed(void);

void ActKI_EmStopProcess(void);

void ActKI_processLoop(void);

#endif
