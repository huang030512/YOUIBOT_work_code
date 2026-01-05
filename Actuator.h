/**
 ******************************************************************************
 * @file    Actutor.h
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
#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__

/* Includes ------------------------------------------------------------------*/
#include "TypeDefine.h"
#include "ActuatorBase.h"

typedef struct ActDrive
{
    uint8_t enable;
    ActuatorBrand_def brand;
    ActuatorType_def type;
    uint8_t walkLockState;
    void (*Init)(pWalkModelCfgParam_def pCfgParam);
    void (*SetRpm)(uint8_t nodeId, float rpm);
    void (*WalkCtrl)(float velX, float velY, float velW);
    int32_t (*OptCtrl)(uint8_t cmd, uint8_t *dataBuf, uint16_t dataLen);
    void (*setParkState)(uint8_t state);
    void (*SetOutputDiffMsgCallBack)(DiffWalkOutputMsgCallBackDef callBack);
    void (*SetOutputSteerMsgCallBack)(SteerWalkOutputMsgCallBackDef callBack);
    void (*SetOutputSteer2MsgCallBack)(Steer2WalkOutputMsgCallBackDef callBack);
    void (*SetOutputFourWheel2MsgCallBack)(FourWheel2WalkOutputMsgCallBackDef callBack);
    void (*SetOutputFourWheel4MsgCallBack)(FourWheel4WalkOutputMsgCallBackDef callBack);
    void (*SetOutputDiff2MsgCallBack)(Diff2WalkOutputMsgCallBackDef callBack);
    void (*SetOutputDiff4MsgCallBack)(Diff4WalkOutputMsgCallBackDef callBack);
    void (*SetOutputSingleAxisMsgCallback)(SingleAxisOutputMsgCallBackDef callBack);
    void (*ProcessLoop)(void);
    void (*SetDiffTickMeter)(uint8_t nodeId, uint32_t tickMeter);
    uint32_t (*GetDiffTickMeter)(uint8_t nodeId);
} ActDrive_def;

/* Exported types ------------------------------------------------------------*/

void Actuator_Init(pWalkModelCfgParam_def pCfgParam);

void Actuator_setLogLevel(uint8_t level);

void Actuator_setRpm(uint8_t nodeId, float rpm);

void Actuator_WalkCtrl(float velX, float velY, float velW);

int32_t Actuator_OptCtrl(uint8_t cmd, uint8_t *dataBuf, uint16_t dataLen);

void Actuator_setOutputDiffMsgCallBack(DiffWalkOutputMsgCallBackDef callBack);

void Actuator_SetOutputSteerMsgCallBack(SteerWalkOutputMsgCallBackDef callBack);

void Actuator_SetOutputSteer2MsgCallBack(Steer2WalkOutputMsgCallBackDef callBack);

void Actuator_SetOutputDiff2MsgCallBack(Diff2WalkOutputMsgCallBackDef callBack);

void Actuator_SetOutputDiff4MsgCallBack(Diff4WalkOutputMsgCallBackDef callBack);

void Actuator_SetOutputSingAxisMsgCallBack(SingleAxisOutputMsgCallBackDef callBack);

void Actuator_SetOutputFourWheel2MsgCallBack(FourWheel2WalkOutputMsgCallBackDef callBack);

void Actuator_setLockWalk(uint8_t state);

void Actuator_SetOutputFourWheel4MsgCallBack(FourWheel4WalkOutputMsgCallBackDef callBack);

void Actuator_SetParkState(uint8_t state);

void Actuator_SetDiffTickMeter(uint8_t nodeId, uint32_t tickMeter);

uint32_t Actuator_GetDiffTickMeter(uint8_t nodeId);

void Actuator_Loop(void);

#endif
