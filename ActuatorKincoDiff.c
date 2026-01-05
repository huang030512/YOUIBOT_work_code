/**
 ******************************************************************************
 * @file    ActutorKincoDiff.c
 * @author
 * @version V1.0.0
 * @date
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ActuatorKincoDiff.h"
#include "superMath.h"
#include <string.h>
#include "common.h"
#include "ActComAPI.h"
#include "SysAppCtrlParam.h"

__attribute__((section(".ccmram"))) Act_KI_DiffCtrlParam_def gActKiCtrlParam;

#define KI_DIFF_PROCESS_LOOP_TIME_MS 10
#define KI_DIFF_SYNC_ACT_DATA_PERIOD_MS 5

/*************************** function define **********************************************/
void ActKI_Init(pWalkModelCfgParam_def pCfgParam)
{
    memset(gActKiCtrlParam.axisData, 0, sizeof(gActKiCtrlParam.axisData));
    memset(gActKiCtrlParam.drive, 0, sizeof(gActKiCtrlParam.drive));

    gActKiCtrlParam.pCfgParam = pCfgParam;

    gActKiCtrlParam.drive[0].nodeState = NODE_STATE_PRE_OPERATION;
    gActKiCtrlParam.drive[0].nodeId = ACTUATOR_NODE1_ID;
    gActKiCtrlParam.drive[1].nodeState = NODE_STATE_PRE_OPERATION;
    gActKiCtrlParam.drive[1].nodeId = ACTUATOR_NODE2_ID;

    OpenCan_addNodeCallBack(ACTUATOR_NODE1_ID, ActKI_Node1RxCallBack);
    OpenCan_addNodeCallBack(ACTUATOR_NODE2_ID, ActKI_Node2RxCallBack);

    gActKiCtrlParam.enable = 1;
    gActKiCtrlParam.diffWalkOutputMsgCallBack = NULL;
    gActKiCtrlParam.runStatus = eWALK_CTRL_STA_INIT;
    gActKiCtrlParam.emStopType = eEmStopTrigStopType_None;
    gActKiCtrlParam.invalidEmStopDetectEnable = 0;

    if ((pCfgParam->item.emStopDec > 0 && pCfgParam->item.emStopDec < 10))
    {
        gActKiCtrlParam.outputMsg.sysErr.bit.cfgParamErr = 1;
        ACT_ERR("config parameter is error");
    }
}

void ActKI_ParaseNodeData(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_EMCY)
    {
        // NMT 0x0080~0x017F
        ActKI_unpackEMCY(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_TPDO1)
    {
        // PDO1 0x0180~0x027F
        ActKI_unpackPDO1Rsp(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_TPDO2)
    {
        // PDO2 0x0280~0x037F
        ActKI_unpackPDO2Rsp(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_TPDO3)
    {
        // PDO3 0x0380~0x047F
        ActKI_unpackPDO3Rsp(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_TPDO4)
    {
        // PDO4 0x0480~0x057F
        // ActKI_unpackPDO4Rsp(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_TSDO)
    {
        // SDO rsp  0x0580~0x6FF
        // ActKI_unpackSDORsp(nodeId, rxFrame);
    }
    else if ((rxFrame.COB_id & FUNCT_CODE_MASK) == FUNCT_CODE_HEART_BEAT)
    {
        // NMI node protect or heartbear: 0x0700~0x07FF
        ActKI_unpackNodeState(nodeId, rxFrame);
    }
    else
    {
        // unknown cob id
    }
}

void ActKI_Node1RxCallBack(OpenCan_FrameType_def rxFrame)
{
    ActKI_ParaseNodeData(ACTUATOR_NODE1_ID, rxFrame);
}

void ActKI_Node2RxCallBack(OpenCan_FrameType_def rxFrame)
{
    ActKI_ParaseNodeData(ACTUATOR_NODE2_ID, rxFrame);
}

void ActKI_unpackPDO1Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    static int32_t lastRealPos[DIFF_WALK_NODE_COUNT] = {0}; // 电机位置
    Int tmp = {0};

    for (int i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].nodeId == nodeId)
        {
            // 实时位置
            for (uint8_t i = 0; i < 4; i++)
            {
                tmp.Byte[i] = rxFrame.dataBuf[i];
            }

            // 编码器过滤
            if (gActKiCtrlParam.pCfgParam->item.encoderFilterThresh > 0)
            {
                if (abs(lastRealPos[i] - tmp.SignedValue) > gActKiCtrlParam.pCfgParam->item.encoderFilterThresh)
                {
                    lastRealPos[i] = tmp.SignedValue;
                    gActKiCtrlParam.axisData[i].realPos = tmp.SignedValue; // real tick position
                }
            }
            else
            {
                gActKiCtrlParam.axisData[i].realPos = tmp.SignedValue; // real tick position
            }

            // 实时速度
            for (uint8_t i = 0; i < 4; i++)
            {
                tmp.Byte[i] = rxFrame.dataBuf[4 + i];
            }
            gActKiCtrlParam.axisData[i].realRpm = Kinco_Dec2Rpm(tmp.SignedValue, gActKiCtrlParam.encResolu);

            // Rpm 过滤
            if (gActKiCtrlParam.pCfgParam->item.RpmFilterThresh > 0)
            {
                if (fabs(gActKiCtrlParam.axisData[i].realRpm) < gActKiCtrlParam.pCfgParam->item.RpmFilterThresh)
                {
                    gActKiCtrlParam.axisData[i].realRpm = 0;
                }
            }

            gActKiCtrlParam.axisData[i].realSpeed = Act_RpmToSpeed(gActKiCtrlParam.axisData[i].realRpm,
                                                                   gActKiCtrlParam.encResolu, gActKiCtrlParam.pCfgParam->item.diffWhlTickMeter[i]);

            break;
        }
    }

    gActKiCtrlParam.realVelX = (gActKiCtrlParam.axisData[0].realSpeed + gActKiCtrlParam.axisData[1].realSpeed) / 2.0f; // 计算线速度
    if (gActKiCtrlParam.pCfgParam->item.wheelDist_y)
    {
        gActKiCtrlParam.realVelW = (gActKiCtrlParam.axisData[1].realSpeed - gActKiCtrlParam.axisData[0].realSpeed) / gActKiCtrlParam.pCfgParam->item.wheelDist_y; // 计算角速度
    }

    // output rpm and position tick msg
    ActKI_OutputMsg();
}
void ActKI_unpackPDO2Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].nodeId == nodeId)
        {
            Short tmp = {0};

            tmp.Byte[0] = rxFrame.dataBuf[0]; // 状态字
            tmp.Byte[1] = rxFrame.dataBuf[1];
            gActKiCtrlParam.drive[i].status.val = tmp.UnsignedValue;

            gActKiCtrlParam.drive[i].staReadyPowerOn = gActKiCtrlParam.drive[i].status.bit.readyPowerOn;
            gActKiCtrlParam.drive[i].staOpertEnable = gActKiCtrlParam.drive[i].status.bit.OpertEnable;
            gActKiCtrlParam.drive[i].staFault = gActKiCtrlParam.drive[i].status.bit.fault; // fault
            gActKiCtrlParam.drive[i].staSwitchOn = gActKiCtrlParam.drive[i].status.bit.switchOn;
            gActKiCtrlParam.drive[i].staEmStop = !gActKiCtrlParam.drive[i].status.bit.fastStop;

            tmp.Byte[0] = rxFrame.dataBuf[2];
            tmp.Byte[1] = rxFrame.dataBuf[3];
            gActKiCtrlParam.drive[i].errorCode1.val = tmp.UnsignedValue; // 错误代码1

            tmp.Byte[0] = rxFrame.dataBuf[4];
            tmp.Byte[1] = rxFrame.dataBuf[5];
            gActKiCtrlParam.drive[i].errorCode2.val = tmp.UnsignedValue; // 错误代码2

            tmp.Byte[0] = rxFrame.dataBuf[6];
            tmp.Byte[1] = rxFrame.dataBuf[7];
            gActKiCtrlParam.drive[i].inputIOState = tmp.UnsignedValue; // 驱动器IO输入状态

            break;
        }
    }
}
void ActKI_unpackPDO3Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    Int tmp = {0};

    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        tmp.Byte[0] = rxFrame.dataBuf[0]; // 电流
        tmp.Byte[1] = rxFrame.dataBuf[1];
        gActKiCtrlParam.drive[i].current_Dec = tmp.SignedValue;
        if (gActKiCtrlParam.drive[i].Ipeak_A)
        {
            gActKiCtrlParam.drive[i].current = gActKiCtrlParam.drive[i].current_Dec / (2048.0f / (gActKiCtrlParam.drive[i].Ipeak_A / 1.414f));
        }

        tmp.Byte[0] = rxFrame.dataBuf[2];
        tmp.Byte[1] = rxFrame.dataBuf[3];
        gActKiCtrlParam.drive[i].voltage = tmp.UnsignedValue;

        tmp.Byte[0] = rxFrame.dataBuf[4];
        tmp.Byte[1] = rxFrame.dataBuf[5];
        gActKiCtrlParam.drive[i].temperature = tmp.UnsignedValue;
    }
}

// void ActKI_unpackPDO4Rsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
// {

// }

// void ActKI_unpackSDORsp(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
// {

// }

void ActKI_unpackNodeState(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].nodeId == nodeId)
        {
            gActKiCtrlParam.drive[i].nodeState = rxFrame.dataBuf[0] & 0x7F;
            gActKiCtrlParam.drive[i].heartbeatTick = HAL_GetTick();

            if (gActKiCtrlParam.runStatus != eWALK_CTRL_STA_INIT && gActKiCtrlParam.runStatus != eWALK_CTRL_STA_BOOTUP)
            {
                if (gActKiCtrlParam.drive[i].nodeState != NODE_STATE_OPERATION)
                {
                    if (!gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.nodeStateExcept)
                    {
                        gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.nodeStateExcept = 1;
                        ACT_ERR("actuator(id=%d) node state exception!(node state = 0x%02x)", gActKiCtrlParam.drive[i].nodeId, gActKiCtrlParam.drive[i].nodeState);
                    }
                }
            }

            break;
        }
    }
}

void ActKI_unpackEMCY(uint8_t nodeId, OpenCan_FrameType_def rxFrame)
{
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].nodeId == nodeId)
        {
            gActKiCtrlParam.drive[i].emcyError = (rxFrame.dataBuf[0] | rxFrame.dataBuf[1] << 8);
            gActKiCtrlParam.drive[i].regError = rxFrame.dataBuf[2];
            memcpy(gActKiCtrlParam.drive[i].mrfsSpecErr, &rxFrame.dataBuf[3], 5);

            break;
        }
    }
}

void ActKI_setRpm(uint8_t nodeId, float rpm)
{
}

void ActKI_WalkCtrl(float velX, float velY, float velW)
{
    gActKiCtrlParam.cmdVelX = velX;
    gActKiCtrlParam.cmdVelY = velY;
    gActKiCtrlParam.cmdVelW = velW;
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        gActKiCtrlParam.axisData[i].cmdInterval = gActKiCtrlParam.pCfgParam->item.cmdTimeoutMs;
    }

    if (gActKiCtrlParam.emergentStop || gActKiCtrlParam.emcRecoverFlag || gActKiCtrlParam.runStatus != eWALK_CTRL_STA_NORMAL)
    {
        return;
    }
    else
    {
        ActKI_SendCmdSpeed();
    }
}

int32_t ActKI_OptCtrl(uint8_t cmd, uint8_t *dataBuf, uint16_t dataLen)
{
    ACT_DEBUG("Rx OptCtrl cmd = 0x%02x", cmd);

    if (cmd == ACT_CTRL_BRAKE_CMD && dataBuf != NULL)
    {
        ActKI_ReleaseBrakeCtrl(dataBuf[0]);

        return 0;
    }
    else if (cmd == ACT_ALL_MOTOR_DISOPERT_CMD)
    {
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
        }

        return 0;
    }
    else if (cmd == ACT_EMSTOP_TRIG_STOP_TYPE_CMD && dataBuf != NULL)
    {
        uint32_t emStopDecTime = 0;

        ACT_INFO("emStop type = %d", dataBuf[0]);
        if (dataBuf[0] < gActKiCtrlParam.emStopType)
        {
            gActKiCtrlParam.emStopType = dataBuf[0];
            gActKiCtrlParam.emergentStop = 1;
            gActKiCtrlParam.emcRecoverFlag = 0;
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                gActKiCtrlParam.drive[i].lastEncPosTick = 0;
            }

            if (gActKiCtrlParam.emStopType == eEmStopTrigStopType_1)
            {
                if (gActKiCtrlParam.emStopDecRps)
                {
                    float maxRpm = 0;
                    if (fabs(gActKiCtrlParam.axisData[0].realRpm) > fabs(gActKiCtrlParam.axisData[1].realRpm))
                    {
                        maxRpm = fabs(gActKiCtrlParam.axisData[0].realRpm);
                    }
                    else
                    {
                        maxRpm = fabs(gActKiCtrlParam.axisData[1].realRpm);
                    }

                    emStopDecTime = maxRpm / 60.0f / gActKiCtrlParam.emStopDecRps * 1000; // 时间单位为ms
                }
                emStopDecTime += ACT_MOTOR_DISOPERT_TIME_MS; // 增加200ms，用于增加掉使能时间
                ACT_INFO("emstopDecTime = %d ms", emStopDecTime);
            }
            ActKI_EmStopProcess();
        }
        else
        {
            ACT_INFO("New emstop type priority is lower than current emstop type ");
        }

        return emStopDecTime;
    }
    else if (cmd == ACT_CLEAR_EMSTOP_TYPE_CMD)
    {
        ACT_INFO("RX clear emStop cmd!");
        if (gActKiCtrlParam.emergentStop)
        {
            gActKiCtrlParam.emergentStop = 0;
            gActKiCtrlParam.emcRecoverFlag = 1;
        }
    }
    else if (cmd == ACT_SET_DET_INVALID_EMSTOP_CMD && dataBuf != NULL)
    {
        ACT_INFO("Set invalid EmStop Detect Enable =%d!", dataBuf[0]);
        gActKiCtrlParam.invalidEmStopDetectEnable = dataBuf[0];
    }
    else if (cmd == ACT_SET_DET_SPEED_INST_TIMEOUT_CMD && dataBuf != NULL)
    {
        ACT_INFO("Set detect speed timeout enable = %d ", dataBuf[0]);
        gActKiCtrlParam.isDetSpeedCmdTimeout = dataBuf[0];
    }

    return 0;
}

void ActKI_EmStopProcess(void)
{
    if (gActKiCtrlParam.runStatus == eWALK_CTRL_STA_INIT || gActKiCtrlParam.runStatus == eWALK_CTRL_STA_BOOTUP)
    {
        return;
    }

    if (gActKiCtrlParam.emergentStop)
    {
        gActKiCtrlParam.cmdVelX = 0;
        gActKiCtrlParam.cmdVelY = 0;
        gActKiCtrlParam.cmdVelW = 0;

        if (gActKiCtrlParam.emStopType == eEmStopTrigStopType_0 || gActKiCtrlParam.emStopType == eEmStopTrigStopType_1)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (gActKiCtrlParam.drive[i].staOpertEnable && !gActKiCtrlParam.drive[i].staEmStop)
                {
                    // 急停，设置快速停止
                    ACT_DEBUG("send fast stop cmd(id=%d)!", gActKiCtrlParam.drive[i].nodeId);
                    ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_FAST_STOP_CMD);
                }
            }
        }
        else if (gActKiCtrlParam.emStopType == eEmStopTrigStopType_2)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (gActKiCtrlParam.drive[i].staOpertEnable)
                {
                    if (fabs(gActKiCtrlParam.axisData[i].realRpm) > 100 || abs(gActKiCtrlParam.axisData[i].realPos - gActKiCtrlParam.drive[i].lastEncPosTick) > 1000)
                    {
                        gActKiCtrlParam.drive[i].lastEncPosTick = gActKiCtrlParam.axisData[i].realPos;
                        // 发送暂停指令
                        ACT_DEBUG("send suspend stop cmd(id=%d, rpm=%f)!", gActKiCtrlParam.drive[i].nodeId, gActKiCtrlParam.axisData[i].realRpm);
                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_HALT_STOP_CMD);
                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_HALT_STOP1_CMD);
                    }
                }
            }
        }
    }
    else if (gActKiCtrlParam.emcRecoverFlag)
    {
        uint8_t checkPass = 1;

        if (gActKiCtrlParam.emStopType == eEmStopTrigStopType_2)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                ACT_DEBUG("Send Power on cmd(id=%d)", gActKiCtrlParam.drive[i].nodeId);
                ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_ON);
            }
        }
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            /* 清错急停导致的错误或者急停信号位*/
            if (gActKiCtrlParam.drive[i].staFault || gActKiCtrlParam.drive[i].staEmStop)
            {
                ACT_DEBUG("send clear fault cmd!(status=0x%02x, error1=0x%02x, error2=0x%02x, id=%d)",
                          gActKiCtrlParam.drive[i].status.val, gActKiCtrlParam.drive[i].errorCode1.val,
                          gActKiCtrlParam.drive[i].errorCode2.val, gActKiCtrlParam.drive[i].nodeId);

                ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
                ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_CLEAR_FAULT_CMD);
                checkPass = 0;
            }
        }

        if (checkPass)
        {
            gActKiCtrlParam.emStopType = eEmStopTrigStopType_None;
            gActKiCtrlParam.emcRecoverFlag = 0;
            gActKiCtrlParam.tickCount = 0;
            gActKiCtrlParam.nextStatusStartTick = HAL_GetTick();
            // 清除急停前保留的速度
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                ACT_DEBUG("Send rpm = 0 cmd(id=%d)", gActKiCtrlParam.drive[i].nodeId);
                ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
            }
        }
    }

    if (gActKiCtrlParam.invalidEmStopDetectEnable && !gActKiCtrlParam.emcRecoverFlag && !gActKiCtrlParam.emergentStop)
    {
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            if (gActKiCtrlParam.drive[i].staEmStop)
            {
                if (!gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.invalidEmStop)
                {
                    ACT_ERR("Actuator(id=%d): Invalid emStop trigger!", gActKiCtrlParam.drive[i].nodeId);
                    gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.invalidEmStop = 1;
                }
            }
            else
            {
                if (gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.invalidEmStop)
                {
                    ACT_INFO("Actuator(id=%d): Invalid emStop clear!", gActKiCtrlParam.drive[i].nodeId);
                    gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.invalidEmStop = 0;
                }
            }
        }
    }
}

uint8_t ActKI_sendRpm(uint8_t nodeId, float rpm)
{
    OpenCan_FrameType_def txFrame;
    int32_t cmdRpm = 0;

    // 变为DEC， 单位换算关系为 DEC=[(RPM*512*编码器分辨率)/1875
    // cmdRpm = (int32_t)(rpm * 512 * gActKiCtrlParam.encResolu / 1875);
    cmdRpm = Kinco_Rpm2Dec(rpm, gActKiCtrlParam.encResolu);

    txFrame.COB_id = FUNCT_CODE_RPDO2 | nodeId;
    txFrame.dataLen = 4;
    txFrame.dataBuf[0] = (uint8_t)(cmdRpm);
    txFrame.dataBuf[1] = (uint8_t)(cmdRpm >> 8);
    txFrame.dataBuf[2] = (uint8_t)(cmdRpm >> 16);
    txFrame.dataBuf[3] = (uint8_t)(cmdRpm >> 24);

    return OpenCan_transPDO(ACTUATOR_CAN_PORT, txFrame);
}
uint8_t ActKI_ctrlCmd(uint8_t nodeId, uint16_t cmd) // 控制字
{
    OpenCan_FrameType_def txFrame;

    txFrame.COB_id = FUNCT_CODE_RPDO1 | nodeId;
    txFrame.dataLen = 2;
    txFrame.dataBuf[0] = (uint8_t)(cmd);
    txFrame.dataBuf[1] = (uint8_t)(cmd >> 8);

    return OpenCan_transPDO(ACTUATOR_CAN_PORT, txFrame);
}

uint8_t ActKI_checkNodeFault(uint8_t nodeId)
{
    uint8_t errorCode = 0;

    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].nodeId == nodeId)
        {
            if (gActKiCtrlParam.drive[i].staFault || !gActKiCtrlParam.drive[i].staOpertEnable || !gActKiCtrlParam.drive[i].status.bit.fastStop)
            {
                errorCode = 1;
            }
            break;
        }
    }

    return errorCode;
}

void ActKI_CmdIntervalProcess(void)
{
    if (gActKiCtrlParam.runStatus != eWALK_CTRL_STA_NORMAL || !gActKiCtrlParam.isDetSpeedCmdTimeout)
    {
        return;
    }

    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.axisData[i].cmdInterval > 0)
        {
            gActKiCtrlParam.axisData[i].cmdInterval -= KI_DIFF_PROCESS_LOOP_TIME_MS;
            if (gActKiCtrlParam.axisData[i].cmdInterval <= 0)
            {
                ACT_ERR("Walk Cmd interval timeout! i=%d ", i);
                gActKiCtrlParam.axisData[i].cmdInterval = 0;
                if (gActKiCtrlParam.axisData[i].cmdRpm != 0.0f)
                {
                    gActKiCtrlParam.axisData[i].cmdRpm = 0.0f;
                }
                gActKiCtrlParam.cmdVelX = 0;
                gActKiCtrlParam.cmdVelY = 0;
                gActKiCtrlParam.cmdVelW = 0;
            }
        }
    }
}

void ActKI_RequestNodeState(void)
{
    static uint32_t lastTicket = 0;

    if (HAL_GetTick() - lastTicket > 100)
    {
        lastTicket = HAL_GetTick();

        if (gActKiCtrlParam.runStatus != eWALK_CTRL_STA_INIT && gActKiCtrlParam.runStatus != eWALK_CTRL_STA_BOOTUP)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                OpenCan_transNodeProtect(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId); // 询问节点状态
            }
        }
    }
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (gActKiCtrlParam.drive[i].heartbeatTick == 0)
        {
            gActKiCtrlParam.drive[i].heartbeatTick = HAL_GetTick();
        }

        if (HAL_GetTick() - gActKiCtrlParam.drive[i].heartbeatTick > KINCO_HEARTBEAT_TIMEOUT_MS)
        {
            if (!gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.heartbeatTimeout)
            {
                ACT_ERR("Actuator(id=%d): Heartbeat timeout!", gActKiCtrlParam.drive[i].nodeId);
                gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.heartbeatTimeout = 1;
            }
        }
        else
        {
            if (gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.heartbeatTimeout)
            {
                gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.heartbeatTimeout = 0;
                ACT_INFO("Actuator(id=%d): Heartbeat restore!", gActKiCtrlParam.drive[i].nodeId);
            }
        }
    }
}

void ActKI_setOutputDiffMsgCallBack(DiffWalkOutputMsgCallBackDef callBack)
{
    gActKiCtrlParam.diffWalkOutputMsgCallBack = callBack;
}

void ActKI_OutputMsg(void)
{
    if (gActKiCtrlParam.diffWalkOutputMsgCallBack != NULL)
    {
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            gActKiCtrlParam.outputMsg.actMsg[i].status = gActKiCtrlParam.drive[i].status.val;
            gActKiCtrlParam.outputMsg.actMsg[i].error1 = gActKiCtrlParam.drive[i].errorCode1.val;
            gActKiCtrlParam.outputMsg.actMsg[i].error2 = gActKiCtrlParam.drive[i].errorCode2.val;
            gActKiCtrlParam.outputMsg.actMsg[i].motorSta.staReadyPowerOn = gActKiCtrlParam.drive[i].staReadyPowerOn;
            gActKiCtrlParam.outputMsg.actMsg[i].motorSta.staSwitchOn = gActKiCtrlParam.drive[i].staSwitchOn;
            gActKiCtrlParam.outputMsg.actMsg[i].motorSta.staOpertEnable = gActKiCtrlParam.drive[i].staOpertEnable;
            gActKiCtrlParam.outputMsg.actMsg[i].motorSta.staFault = gActKiCtrlParam.drive[i].staFault;
            gActKiCtrlParam.outputMsg.actMsg[i].motorSta.emStop = gActKiCtrlParam.drive[i].staEmStop;
            gActKiCtrlParam.outputMsg.actMsg[i].realPos = gActKiCtrlParam.axisData[i].realPos;
            gActKiCtrlParam.outputMsg.actMsg[i].realRpm = gActKiCtrlParam.axisData[i].realRpm;
            memcpy(gActKiCtrlParam.outputMsg.actMsg[i].swVer, gActKiCtrlParam.drive[i].swVer, sizeof(gActKiCtrlParam.drive[i].swVer));
            gActKiCtrlParam.outputMsg.actMsg[i].motorCurrent = gActKiCtrlParam.drive[i].current;
            gActKiCtrlParam.outputMsg.actMsg[i].motorVolt = gActKiCtrlParam.drive[i].voltage;
            gActKiCtrlParam.outputMsg.actMsg[i].temperature = gActKiCtrlParam.drive[i].temperature;
        }

        gActKiCtrlParam.outputMsg.realSpeedData.velX = gActKiCtrlParam.realVelX;
        gActKiCtrlParam.outputMsg.realSpeedData.velY = gActKiCtrlParam.realVelY;
        gActKiCtrlParam.outputMsg.realSpeedData.velW = gActKiCtrlParam.realVelW;
        gActKiCtrlParam.outputMsg.cmdSpeedData.velX = gActKiCtrlParam.cmdVelX;
        gActKiCtrlParam.outputMsg.cmdSpeedData.velY = gActKiCtrlParam.cmdVelY;
        gActKiCtrlParam.outputMsg.cmdSpeedData.velW = gActKiCtrlParam.cmdVelW;

        gActKiCtrlParam.outputMsg.runState = (uint8_t)gActKiCtrlParam.runStatus;
        gActKiCtrlParam.outputMsg.statusTick = gActKiCtrlParam.tickCount;
        gActKiCtrlParam.outputMsg.emStop = gActKiCtrlParam.emergentStop;

        gActKiCtrlParam.diffWalkOutputMsgCallBack(gActKiCtrlParam.outputMsg);
    }
}

void ActKI_setStatus(WalkCtrlRunState_def status)
{
    gActKiCtrlParam.runStatus = status;
    gActKiCtrlParam.step = 0;
    gActKiCtrlParam.tickCount = 0;
    gActKiCtrlParam.nextStatusStartTick = HAL_GetTick();
}

void ActKI_ReleaseBrakeCtrl(uint8_t enable)
{
    SdoFrameType_def txSdo = {0};
    SdoFrameType_def rxSdo = {0};

    if (enable)
    {
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            // 先让电机掉使能然后再使能抱闸
            ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_CLEAR_FAULT_CMD);
            ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
            osDelay(1);

            txSdo.node_id = gActKiCtrlParam.drive[i].nodeId;
            txSdo.index = (uint16_t)(KI_MOTOR_APPENDIX_INDEX >> 8);
            txSdo.subIndex = (uint8_t)KI_MOTOR_APPENDIX_INDEX;
            txSdo.cmd = SDO_WRITE_1_BYTE_CMD;
            txSdo.dataLen = 1;
            txSdo.dataBuf[0] = 0x03; // 切除抱闸电源命令

            OpenCan_transSDO(ACTUATOR_CAN_PORT, txSdo, &rxSdo, 10);
            if (rxSdo.cmd != SDO_RSP_WRITE_OPERT_CMD)
            {
                ACT_ERR("send free brake ctrl cmd error(rxSdo.cmd=0x%02x)!", rxSdo.cmd);
            }
            else
            {
                ACT_DEBUG("send free brake ctrl cmd OK!");
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            txSdo.node_id = gActKiCtrlParam.drive[i].nodeId;
            txSdo.index = (uint16_t)(KI_MOTOR_APPENDIX_INDEX >> 8);
            txSdo.subIndex = (uint8_t)KI_MOTOR_APPENDIX_INDEX;
            txSdo.cmd = SDO_WRITE_1_BYTE_CMD;
            txSdo.dataLen = 1;
            txSdo.dataBuf[0] = 0x01; // 恢复抱闸控制

            OpenCan_transSDO(ACTUATOR_CAN_PORT, txSdo, &rxSdo, 10);
            if (rxSdo.cmd != SDO_RSP_WRITE_OPERT_CMD)
            {
                ACT_ERR("send rstore brake ctrl cmd error(rxSdo.cmd=0x%02x)!", rxSdo.cmd);
            }
            else
            {
                ACT_DEBUG("send rstore brake ctrl cmd OK!");
            }
        }
    }
}

void ActKI_SyncData(void)
{
    static uint32_t lastSendTick = 0;

    if (gActKiCtrlParam.runStatus != eWALK_CTRL_STA_INIT && gActKiCtrlParam.runStatus != eWALK_CTRL_STA_BOOTUP)
    {
        if (HAL_GetTick() - lastSendTick >= KI_DIFF_SYNC_ACT_DATA_PERIOD_MS)
        {
            lastSendTick = HAL_GetTick();
            OpenCan_transSYNC(ACTUATOR_CAN_PORT);
        }
    }
}

void ActKI_SendCmdSpeed(void)
{
    float cmdRpm = 0.0f;
    float absRpm = 0.0f;
    int signRpm = 0;

    gActKiCtrlParam.axisData[0].cmdSpeed = gActKiCtrlParam.cmdVelX - gActKiCtrlParam.cmdVelW * gActKiCtrlParam.pCfgParam->item.wheelDist_y / 2.0f;
    gActKiCtrlParam.axisData[1].cmdSpeed = gActKiCtrlParam.cmdVelX + gActKiCtrlParam.cmdVelW * gActKiCtrlParam.pCfgParam->item.wheelDist_y / 2.0f;

    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        gActKiCtrlParam.axisData[i].cmdRpm = Act_SpeedToRpm(gActKiCtrlParam.axisData[i].cmdSpeed,
                                                            gActKiCtrlParam.encResolu, gActKiCtrlParam.pCfgParam->item.diffWhlTickMeter[i]);
        cmdRpm = gActKiCtrlParam.axisData[i].cmdRpm;
        absRpm = SuperMath_Abs(cmdRpm);
        signRpm = SuperMath_Sign(cmdRpm);

        if (signRpm == 0)
        {
            if (gActKiCtrlParam.pCfgParam->item.RpmFilterThresh > 0)
            {
                ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
            }
            else
            {
                if (!SuperMath_FloatEqual(gActKiCtrlParam.axisData[i].realRpm, 0))
                {
                    ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
                }
            }
        }
        else
        {
            if (absRpm < gActKiCtrlParam.pCfgParam->item.minRpm)
            {
                cmdRpm = gActKiCtrlParam.pCfgParam->item.minRpm * signRpm;
            }

            if (absRpm > gActKiCtrlParam.pCfgParam->item.maxRpm)
            {
                cmdRpm = gActKiCtrlParam.pCfgParam->item.maxRpm * signRpm;
            }

            ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, cmdRpm);
        }
    }
}

/* 设置左/右轮行走1m脉冲值*/
void ActKI_setTickMeter(uint8_t nodeId, uint32_t tickMeter)
{
    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
    {
        if (nodeId == gActKiCtrlParam.drive[i].nodeId)
        {
            gActKiCtrlParam.pCfgParam->item.diffWhlTickMeter[i] = tickMeter;
            break;
        }
    }
}

/* 反馈当前左/右轮行走1m脉冲值*/
uint32_t ActKI_getTickMeter(uint8_t nodeId)
{
    float tickMeter = 0;

    if (nodeId == DIFF_WALK_LEFT_NODE_ID)
    {
        tickMeter = gActKiCtrlParam.pCfgParam->item.diffWhlTickMeter[0];
    }
    else if (nodeId == DIFF_WALK_RIGHT_NODE_ID)
    {
        tickMeter = gActKiCtrlParam.pCfgParam->item.diffWhlTickMeter[1];
    }

    return tickMeter;
}

void ActKI_RunStatusProcess(void)
{
    /*急停或者待复位状态，除了INIT，BOOTUP状态，其他都不处理 */
    if (gActKiCtrlParam.emergentStop       // 急停状态
        || gActKiCtrlParam.emcRecoverFlag) // 复位状态
    {
        if (gActKiCtrlParam.runStatus != eWALK_CTRL_STA_INIT && gActKiCtrlParam.runStatus != eWALK_CTRL_STA_BOOTUP)
        {
            return;
        }
    }

    switch (gActKiCtrlParam.runStatus)
    {
    case eWALK_CTRL_STA_INIT: // 0x00 初始化状态
    {
        uint8_t checkPass = 1;

        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            if (gActKiCtrlParam.drive[i].nodeId == 0x00)
            {
                checkPass = 0;
                break;
            }
        }

        if (checkPass)
        {
            ActKI_setStatus(eWALK_CTRL_STA_BOOTUP);
            ACT_INFO("turn to bootup mode!");
        }
    }
    break;

    case eWALK_CTRL_STA_BOOTUP: // 01 节点启动状态
    {
        uint8_t ret = 0;

        if (gActKiCtrlParam.step == 0)
        {
            static uint8_t index = 0;
            static uint8_t isSetPdoCfgFinish = 0;

            if (isSetPdoCfgFinish == 0)
            {
                if (gActKiCtrlParam.tickCount % 100 == 0)
                {
                    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                    {
                        ret = Act_SendPdoCfg(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, pWALK_KI_PdoCfgTable[index]);
                        if (ret)
                        {
                            ACT_ERR("Set PdoCfg table (index=%d) fail(id=%d)!", index, gActKiCtrlParam.drive[i].nodeId);
                            gActKiCtrlParam.tickCount = 0;
                            break;
                        }
                        else
                        {
                            ACT_INFO("Set PdoCfg table (index=%d) OK(id=%d)!", index, gActKiCtrlParam.drive[i].nodeId);
                        }
                    }
                    if (!ret)
                    {
                        index++;
                        if (index >= WALK_KI_PDO_CFT_TABLE_ITEMS)
                        {
                            index = 0;
                            isSetPdoCfgFinish = 1;
                            gActKiCtrlParam.tickCount = 0;
                            ACT_INFO("Set PdoCfg table done!");
                        }
                    }
                }
            }
            else
            {
                if (gActKiCtrlParam.tickCount == 100)
                {
                    /*pdo配置参数不进行驱动器flash存储*/
                    // for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                    // {
                    //     ret = OpenCan_WriteSDO(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, KI_SAVE_CTRL_PARAM_INDEX, 0x01, 1, 10);
                    //     if (ret)
                    //     {
                    //         gActKiCtrlParam.tickCount = 0;
                    //         ACT_ERR("Actuator(id=%d) save ctrl param cmd Fail(ret=%d)!", gActKiCtrlParam.drive[i].nodeId, ret);
                    //     }
                    //     else
                    //     {
                    //         ACT_DEBUG("Actuator(id=%d) save ctrl param cmd Success!", gActKiCtrlParam.drive[i].nodeId);
                    //     }
                    // }
                }
                else if (gActKiCtrlParam.tickCount == 1000)
                {
                    gActKiCtrlParam.step = 1;
                    gActKiCtrlParam.tickCount = 0;
                    ACT_INFO("Bootup mode: Turn to step = 1!");
                }
            }
        }
        else if (gActKiCtrlParam.step == 1)
        {
            if (gActKiCtrlParam.tickCount == 100)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    OpenCan_transNodeProtect(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId); // 询问节点状态
                }
            }
            else if (gActKiCtrlParam.tickCount == 200)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (gActKiCtrlParam.drive[i].nodeState == NODE_STATE_PRE_OPERATION)
                    {
                        OpenCan_transNMT(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, NMT_CMD_START_REMOTE_NODE); // start node
                    }
                    else if (gActKiCtrlParam.drive[i].nodeState == NODE_STATE_STOPED)
                    {
                        OpenCan_transNMT(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, NMT_CMD_ENTER_PRE_OPERAT_STATE); // 节点停止，则进入预操作模式
                    }
                    else if (gActKiCtrlParam.drive[i].nodeState == NODE_STATE_OPERATION)
                    {
                        // operation
                    }
                    else
                    {
                        // none
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 300)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (gActKiCtrlParam.drive[i].nodeState != NODE_STATE_OPERATION)
                    {
                        OpenCan_transNodeProtect(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount >= 400)
            {
                uint8_t checkpass = 1;

                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (gActKiCtrlParam.drive[i].nodeState != NODE_STATE_OPERATION)
                    {
                        checkpass = 0;
                        break;
                    }
                }

                if (checkpass)
                {
                    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                    {
                        if (gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.bootupTimeout)
                        {
                            gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.bootupTimeout = 0;
                        }
                    }
                    ActKI_setStatus(eWALK_CTRL_STA_STARTUP);
                    ACT_INFO("Bootup mode finish, turn to start up mode!");
                    break;
                }
                else
                {
                    if (gActKiCtrlParam.tickCount >= 1000)
                    {
                        gActKiCtrlParam.tickCount = 0;
                    }
                }
            }
        }

        if (HAL_GetTick() - gActKiCtrlParam.nextStatusStartTick > DIFF_KI_BOOTUP_TIMEOUT_MS)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (gActKiCtrlParam.drive[i].nodeState != NODE_STATE_OPERATION)
                {
                    if (!gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.bootupTimeout)
                    {
                        ACT_ERR("Actuator(id=%d) bootup timeout!", gActKiCtrlParam.drive[i].nodeId);
                        gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.bootupTimeout = 1;
                    }
                }
            }
        }
    }
    break;

    case eWALK_CTRL_STA_STARTUP: // 02 启动状态
    {
        uint8_t ret = 0;

        if (gActKiCtrlParam.step == 0)
        {
            if (gActKiCtrlParam.tickCount == 100)
            {
                ret = OpenCan_ReadSDO(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[0].nodeId, KI_ENC_RESOLUT_INDEX, &gActKiCtrlParam.encResolu, 10);
                if (!ret)
                {
                    ACT_INFO("Read motor encoder resoution OK(motor encoder resoution=%d)!", gActKiCtrlParam.encResolu);
                }
                else
                {
                    ACT_ERR("Read motor encoder resoution error!");
                    gActKiCtrlParam.tickCount = 0;
                }
            }
            else if (gActKiCtrlParam.tickCount == 200)
            {
                // Read SW Version
                SdoFrameType_def txSdo = {0};
                SdoFrameType_def rxSdo = {0};

                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    txSdo.node_id = gActKiCtrlParam.drive[i].nodeId;
                    txSdo.index = (uint16_t)(KI_READ_SW_VER_INDEX >> 8);
                    txSdo.subIndex = (uint8_t)(KI_READ_SW_VER_INDEX);
                    txSdo.cmd = SDO_READ_BYTE_CMD;
                    txSdo.dataLen = 0;

                    OpenCan_transSDO(ACTUATOR_CAN_PORT, txSdo, &rxSdo, 10);
                    if (rxSdo.cmd != SDO_RSP_4_BYTE_CMD)
                    {
                        ACT_ERR("Read motor Simplified SW version fail(id=%d)!", gActKiCtrlParam.drive[i].nodeId);
                    }
                    else
                    {
                        Hex_to_Str(gActKiCtrlParam.drive[i].swVer, rxSdo.dataBuf, rxSdo.dataLen);
                        ACT_INFO("Read motor Simplified SW version:%s (id=%d)!", gActKiCtrlParam.drive[i].swVer, gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 300)
            {
                Int tmp = {0};

                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    tmp.UnsignedValue = 0;
                    ret = OpenCan_ReadSDO(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, KI_MAX_CURRET_INDEX, &tmp.UnsignedValue, 10);
                    if (ret)
                    {
                        gActKiCtrlParam.drive[i].Ipeak_A = (float)(tmp.UnsignedValue) / 10.0f;
                        ACT_DEBUG("Read Ipeak_Dec OK(id = %d, Ipeak_Dec=%0.2f)!", gActKiCtrlParam.drive[i].nodeId, gActKiCtrlParam.drive[i].Ipeak_A);
                    }
                    else
                    {
                        ACT_ERR("read Ipeak_Dec error(id=%d)!", gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 400)
            {
                Int temp = {0};

                /* 配置急停减速度(rps) */
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (gActKiCtrlParam.pCfgParam->item.emStopDec == 0)
                    {
                        temp.UnsignedValue = Kinco_ACC_Rps2Dec(KI_EM_STOP_DEFAULT_DEC_RPS, gActKiCtrlParam.encResolu);
                        gActKiCtrlParam.emStopDecRps = KI_EM_STOP_DEFAULT_DEC_RPS;
                    }
                    else
                    {
                        temp.UnsignedValue = Kinco_ACC_Rps2Dec(gActKiCtrlParam.pCfgParam->item.emStopDec, gActKiCtrlParam.encResolu);
                        gActKiCtrlParam.emStopDecRps = gActKiCtrlParam.pCfgParam->item.emStopDec;
                    }

                    ret = OpenCan_WriteSDO(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, KI_FAST_STOP_DEC_INDEX, temp.UnsignedValue, 4, 10);
                    if (ret)
                    {
                        gActKiCtrlParam.tickCount = 0;
                        ACT_ERR("Set fast stop dec rps error error!(id=%d)", gActKiCtrlParam.drive[i].nodeId);
                    }
                    else
                    {
                        ACT_DEBUG("Set fast stop dec rps = %f  OK!(id=%d)", gActKiCtrlParam.emStopDecRps, gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 500)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    ret = OpenCan_WriteSDO(ACTUATOR_CAN_PORT, gActKiCtrlParam.drive[i].nodeId, KI_WORK_MODE_INDEX, KI_WORK_MODE_IMM_SPEED, 1, 10);
                    if (ret)
                    {
                        gActKiCtrlParam.tickCount = 0;
                        ACT_ERR("set speed mode fail((id=%d))!", gActKiCtrlParam.drive[i].nodeId);
                    }
                    else
                    {
                        ACT_DEBUG("set speed mode OK(id=%d)!", gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 600)
            {
                gActKiCtrlParam.step = 1;
                gActKiCtrlParam.tickCount = 0;
                ACT_INFO("Starutup mode: turn to step = 1;");
            }
        }
        else if (gActKiCtrlParam.step == 1)
        {
            if (gActKiCtrlParam.tickCount == 100)
            {
                // 清除错误
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
                    {
                        ACT_ERR("start up mode:Clear Fault(id = %d, error1=0x%04x, error2=0x%04x, status=0x%04x)",
                                gActKiCtrlParam.drive[i].nodeId, gActKiCtrlParam.drive[i].errorCode1, gActKiCtrlParam.drive[i].errorCode2, gActKiCtrlParam.drive[i].status);

                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_CLEAR_FAULT_CMD);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount == 200)
            {
                // 电机断电
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
                }
            }
            else if (gActKiCtrlParam.tickCount == 300)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
                }
            }
            else if (gActKiCtrlParam.tickCount == 400)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    // 电机上电
                    if (!gActKiCtrlParam.drive[i].staOpertEnable)
                    {
                        ACT_DEBUG("start up mode: send  power on cmd (id = %d)!", gActKiCtrlParam.drive[i].nodeId);
                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_ON);
                    }
                }
            }
            else if (gActKiCtrlParam.tickCount > 500)
            {
                uint8_t checkPass = 1;

                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
                    {
                        checkPass = 0;
                        break;
                    }
                }
                if (checkPass)
                {
                    for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                    {
                        if (gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.startupTimeout)
                        {
                            gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.startupTimeout = 0;
                        }
                    }

                    ActKI_setStatus(eWALK_CTRL_STA_NORMAL);
                    ACT_INFO("Startup finish, turn to normal mode!");
                    break;
                }
                else // 上电失败，重新上电
                {
                    if (gActKiCtrlParam.tickCount > 1000)
                    {
                        ACT_ERR("motor Power on fail!");
                        gActKiCtrlParam.tickCount = 0;
                    }
                }
            }
        }

        // 监控startup是否超时
        if (HAL_GetTick() - gActKiCtrlParam.nextStatusStartTick > KINCO_STARTUP_TIMEOUT_MS)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (!gActKiCtrlParam.drive[i].staOpertEnable)
                {
                    if (!gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.startupTimeout)
                    {
                        gActKiCtrlParam.outputMsg.actMsg[i].runErr.bit.startupTimeout = 1;
                        ACT_ERR("actutor(id=%d) startup timeout!", gActKiCtrlParam.drive[i].nodeId);
                    }
                }
            }
        }
    }
    break;

    case eWALK_CTRL_STA_NORMAL: // 03 正常状态
    {
        uint8_t nodeFault = 0;

        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
            {
                ACT_ERR("Normal mode:report error1 = 0x%04x, error2 = 0x%04x, status= 0x%04x(id = %d)",
                        gActKiCtrlParam.drive[i].errorCode1.val, gActKiCtrlParam.drive[i].errorCode2.val,
                        gActKiCtrlParam.drive[i].status.val, gActKiCtrlParam.drive[i].nodeId);

                nodeFault = 1;
                break;
            }
        }
        if (nodeFault)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                // 有错误，所有零速停止
                ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
            }
            ActKI_setStatus(eWALK_CTRL_STA_ERROR);
            ACT_INFO("Normal mode turn to error mode!");
            break;
        }
        else
        {
            ActKI_SendCmdSpeed();
        }
    }
    break;

    case eWALK_CTRL_STA_RESET: // 04 复位状态
    {
        if (gActKiCtrlParam.tickCount == 50)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                ActKI_sendRpm(gActKiCtrlParam.drive[i].nodeId, 0);
            }
        }
        else if (gActKiCtrlParam.tickCount == 100)
        {
            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
                {
                    ACT_ERR("ResetMode: Node report fault(id = %d), status=0x%04x, errorCode1=0x%04x, errorCode2=0x%04x,",
                            gActKiCtrlParam.drive[i].nodeId, gActKiCtrlParam.drive[i].status.val, gActKiCtrlParam.drive[i].errorCode1.val,
                            gActKiCtrlParam.drive[i].errorCode2.val);

                    ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_OFF);
                    ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_CLEAR_FAULT_CMD);
                }
            }
        }
        else if (gActKiCtrlParam.tickCount == 200)
        {
            uint8_t checkFlag = 1;

            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (gActKiCtrlParam.drive[i].staFault)
                {
                    checkFlag = 0;
                    break;
                }
            }
            if (checkFlag)
            {
                for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
                {
                    if (!gActKiCtrlParam.drive[i].staOpertEnable)
                    {
                        ACT_DEBUG("ResetMode: send power on cmd (id = %d)", gActKiCtrlParam.drive[i].nodeId);
                        ActKI_ctrlCmd(gActKiCtrlParam.drive[i].nodeId, KI_SERVO_POWER_ON);
                    }
                }
            }
            else
            {
                gActKiCtrlParam.tickCount = 0;
            }
        }
        else if (gActKiCtrlParam.tickCount >= 500)
        {
            uint8_t checkPass = 1;

            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
                {
                    checkPass = 0;
                    break;
                }
            }

            if (checkPass)
            {
                ActKI_setStatus(eWALK_CTRL_STA_NORMAL);
                ACT_INFO("Reset mode turn to normal mode!");
            }
            else
            {
                ActKI_setStatus(eWALK_CTRL_STA_ERROR);
                ACT_INFO("Reset mode turn to error mode!");
            }
        }
    }
    break;

    case eWALK_CTRL_STA_ERROR: // 05-错误处理状态
    {
        uint8_t checkFlag = 1;

        for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
        {
            if (ActKI_checkNodeFault(gActKiCtrlParam.drive[i].nodeId))
            {
                checkFlag = 0;
                break;
            }
        }

        if (checkFlag)
        {
            ActKI_setStatus(eWALK_CTRL_STA_NORMAL);
            ACT_INFO("Error mode turn to normal mode!");
        }
        else
        {
            uint8_t seriousFaultFlag = 0;

            for (uint8_t i = 0; i < DIFF_WALK_NODE_COUNT; i++)
            {
                if (gActKiCtrlParam.drive[i].errorCode1.val || gActKiCtrlParam.drive[i].errorCode2.val)
                {
                    if (Kinco_IsSeriousFault(gActKiCtrlParam.drive[i].errorCode1, gActKiCtrlParam.drive[i].errorCode2))
                    {
                        seriousFaultFlag = 1;
                    }
                }

                if (gActKiCtrlParam.drive[i].nodeState != NODE_STATE_OPERATION)
                {
                    seriousFaultFlag = 1;
                }
            }

            if (!seriousFaultFlag || gActKiCtrlParam.pCfgParam->item.ctrlParam.bit.isAutoClearErr)
            {
                ActKI_setStatus(eWALK_CTRL_STA_RESET);
                ACT_INFO("Error mode turn to reset mode!");
            }
            else
            {
                // do nothing, maintain error mode
                if (gActKiCtrlParam.tickCount >= 3000)
                {
                    gActKiCtrlParam.tickCount = 0;
                    ACT_INFO("maintain error mode!");
                }
            }
        }
    }
    break;

    default:
    {
    }
    break;
    }
}

void ActKI_processLoop(void)
{
    static uint32_t lastProcessTicket = 0;

    if (!gActKiCtrlParam.enable)
    {
        return;
    }

    ActKI_OutputMsg();
    ActKI_SyncData();
    if (HAL_GetTick() - lastProcessTicket >= KI_DIFF_PROCESS_LOOP_TIME_MS)
    {
        lastProcessTicket = HAL_GetTick();
        gActKiCtrlParam.tickCount += KI_DIFF_PROCESS_LOOP_TIME_MS;
        ActKI_EmStopProcess();
        ActKI_RequestNodeState();
        ActKI_CmdIntervalProcess();
        ActKI_RunStatusProcess();
    }
}
