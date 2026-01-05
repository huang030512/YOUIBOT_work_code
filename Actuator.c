/**
 ******************************************************************************
 * @file    Actuator.c
 * @author
 * @version V1.0.0
 * @date
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "Actuator.h"
// #include "ActuatorTtDiff.h"
#include "ActuatorKincoDiff.h"
#include "KincoCmd.h"
// #include "Actuator_Ki_Hp_FourWheel2.h"
// #include "ActuatorKincoSteer2.h"
#include "ActuatorKincoDiff2.h"
// #include "ActuatorCurtisSteer.h"
 #include "ActuatorKincoSingleAxis.h"
 #include "ActuatorAKSingleAxis.h"
// #include "ActuatorAkFourWheel4.h"
// #include "ActuatorKincoDiff4.h"
// #include "ActuatorVeichiDiff.h"

__attribute__((section(".ccmram"))) ActDrive_def gActDrive;

void Actuator_Init(pWalkModelCfgParam_def pCfgParam)
{
	if (pCfgParam == NULL)
	{
		return;
	}
	
    gActDrive.enable = 0;
    gActDrive.brand = (ActuatorBrand_def)pCfgParam->item.modelParam.item.actBrand;
    gActDrive.type = (ActuatorType_def)pCfgParam->item.modelParam.item.walkType;
    gActDrive.Init = NULL;
    gActDrive.SetRpm = NULL;
    gActDrive.WalkCtrl = NULL;
    gActDrive.OptCtrl = NULL;
    gActDrive.setParkState = NULL;
    gActDrive.SetOutputDiffMsgCallBack = NULL;
    gActDrive.SetOutputSteer2MsgCallBack = NULL;
    gActDrive.SetOutputFourWheel2MsgCallBack = NULL;
    gActDrive.SetDiffTickMeter = NULL;
    gActDrive.GetDiffTickMeter = NULL;
    gActDrive.ProcessLoop = NULL;

    if (gActDrive.type == ACT_TYPE_DIFF)
    {

        if (gActDrive.brand == ACT_BR_KINCO)
        {
            gActDrive.enable = 1;
            gActDrive.Init = ActKI_Init;
            gActDrive.SetRpm = ActKI_setRpm;
            gActDrive.WalkCtrl = ActKI_WalkCtrl;
            gActDrive.OptCtrl = ActKI_OptCtrl;
            gActDrive.SetOutputDiffMsgCallBack = ActKI_setOutputDiffMsgCallBack;
            gActDrive.SetDiffTickMeter = ActKI_setTickMeter;
            gActDrive.GetDiffTickMeter = ActKI_getTickMeter;

            gActDrive.ProcessLoop = ActKI_processLoop;

            gActDrive.Init(pCfgParam);
        }
#if 0 // 有项目上线再启用
        		else if (gActDrive.brand == ACT_BR_VEICHI)
        		{
        			gActDrive.enable = 1;
        			gActDrive.Init = ActVeichi_Init;
        			gActDrive.SetRpm = ActVei_setRpm;
        			gActDrive.WalkCtrl = ActVei_WalkCtrl;
        			gActDrive.OptCtrl = ActVei_OptCtrl;
        			gActDrive.SetOutputDiffMsgCallBack = ActVei_setOutputDiffMsgCallBack;
        			gActDrive.SetDiffTickMeter = ActVeichi_setTickMeter;
        			gActDrive.GetDiffTickMeter = ActVeichi_getTickMeter;
        			gActDrive.ProcessLoop = ActVei_processLoop;

        			gActDrive.Init(cfgParam);
        		}
#endif
    }
    //    else if (gActDrive.type == ACT_TYPE_STEER)
    //    {
    //        if (gActDrive.brand == ACT_BR_CURTIS)
    //        {
    //            gActDrive.enable = 1;
    //            gActDrive.Init = CurtisSteer_Init;
    //            gActDrive.WalkCtrl = CurtisSteer_WalkCtrl;
    //            gActDrive.SetOutputSteerMsgCallBack = CurtisSteer_SetOutputMsgCallBack;
    //            gActDrive.ProcessLoop = CurtisSteer_Loop;
    //            gActDrive.OptCtrl = CurtisSteer_OptCtrl;

    //            gActDrive.Init(cfgParam);
    //        }
    //    }
    //    else if (gActDrive.type == ACT_TYPE_STEER2)
    //    {
    //        if (gActDrive.brand == ACT_BR_HINSON)
    //        {
    //        }
    //        else if (gActDrive.brand == ACT_BR_KINCO)
    //        {
    //            gActDrive.enable = 1;
    //            gActDrive.Init = KiSteer2_Init;
    //            gActDrive.SetRpm = NULL;
    //            gActDrive.WalkCtrl = KiSteer2_WalkCtrl;
    //            gActDrive.OptCtrl = KiSteer2_OptCtrl;
    //            gActDrive.SetOutputSteer2MsgCallBack = KiSteer2_setOutputMsgCallBack;
    //            gActDrive.ProcessLoop = KiSteer2_processLoop;
    //            gActDrive.Init(cfgParam);
    //        }
    //    }
    //    else if (gActDrive.type == ACT_TYPE_FourWheel2)
    //    {
    //        if (gActDrive.brand == ACT_BR_KINCO_HONGPAN)
    //        {
    //            gActDrive.enable = 1;
    //            gActDrive.Init = KiHpFourWheel2_Init;
    //            gActDrive.SetRpm = KiHpFourWheel2_sendSpeed;
    //            gActDrive.WalkCtrl = KiHpFourWheel2_WalkCtrl;
    //            gActDrive.setParkState = KiHpFourWheel2_SetParkState;
    //            gActDrive.SetOutputFourWheel2MsgCallBack = KiHpFourWheel2_setOutputMsgCallBack;
    //            gActDrive.ProcessLoop = KiHpFourWheel2_processLoop;
    //            gActDrive.OptCtrl = KiHpFourWheel2_OptCtrl;

    //            gActDrive.Init(cfgParam);
    //        }
    //    }
    //    else if (gActDrive.type == ACT_TYPE_FourWheel4)
    //    {
    //        if (gActDrive.brand == ACT_BR_AK)
    //        {
    //            gActDrive.enable = 1;
    //            gActDrive.Init = AK_FourWheel4_Init;
    //            gActDrive.SetRpm = AK_FourWheel4_sendSpeed;
    //            gActDrive.WalkCtrl = AK_FourWheel4_WalkCtrl;
    //            gActDrive.setParkState = AK_FourWheel4_SetParkState;
    //            gActDrive.SetOutputFourWheel4MsgCallBack = AK_FourWheel4_setOutputMsgCallBack;
    //            gActDrive.ProcessLoop = AK_FourWheel4_processLoop;
    //            gActDrive.OptCtrl = AK_FourWheel4_OptCtrl;

    //            gActDrive.Init(cfgParam);
    //        }
    //    }
    else if (gActDrive.type == ACT_TYPE_DIFF2)
    {
        if (gActDrive.brand == ACT_BR_KINCO)
        {
            gActDrive.enable = 1;
            gActDrive.Init = KiDiff2_Init;
            gActDrive.SetRpm = NULL;
            gActDrive.WalkCtrl = KiDiff2_WalkCtrl;
            gActDrive.SetOutputDiff2MsgCallBack = KiDiff2_setOutputMsgCallBack;
            gActDrive.ProcessLoop = KiDiff2_processLoop;
            gActDrive.OptCtrl = KiDiff2_OptCtrl;

            gActDrive.Init(pCfgParam);
        }
    }
    //    else if (gActDrive.type == ACT_TYPE_DIFF4)
    //    {
    //        if (gActDrive.brand == ACT_BR_KINCO)
    //        {
    //            gActDrive.enable = 1;
    //            gActDrive.Init = KiDiff4_Init;
    //            gActDrive.SetRpm = NULL;
    //            gActDrive.WalkCtrl = KiDiff4_WalkCtrl;
    //            gActDrive.OptCtrl = KiDiff4_OptCtrl;
    //            gActDrive.SetOutputDiff4MsgCallBack = KiDiff4_setOutputMsgCallBack;
    //            gActDrive.ProcessLoop = KiDiff4_processLoop;

    //            gActDrive.Init(cfgParam);
    //        }
    //    }
    else if (gActDrive.type == ACT_TYPE_SINGLE_AXIS)
    {
#if 0 // 有项目上线再启用
        if (gActDrive.brand == ACT_BR_KINCO)
        {
            gActDrive.enable = 1;
            gActDrive.Init = ActKI_SingleAxis_Init;
            gActDrive.SetRpm = NULL;
            gActDrive.OptCtrl = ActKI_SingleAxis_OptCtrl;
            gActDrive.WalkCtrl = ActKI_SingleAxis_WalkCtrl;
            gActDrive.SetOutputSingleAxisMsgCallback = ActKI_SingleAxis_setOutputDiffMsgCallBack;
            gActDrive.ProcessLoop = ActKI_SingleAxis_processLoop;

            gActDrive.Init(pCfgParam);
        }
#endif
       if (gActDrive.brand == ACT_BR_AK)
       {
           gActDrive.enable = 1;
           gActDrive.Init = ActAK_SingleAxis_Init;
           gActDrive.SetRpm = NULL;
           gActDrive.OptCtrl = ActAK_SingleAxis_OptCtrl;
           gActDrive.WalkCtrl = ActAK_SingleAxis_WalkCtrl;
           gActDrive.SetOutputSingleAxisMsgCallback = ActAK_SingleAxis_setOutputMsgCallBack;
           gActDrive.ProcessLoop = ActAK_SingleAxis_processLoop;

           gActDrive.Init(pCfgParam);
       }

    }
}

void Actuator_setLogLevel(uint8_t level)
{
    gAct_log_level = level;
}

void Actuator_setRpm(uint8_t nodeId, float rpm)
{
    if (gActDrive.enable && gActDrive.SetRpm != NULL)
    {
        gActDrive.SetRpm(nodeId, rpm);
    }
}

void Actuator_WalkCtrl(float velX, float velY, float velW)
{
    if (gActDrive.enable && gActDrive.WalkCtrl != NULL)
    {
        if (gActDrive.walkLockState)
        {
            gActDrive.WalkCtrl(0, 0, 0);
        }
        else
        {
            gActDrive.WalkCtrl(velX, velY, velW);
        }
    }
}

int32_t Actuator_OptCtrl(uint8_t cmd, uint8_t *dataBuf, uint16_t dataLen)
{
    if (gActDrive.enable && gActDrive.OptCtrl != NULL)
    {
        return gActDrive.OptCtrl(cmd, dataBuf, dataLen);
    }

    return 0;
}

void Actuator_setOutputDiffMsgCallBack(DiffWalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputDiffMsgCallBack != NULL)
    {
        gActDrive.SetOutputDiffMsgCallBack(callBack);
    }
}

void Actuator_SetOutputSteerMsgCallBack(SteerWalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputSteerMsgCallBack != NULL)
    {
        gActDrive.SetOutputSteerMsgCallBack(callBack);
    }
}

void Actuator_SetOutputSteer2MsgCallBack(Steer2WalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputSteer2MsgCallBack != NULL)
    {
        gActDrive.SetOutputSteer2MsgCallBack(callBack);
    }
}

void Actuator_SetOutputDiff2MsgCallBack(Diff2WalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputDiff2MsgCallBack != NULL)
    {
        gActDrive.SetOutputDiff2MsgCallBack(callBack);
    }
}

void Actuator_SetOutputDiff4MsgCallBack(Diff4WalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputDiff4MsgCallBack != NULL)
    {
        gActDrive.SetOutputDiff4MsgCallBack(callBack);
    }
}

void Actuator_SetOutputFourWheel2MsgCallBack(FourWheel2WalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputFourWheel2MsgCallBack != NULL)
    {
        gActDrive.SetOutputFourWheel2MsgCallBack(callBack);
    }
}

void Actuator_SetOutputFourWheel4MsgCallBack(FourWheel4WalkOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputFourWheel4MsgCallBack != NULL)
    {
        gActDrive.SetOutputFourWheel4MsgCallBack(callBack);
    }
}

void Actuator_SetOutputSingAxisMsgCallBack(SingleAxisOutputMsgCallBackDef callBack)
{
    if (gActDrive.enable && gActDrive.SetOutputSingleAxisMsgCallback != NULL)
    {
        gActDrive.SetOutputSingleAxisMsgCallback(callBack);
    }
}

void Actuator_setLockWalk(uint8_t state)
{
    gActDrive.walkLockState = state;
    if (gActDrive.walkLockState && gActDrive.WalkCtrl != NULL)
    {
        gActDrive.WalkCtrl(0, 0, 0);
    }
}

void Actuator_SetParkState(uint8_t state)
{
    if (gActDrive.setParkState != NULL)
    {
        gActDrive.setParkState(state);
    }
}

/* 设置差速模型 左右轮1m脉冲值*/
void Actuator_SetDiffTickMeter(uint8_t nodeId, uint32_t tickMeter)
{
    if (gActDrive.SetDiffTickMeter != NULL)
    {
        gActDrive.SetDiffTickMeter(nodeId, tickMeter);
    }
}

/* 读取差速模型 左轮1m脉冲值*/
uint32_t Actuator_GetDiffTickMeter(uint8_t nodeId)
{

    if (gActDrive.GetDiffTickMeter != NULL)
    {
        return gActDrive.GetDiffTickMeter(nodeId);
    }

    return 0;
}

void Actuator_Loop(void)
{
    if (gActDrive.enable && gActDrive.ProcessLoop != NULL)
    {
        gActDrive.ProcessLoop();
    }
}
