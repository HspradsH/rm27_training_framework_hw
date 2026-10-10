#include "M2006.hpp"

M2006::M2006()
{
    // 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
    gearBox = GearBox_M2006;
    maxCurrent = 10000;

    speedPid = PID(3000.0f, 500.0f, 0.0f, 10000.0f, 5000.0f, PID_POSITION); ///配置速度环参数
    positionPid = PID(8.0f, 0.0f, 0.0f, 2.0f, 0.0f, PID_POSITION); 

    controlMode = RELAX_MODE;
    currentSet = 0;

}

void M2006::setOutput()
{
    // 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。
    switch(controlMode){
        case RELAX_MODE:
            speedPid.Clear();
            positionPid.Clear();
            currentSet = 0;
            break;
        case SPD_MODE:
            speedPid.ref = speedSet;
            speedPid.fdb = motorFeedback.speedFdb;
            speedPid.UpdateResult();
            currentSet = (int16_t)Numeric::LimitABS(speedPid.result,(float)maxCurrent);
            break;
        case POS_MODE:
            positionPid.ref = positionSet;
            positionPid.fdb = motorFeedback.positionFdb;
            positionPid.UpdateResult();
            speedPid.ref = positionPid.result;
            speedPid.fdb = motorFeedback.speedFdb;
            speedPid.UpdateResult();
            currentSet = (int16_t)Numeric::LimitABS(speedPid.result,(float)maxCurrent);
            break;
        default:
            currentSet = 0;
            break;
    }
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
    if(AliveFlag != Pre_AliveFlag)
        MotorState = MOTOR_ONLINE;
    else
        MotorState = MOTOR_OFFLINE;
    Pre_AliveFlag = AliveFlag;
    return MotorState;
}
