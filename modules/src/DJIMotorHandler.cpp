#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"

constexpr float MOTOR_2PI = 6.283185307f;

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
    if((hcan != &hcan1 && hcan != &hcan2) || canId < 0x201 || canId > 0x208 || motor == nullptr)return;
    uint8_t id = hcan == &hcan1 ? 0 : 1;
    DJIMotorList[id][canId - 0x201] = motor;
    motor->hcan = hcan;
    motor->canId = canId;
    if(id == 0){
        if(0x201 <= canId && canId <= 0x204)
            CAN1_0x200_Exist = true;
        else
            CAN1_0x1FF_Exist = true;
    }
    else{
        if(0x201 <= canId && canId <= 0x204)
            CAN2_0x200_Exist = true;
        else
            CAN2_0x1FF_Exist = true;
    }
}

static void PackCurrentFrame(DJIMotor *list[8], uint8_t *frame0, uint8_t *frame1){
    for(uint8_t i = 0;i < 8;i++){
        if(list[i] == nullptr)continue;
        if(i < 4){
            frame0[2 * i] = list[i]->currentSet >> 8;
            frame0[2 * i + 1] = list[i]->currentSet & 0xFF;
        }
        else{
            frame1[2 * (i - 4)] = list[i]->currentSet >> 8;
            frame1[2 * (i - 4) + 1] = list[i]->currentSet & 0xFF;
        }
    }
}

void DJIMotorHandler::sendControlData()
{
    // 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
    for(uint8_t i = 0;i < 8;i++){
        can1_send_data_0[i] = 0;
        can1_send_data_1[i] = 0;
        can2_send_data_0[i] = 0;
        can2_send_data_1[i] = 0;
    }

    PackCurrentFrame(DJIMotorList[0],can1_send_data_0,can1_send_data_1);
    PackCurrentFrame(DJIMotorList[1],can2_send_data_0,can2_send_data_1);
    
    if(CAN1_0x200_Exist)CAN_Transmit(&hcan1,0x200,can1_send_data_0,8);
    if(CAN1_0x1FF_Exist)CAN_Transmit(&hcan1,0x1FF,can1_send_data_1,8);
    if(CAN2_0x200_Exist)CAN_Transmit(&hcan2,0x200,can2_send_data_0,8);
    if(CAN2_0x1FF_Exist)CAN_Transmit(&hcan2,0x1FF,can2_send_data_1,8);
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    // 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
    if(hcan != &hcan1 && hcan != &hcan2)return;
    if(index < 0 || index > 7)return;
    if(rx_data == nullptr)return;
    int id = hcan == &hcan1 ? 0 : 1;
    DJIMotor *motor = DJIMotorList[id][index];
    if(motor != nullptr && motor->hcan == hcan && motor->canId == 0x201 + index){
        UpdateSensorData(motor,rx_data);
        motor->AliveFlag++;
    }
}

static float GetGearRatio(GearBox gearbox){
    switch(gearbox){
        case GearBox_M2006:
            return 36.0f;
        default:
            return 1.0f;
    }
}

void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    // 从 CAN 数据解析编码器、转速、电流和温度。
    // 处理编码器回绕，并按减速比换算输出轴位置和角速度
    DJIMotor::MotorFeedBack *fb = &motor->motorFeedback;

    fb->last_ecd = fb->ecd;
    fb->ecd = can_data[0] << 8 | can_data[1];

    int16_t diff = fb->ecd - fb->last_ecd;
    if(diff > 4096)diff -= 8192;
    else if(diff < -4096)diff += 8192;

    fb->speed_rpm = (int16_t)(can_data[2] << 8 | can_data[3]);
    fb->currentFdb = (float)(int16_t)(can_data[4] << 8 | can_data[5]);
    fb->temperatureFdb = can_data[6];

    float ratio = GetGearRatio(motor->gearBox);
    fb->positionFdb += (float)diff / 8192.0f * MOTOR_2PI / ratio;
    fb->speedFdb = (float)fb->speed_rpm / 60.0f * MOTOR_2PI / ratio;

}

void DJIMotorHandler::AllMotorAliveCheck()
{
    // 遍历已注册电机并调用 AliveCheck。
    for(int i = 0;i < 2;i++){
        for(int j = 0;j < 8;j++){
            if(DJIMotorList[i][j] == nullptr)continue;
            DJIMotorList[i][j]->AliveCheck();
        }
    }
}
