#include "BMI088.hpp"
#include "tx_api.h"
#include "bsp_pwm.hpp"
#include <cmath>

namespace BMI088
{
    /**
     * @brief BMI088 acc gyro 标定
     * @note 标定后的陀螺仪零偏存储在 Gyro_offset 中。
     * @attention 不管工作模式是blocking还是IT,标定时都是blocking模式,所以不用担心中断关闭后无法标定(RobotInit关闭了全局中断)
     * @attention 标定精度和等待时间有关。
     * @todo 将标定次数(等待时间)变为参数供设定
     * @section 整体流程为1.累加加速度数据计算gNrom()
     *                   2.累加陀螺仪数据计算零飘
     *                   3. 如果标定过程运动幅度过大,重新标定
     *                   4.保存标定参数
     */
    void cBMI088::Calibrate()
    {
        const int calib_samples = 4000; // 采样次数
        float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
        gyro_data_t temp_gyro;

        // 1. 清除旧的 Offset，防止叠加
        Gyro_offset[0] = 0.0f;
        Gyro_offset[1] = 0.0f;
        Gyro_offset[2] = 0.0f;

        // 2. 循环采样
        for (int i = 0; i < calib_samples; i++)
        {
            ReadGyroData(&temp_gyro); // 这里读取的是原始值（因为Offset已清零）
            gyro_sum[0] += temp_gyro.x;
            gyro_sum[1] += temp_gyro.y;
            gyro_sum[2] += temp_gyro.z;
            
            tx_thread_sleep(1); // 等待一个 ThreadX tick；实际耗时由 tick 频率决定。
        }

        // 3. 计算平均值作为零偏
        Gyro_offset[0] = gyro_sum[0] / calib_samples;
        Gyro_offset[1] = gyro_sum[1] / calib_samples;
        Gyro_offset[2] = gyro_sum[2] / calib_samples;
        
        // 如果零偏过大（例如超过 0.1 rad/s），可能是在运动中标定的，应报错或丢弃
        if (fabs(Gyro_offset[0]) > 0.1f || fabs(Gyro_offset[1]) > 0.1f || fabs(Gyro_offset[2]) > 0.1f)
        {
            self_test.CALIBRATE_ERR = true;
            // 恢复默认值或保留上次值
            Gyro_offset[0] = BMI088_GYRO_PRE_CALI_OFFSET_X; 
            Gyro_offset[1] = BMI088_GYRO_PRE_CALI_OFFSET_Y;
            Gyro_offset[2] = BMI088_GYRO_PRE_CALI_OFFSET_Z;
        }
        else
        {
            self_test.CALIBRATE_ERR = false;
        }
    }

    void cBMI088::TemperatureControl(float target_temp)
    {
        // 温度读取完成后，用 TempPid 计算加热占空比并限制到 [0, 1]。
        // 未完成前保持加热关闭。
        float temp;
        ReadAccTemperature(&temp);
        TempPid.ref = target_temp;
        TempPid.fdb = temp;
        TempPid.UpdateResult();
        float duty = TempPid.result / TempPid.maxOut;
        PWM_SetDutyRatio(&HEATING_RESISTANCE_TIM, duty, HEATING_RESISTANCE_CHANNEL);
        self_test.TEMP_CTRL_ERR = false;
    }

    void cBMI088::VerifyAccChipID()
    {
        uint8_t pRxData[2]; //< 读取两个字节，第一个是需要丢弃的无效字节，第二个是芯片 ID

        ReadReg(BMI088_CS_ACC, ACC_CHIP_ID_ADDR, pRxData, 2); //< 读取加速度计chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则加速度计ID错误,初始化错误
        if (pRxData[1] != ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData[1] == ACC_CHIP_ID_VAL)
        {
            self_test.ACC_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyGyroChipID()
    {
        uint8_t pRxData;                                                 //< 读取一个字节,chip id
        ReadReg(BMI088_CS_GYRO, GYRO_CHIP_ID_ADDR, &pRxData, 1); //< 读取陀螺仪chip id
        tx_thread_sleep(1);
        //< 如果chip id不等于预设值,则陀螺仪ID错误,初始化错误
        if (pRxData != GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = true;
            self_test.INIT_ERR = true;
        }
        else if (pRxData == GYRO_CHIP_ID_VAL)
        {
            self_test.GYRO_CHIP_ID_ERR = false;
        }
    }

    void cBMI088::VerifyAccData()
    {
        // 检查加速度数据是否有效，并更新 ACC_DATA_ERR。
        uint8_t buf[3];
        ReadReg(BMI088_CS_ACC,ACC_ERR_REG_ADDR,buf,2);
        if(buf[1] & 0x01){
            self_test.ACC_DATA_ERR = true;
        }
        else{
            self_test.ACC_DATA_ERR = false;
        }
    }

    void cBMI088::VerifyGyroData()
    {
        // 检查角速度数据是否有效，并更新 GYRO_DATA_ERR。
        uint8_t buf[8];
        ReadReg(BMI088_CS_GYRO,GYRO_RATE_X_LSB_ADDR,buf,6);
        int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);
        if(raw_x == 0 && raw_y == 0 && raw_z == 0){
            self_test.GYRO_DATA_ERR = true;
        }
        else{
            self_test.GYRO_DATA_ERR = false;
        }
    }

    static void CS_Low(BMI088_SENSOR cs){
        if(cs == BMI088_CS_ACC){
            HAL_GPIO_WritePin(BMI088_CS_ACC_GPIO_Port,BMI088_CS_ACC_Pin,GPIO_PIN_RESET);
        }
        else if(cs == BMI088_CS_GYRO){
            HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_Port,BMI088_CS_GYRO_Pin,GPIO_PIN_RESET);
        }
    }

    static void CS_High(BMI088_SENSOR cs){
        if(cs == BMI088_CS_ACC){
            HAL_GPIO_WritePin(BMI088_CS_ACC_GPIO_Port,BMI088_CS_ACC_Pin,GPIO_PIN_SET);
        }
        else if(cs == BMI088_CS_GYRO){
            HAL_GPIO_WritePin(BMI088_CS_GYRO_GPIO_Port,BMI088_CS_GYRO_Pin,GPIO_PIN_SET);
        }
    }

    void cBMI088::WriteReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // 根据 cs 拉低对应 GPIO 片选；按 SPI 写协议发送地址和数据，
        // 最后释放片选，并处理通信失败与必要的延时。
        if(data == nullptr || len == 0 || len > 7)return;
        if(cs != BMI088_CS_ACC && cs != BMI088_CS_GYRO)return;
        uint8_t tx[8];
        tx[0] = addr & BMI088_SPI_WRITE_CODE;
        for(uint8_t i = 0;i < len;i++)
            tx[i + 1] = data[i];
        CS_Low(cs);
        if(HAL_SPI_Transmit(&BMI088_SPI_HANDLE,tx,len + 1,100) != HAL_OK)
            self_test.INIT_ERR = true;
        CS_High(cs);
    }

    void cBMI088::ReadReg(enum BMI088_SENSOR cs, uint8_t addr, uint8_t *data, uint8_t len)
    {
        // 根据 cs 拉低对应 GPIO 片选；按 SPI 读协议发送地址，
        // 丢弃加速度计返回的首个无效字节，读取 len 字节并释放片选。
        // 未实现前清零缓冲区，避免芯片 ID 校验读取未初始化数据。
        if(data == nullptr || len == 0 || len > 7)return;
        if(cs != BMI088_CS_ACC && cs != BMI088_CS_GYRO)return;
        uint8_t tx[9] = {0};
        uint8_t rx[9] = {0};
        tx[0] = addr | BMI088_SPI_READ_CODE;
        for(uint8_t i = 1;i < 9;i++)tx[i] = 0xFF;
        uint8_t size = len + 2;
        /* if(cs == BMI088_CS_ACC)
            size = len + 2;
        else
            size = len + 1; */
        CS_Low(cs);
        if(HAL_SPI_TransmitReceive(&BMI088_SPI_HANDLE,tx,rx,size,100) != HAL_OK)
            self_test.INIT_ERR = true;
        CS_High(cs);
        for(uint8_t i = 0;i < len;i++)
            data[i] = rx[i + 1];
        /* if(cs == BMI088_CS_ACC){
            for(uint8_t i = 0;i < len;i++)
                data[i] = rx[i + 1];
        }
        else{
            for(uint8_t i = 0;i < len;i++)
                data[i] = rx[i];
        } */
    }

    void cBMI088::Config()
    {
        tx_thread_sleep(10); //< 等待系统稳定

        /*-------------------------------------加速度计初始化-------------------------------------*/
        
        //< 先软重启，清空所有寄存器
        uint8_t pTxData;
        pTxData = ACC_SOFTRESET_VAL;
        WriteReg(BMI088_CS_ACC, ACC_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        //< 打开加速度计电源
        pTxData = ACC_PWR_CTRL_ON;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(150);

        //< 加速度计变成正常模式
        pTxData = ACC_PWR_CONF_ACT;
        WriteReg(BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(10); //

        //< 测量范围
        pTxData = ACC_RANGE_6G;
        WriteReg(BMI088_CS_ACC, ACC_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0xAB;
        WriteReg(BMI088_CS_ACC, ACC_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x08;
        WriteReg(BMI088_CS_ACC, INT1_IO_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x04;
        WriteReg(BMI088_CS_ACC, INT_MAP_DATA_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        /*-------------------------------------陀螺仪初始化-------------------------------------*/
        //< 先软重启，清空所有寄存器
        pTxData = GYRO_SOFTRESET_VAL;
        WriteReg(BMI088_CS_GYRO, GYRO_SOFTRESET_ADDR, &pTxData, 1);
        tx_thread_sleep(100); //< 延时100ms,重启需要时间

        pTxData = GYRO_RANGE_2000_DEG_S;
        WriteReg(BMI088_CS_GYRO, GYRO_RANGE_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_ODR_2000Hz_BANDWIDTH_230Hz | GYRO_LPM1_SUS;
        WriteReg(BMI088_CS_GYRO, GYRO_BANDWIDTH_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_LPM1_NOR;
        WriteReg(BMI088_CS_GYRO, GYRO_LPM1_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = GYRO_DRDY_ON;
        WriteReg(BMI088_CS_GYRO, GYRO_INT_CTRL_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x00;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_CONF_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms

        pTxData = 0x01;
        WriteReg(BMI088_CS_GYRO, GYRO_INT3_INT4_IO_MAP_ADDR, &pTxData, 1);
        tx_thread_sleep(5); //< 延时5ms
    }


    void cBMI088::ReadAccData(acc_data_t *data)
    {
        // 按 BMI088 加速度计 SPI 协议丢弃首个无效字节，拼接三轴有符号原始值，
        // 再按配置的量程换算为 m/s²，写入 data。
        if (data == nullptr)return;
        uint8_t buf[8];
        ReadReg(BMI088_CS_ACC,ACC_X_LSB_ADDR,buf,7);
        int16_t raw_x = (int16_t)(buf[2] << 8 | buf[1]);
        int16_t raw_y = (int16_t)(buf[4] << 8 | buf[3]);
        int16_t raw_z = (int16_t)(buf[6] << 8 | buf[5]);
        data->x = raw_x * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_X;
        data->y = raw_y * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Y;
        data->z = raw_z * IMU_ACCEL_6G_SEN - BMI088_ACCEL_PRE_CALI_OFFSET_Z;
    }

    void cBMI088::ReadGyroData(gyro_data_t *data)
    {
        // 读取并拼接陀螺仪三轴原始值，按配置量程换算为 rad/s，
        // 减去 Gyro_offset 后写入 data。
        if(data == nullptr)return;
        uint8_t buf[8];
        ReadReg(BMI088_CS_GYRO,GYRO_RATE_X_LSB_ADDR,buf,6);
        int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]);
        int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]);
        int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]);
        data->x = raw_x * IMU_GYRO_2000_SEN - Gyro_offset[0];
        data->y = raw_y * IMU_GYRO_2000_SEN - Gyro_offset[1];
        data->z = raw_z * IMU_GYRO_2000_SEN - Gyro_offset[2];
    }

    void cBMI088::ReadAccTemperature(float *temp)
    {
        // 丢弃加速度计读取时的首个无效字节，解析 11 位有符号温度并换算为摄氏度。
        if(temp == nullptr)return;
        uint8_t buf[8];
        ReadReg(BMI088_CS_ACC,TEMP_MSB_ADDR,buf,3);
        int16_t raw = (int16_t)(buf[1] << 3 | buf[2] >> 5);
        if(raw & 0x400)raw |= 0xF800;
        *temp = raw * TEMP_UNIT + TEMP_BIAS;
    }

}

