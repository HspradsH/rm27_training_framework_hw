#include "threads.hpp"
#include "LED.hpp"

BMI088::cBMI088 imu;
M2006 motor;

void ImuThread_entry(ULONG input){
    imu.Config();
    imu.VerifyAccChipID();
    imu.VerifyAccData();
    imu.VerifyGyroChipID();
    imu.Calibrate();
    
    while(1){
        imu.ReadAccData(&imu.acc_data);
        imu.ReadGyroData(&imu.gyro_data);
        imu.ReadAccTemperature(&imu.acc_data.temperature);
        imu.TemperatureControl(50.0f);
        imu.VerifyAccData();
        imu.VerifyGyroData();

        tx_semaphore_put(&imu_alive_sem);
        tx_thread_sleep(1);
    }
}

void ControlThread_entry(ULONG input){
    DJIMotorHandler::Instance()->registerMotor(&motor,&hcan1,0x201);
    uint32_t tick = 0;
    while(1){
        if(motor.MotorState == M2006::MOTOR_OFFLINE)
            motor.controlMode = M2006::RELAX_MODE;
        
        motor.controlMode = M2006::RELAX_MODE;
        
        motor.setOutput();
        DJIMotorHandler::Instance()->sendControlData();
        DJIMotorHandler::Instance()->AllMotorAliveCheck();

        tx_semaphore_put(&control_alive_sem);
        tx_thread_sleep(1);
    }
}

void AliveThread_entry(ULONG input){
    uint32_t imu_count,ctrl_count;
    while(1){
        imu_count = 0;
        while(tx_semaphore_get(&imu_alive_sem,TX_NO_WAIT) == TX_SUCCESS)
            imu_count++;
        ctrl_count = 0;
        while(tx_semaphore_get(&control_alive_sem,TX_NO_WAIT) == TX_SUCCESS)
            ctrl_count++;
        if(imu.self_test.INIT_ERR){
            LED_On(GPIO_PIN_12);//红灯亮
        }
        else if(imu_count == 0){

        }
        else if(ctrl_count == 0){

        }
        else{

        }
        tx_thread_sleep(500);
    }
}