#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"
#include "BMI088.hpp"
#include "DJIMotorHandler.hpp"
#include "DJIMotor.hpp"
#include "M2006.hpp"

extern TX_SEMAPHORE imu_alive_sem;
extern TX_SEMAPHORE control_alive_sem;

extern BMI088::cBMI088 imu;
extern M2006 motor;

extern "C" void AliveThread_entry(ULONG input);
extern "C" void ImuThread_entry(ULONG input);
extern "C" void ControlThread_entry(ULONG input);
#endif