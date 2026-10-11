/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"
#include "tx_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern void AliveThread_entry(ULONG input);
extern void ImuThread_entry(ULONG input);
extern void ControlThread_entry(ULONG input);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALIVE_THREAD_STACK_SIZE 1024
#define IMU_THREAD_STACK_SIZE 2048
#define CONTROL_THREAD_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static TX_THREAD alive_thread;
static TX_THREAD imu_thread;
static TX_THREAD control_thread;

TX_SEMAPHORE imu_alive_sem;
TX_SEMAPHORE control_alive_sem;

static VOID *alive_stack_ptr;
static VOID *imu_stack_ptr;
static VOID *control_stack_ptr;

extern void AliveThread_entry(ULONG thread_input);
extern void ImuThread_entry(ULONG thread_input);
extern void ControlThread_entry(ULONG thread_input);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  if(tx_semaphore_create(&imu_alive_sem,"imu alive",0) != TX_SUCCESS)
    ret = TX_START_ERROR;
  if(tx_semaphore_create(&control_alive_sem,"control alive",0) != TX_SUCCESS)
    ret = TX_START_ERROR;

  if(tx_byte_allocate(byte_pool,&alive_stack_ptr,ALIVE_THREAD_STACK_SIZE,TX_NO_WAIT) != TX_SUCCESS)
    ret = TX_START_ERROR;
  if(tx_byte_allocate(byte_pool,&imu_stack_ptr,IMU_THREAD_STACK_SIZE,TX_NO_WAIT) != TX_SUCCESS)
    ret = TX_START_ERROR;
  if(tx_byte_allocate(byte_pool,&control_stack_ptr,CONTROL_THREAD_STACK_SIZE,TX_NO_WAIT) != TX_SUCCESS)
    ret = TX_START_ERROR;
  /* USER CODE END App_ThreadX_Init */

  if(ret == TX_SUCCESS){
    tx_thread_create(&control_thread,"control",ControlThread_entry,
    0,control_stack_ptr,CONTROL_THREAD_STACK_SIZE,
    5,5,TX_NO_TIME_SLICE,TX_AUTO_START);

    tx_thread_create(&imu_thread,"imu",ImuThread_entry,
    0,imu_stack_ptr,IMU_THREAD_STACK_SIZE,
    6,6,TX_NO_TIME_SLICE,TX_AUTO_START);

    tx_thread_create(&alive_thread,"alive",AliveThread_entry,
    0,alive_stack_ptr,ALIVE_THREAD_STACK_SIZE,
    10,10,TX_NO_TIME_SLICE,TX_AUTO_START);
  }
  
  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
