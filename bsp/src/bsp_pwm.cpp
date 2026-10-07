//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"

void PWM_Init(void)
{
    //定时器时钟与 PWM 模式由 CubeMX 生成的 MX_TIMx_Init 完成，此处无需额外初始化。
}

void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态
    if(htim == nullptr)return;
    HAL_TIM_PWM_Start(htim,Channel);
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
    if(htim == nullptr)return;
    HAL_TIM_PWM_Stop(htim,Channel);
}

static bool PWM_TimerOnAPB1(TIM_TypeDef *instance){
    return instance == TIM2 || instance == TIM3 || instance == TIM4
     || instance == TIM5 || instance == TIM6 || instance == TIM7
     || instance == TIM12 || instance == TIM13 || instance == TIM14;
}

static uint32_t PWM_GetTimerClock(TIM_HandleTypeDef * htim){
    uint32_t pclk;
    if(PWM_TimerOnAPB1(htim->Instance)){
        pclk = HAL_RCC_GetPCLK1Freq();
        if((RCC->CFGR & RCC_CFGR_PPRE1) != 0)pclk *= 2;
    }
    else{
        pclk = HAL_RCC_GetPCLK2Freq();
        if((RCC->CFGR & RCC_CFGR_PPRE2) != 0)pclk *= 2;
    }
    return pclk;
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
    if(htim == nullptr || period_s <= 0.0f)return;
    uint32_t timer_clk = PWM_GetTimerClock(htim);
    uint32_t PSC = htim->Init.Prescaler;
    uint32_t arr = (uint32_t)(period_s * timer_clk / (PSC + 1) + 0.5f) - 1;
    if(arr < 1)arr = 1;
    if(arr > 0xFFFF)arr = 0xFFFF;
    __HAL_TIM_SET_AUTORELOAD(htim,arr);
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
    if(htim == nullptr)return;
    if(dutyratio < 0.0f)dutyratio = 0.0f;
    if(dutyratio > 1.0f)dutyratio = 1.0f;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)(dutyratio * (arr + 1) + 0.5f);
    if(ccr > arr)ccr = arr;
    __HAL_TIM_SET_COMPARE(htim,channel,ccr);
}
