#include "LED.hpp"

void LED_On(uint16_t led){
    HAL_GPIO_WritePin(GPIOH,led,GPIO_PIN_SET);
}
void LED_Off(uint16_t led){
    HAL_GPIO_WritePin(GPIOH,led,GPIO_PIN_RESET);
}
void LED_Toggle(uint16_t led){
    HAL_GPIO_TogglePin(GPIOH,led);
}