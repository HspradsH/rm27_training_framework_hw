#ifndef LED_HPP
#define LED_HPP
#include <cstdint>
#include "main.h"
#define LED_R GPIO_PIN_12
#define LED_G GPIO_PIN_11
#define LED_B GPIO_PIN_10

void LED_On(uint16_t led);
void LED_Off(uint16_t led);
void LED_Toggle(uint16_t led);

#endif