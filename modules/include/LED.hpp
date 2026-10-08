#ifndef LED_HPP
#define LED_HPP
#include <cstdint>
#include "main.h"

void LED_On(uint16_t led);
void LED_Off(uint16_t led);
void LED_Toggle(uint16_t led);

#endif