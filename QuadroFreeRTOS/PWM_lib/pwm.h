#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>


void InitializeTimer();
void InitializeLEDs();
void InitializePWMChannel_1(uint16_t pulse);
void InitializePWMChannel_2(uint16_t pulse);
void InitializePWMChannel_3(uint16_t pulse);
void InitializePWMChannel_4(uint16_t pulse);
