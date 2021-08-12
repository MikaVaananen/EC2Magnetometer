/* 
 * My implementations of some basic utility functions, like LEDs and delay
 */


#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H
#endif

#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "string.h"

#define GREEN 1
#define BLUE 0

volatile extern uint32_t ticks;

void DeInitAll(void);
void Init_LEDs(void);
void Init_Button(void);
void LEDOn(int LED);
void LEDOff(int LED);
void LEDToggle(int LED);
void LEDBlink(int LED, int time);

void Init_DelayTIM(void); // TIM2 used as 1kHz timer for the delay function
void delay(uint16_t ms);
void delayus(uint16_t us);

void dot(void);
void dash(void);
void morse(char letter);

void SysTick_Init(uint16_t frequency);
void SysTick_Handler(void);
uint32_t millis(void);
