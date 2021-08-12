/* 
 * My implementations of some basic utility functions, like LEDs and delay
 * Don't change these if using the STM32F100!
 */


#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "utility_functions.h"
#include "peripheral_drivers.h"
#include "string.h"

volatile extern uint32_t ticks = 0;

void DeInitAll(void) {
	RCC_DeInit();
	GPIO_DeInit(GPIOC);
	GPIO_DeInit(GPIOA);
	TIM_DeInit(TIM2);
	TIM_DeInit(TIM3);
	TIM_DeInit(TIM4);
	USART_DeInit(USART1);
	USART_DeInit(USART2);
}

void Init_LEDs(void) {	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Init_DelayTIM(void){
	RCC_ClocksTypeDef clocks;
	int clock_freq;
	int tim_ms_freq = 1000;
	int tim_us_freq = 1000000;
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	RCC_GetClocksFreq(&clocks);
	
	clock_freq = clocks.PCLK1_Frequency;
	TIM_TimeBaseInitStruct.TIM_Prescaler = (clock_freq/tim_ms_freq) - 1; // 24 MHz / (24000) = 1000 Hz
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM2, ENABLE);
	
	/* TIM3 microsecond timer initialization */
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	RCC_GetClocksFreq(&clocks);
	
	clock_freq = clocks.PCLK1_Frequency;
	TIM_TimeBaseInitStruct.TIM_Prescaler = (clock_freq/tim_us_freq) - 1; // 24 MHz / (24000) = 1000 Hz
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = 65535;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
	TIM_Cmd(TIM3, ENABLE);
	
}
void delay(uint16_t ms){
	TIM_SetCounter(TIM2, 0);
	while(ms > TIM_GetCounter(TIM2));
}

void delayus(uint16_t us){
	TIM_SetCounter(TIM3, 0);
	while(us > TIM_GetCounter(TIM3));
}

void LEDOn(int LED) {
	switch(LED){
		case BLUE:
			GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
			break;
		case GREEN:
			GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
			break;
	}
}

void LEDOff(int LED) {
	switch(LED){
		case BLUE:
			GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
			break;
		case GREEN:
			GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
			break;
	}
}

void Init_Button() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef ButtonInit;
	ButtonInit.GPIO_Mode = GPIO_Mode_IPD;
	ButtonInit.GPIO_Speed = GPIO_Speed_50MHz;
	ButtonInit.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &ButtonInit);	
}

void LEDToggle(int LED) {
	int output;
	switch(LED){
		case GREEN:
			output = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9);
			if(output == Bit_RESET) {
				LEDOn(GREEN);
			} else {
				LEDOff(GREEN);
			}
			break;
		case BLUE:
			output = GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_8);
			if(output == Bit_RESET) {
				LEDOn(BLUE);
			} else {
				LEDOff(BLUE);
			}
			break;
	}
}
void LEDBlink(int LED, int time) {
	LEDOn(LED);
	delay(time);
	LEDOff(LED);
}

void dot(void) {
	LEDBlink(GREEN, 100);
	delay(100);
}

void dash(void){
	LEDBlink(GREEN, 200);
	delay(100);
}

void morse(char letter){
	LEDOff(GREEN);
	switch(letter) {
		case 'a': dot(); dash(); break;
		case 'b': dash(); dot(); dot(); dot(); break;
		case 'c': dash(); dot(); dash(); dot(); break;
		case 'd': dash(); dot(); dot(); break;
		case 'e': dot(); break;
		case 'f': dot(); dot(); dash(); dot(); break;
		case 'g': dash(); dash(); dot(); break;
		case 'h': dot(); dot(); dot(); dot(); break;
		case 'i': dot(); dot(); break;
		case 'j': dot(); dash(); dash(); dash(); break;
		case 'k': dash(); dot(); dash(); break;
		case 'l': dot(); dash(); dot(); dot(); break;
		case 'm': dash(); dash(); break;
		case 'n': dash(); dot(); break;
		case 'o': dash(); dash(); dash(); break;
		case 'p': dot(); dash(); dash(); dot(); break;
		case 'q': dash(); dash(); dot(); dash(); break;
		case 'r': dot(); dash(); dash(); break;
		case 's': dot(); dot(); dot(); break;
		case 't': dash(); break;
		case 'u': dot(); dot(); dash(); break;
		case 'v': dot(); dot(); dot(); dash(); break;
		case 'w': dot(); dash(); dash(); break;
		case 'x': dash(); dot(); dot(); dash(); break;
		case 'y': dash(); dot(); dash(); dash(); break;
		case 'z': dash(); dash(); dot(); dot(); break;
		
		case '1': dot(); dash(); dash(); dash(); dash(); break;
		case '2': dot(); dot(); dash(); dash(); dash(); break;
		case '3': dot(); dot(); dot(); dash(); dash(); break;
		case '4': dot(); dot(); dot(); dot(); dash(); break;
		case '5': dot(); dot(); dot(); dot(); dot(); break;
		case '6': dash(); dot(); dot(); dot(); dot(); break;
		case '7': dash(); dash(); dot(); dot(); dot(); break;
		case '8': dash(); dash(); dash(); dot(); dot(); break;
		case '9': dash(); dash(); dash(); dash(); dot(); break;
		case '0': dash(); dash(); dash(); dash(); dash(); break;
	}
	USART1_SendData(letter);
}

void SysTick_Init(uint16_t frequency) {
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq (&RCC_Clocks);
	uint32_t SysTick_frequency = RCC_Clocks.HCLK_Frequency / frequency;
	(void) SysTick_Config(SysTick_frequency );
}

void SysTick_Handler(){
	ticks++;
}

uint32_t millis(){
	return ticks;
}
