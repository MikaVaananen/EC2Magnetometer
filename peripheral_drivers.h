/* Header file for all the peripheral drivers and their relevant functions.
 * Declarations can be found in peripheral_drivers.c
 * If using the STM32F100, don't modify these! Add new ones if you need more functionality
 */
 
#ifndef PERIPHERAL_DRIVERS_H
#define PERIPHERAL_DRIVERS_H
#endif

#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "utility_functions.h"

extern int pwmPeriod;
extern int globalDutyCycle;
extern int globalDelay;

extern volatile int channels;
extern volatile uint16_t measurements;
extern volatile int measurementFlag;
extern volatile uint32_t startTime;

extern volatile uint16_t samplerate; // Samplerate default is 1kSps
extern float ref_voltage; // The value is set in the config function

void Init_Button_EXTI(void);
void Init_PWMTIM(void); // Use TIM4CH4 for PWM
void setPWMDutyCycle(int dutycycle);
void delay(uint16_t ms);

void Init_USART1(int baudrate);
void USART1_SendData(char data);
uint16_t USART1_ReceiveData(void);
void USART1_SendTerminator(void); // Are you John Connor
void USART1_SendFloat(float data);
void USART1_SendInt(int16_t data);
void USART1_SendUInt(uint16_t data);
void USART1_SendString(char string[]);
void USART1_SendLong(uint32_t data);
void USART1_IRQHandler(void);

void Init_USART2(int baudrate);
void USART2_SendData(char data);
uint16_t USART2_ReceiveData(void);
void USART2_SendTerminator(void); // Are you John Connor
void USART2_SendDouble(double data);
void USART2_SendInt(int16_t data);
void USART2_SendString(char string[]);
void USART2_IRQHandler(void);

void configADS122(int channel, int samplerate, int temperature_sensor);
