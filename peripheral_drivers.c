/* Source file for all the peripheral drivers and their relevant functions.
 * If using the STM32F100, don't modify these! Add new ones if you need more functionality
 */


#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "utility_functions.h"
#include "peripheral_drivers.h"

int globalDutyCycle = 50;
int pwmPeriod = 0;
int globalDelay = 100;
extern volatile int measurementFlag = 0;
extern volatile int channels = 0;
extern volatile uint32_t startTime = 0;
extern volatile uint16_t measurements = 0;

extern volatile uint16_t samplerate = 1000; // Samplerate default is 1kSps
extern float ref_voltage = 0; // The value is set in the config function

void Init_Button_EXTI(void){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef ButtonInit;
	ButtonInit.GPIO_Mode = GPIO_Mode_IPD;
	ButtonInit.GPIO_Speed = GPIO_Speed_50MHz;
	ButtonInit.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &ButtonInit);
	
	// External interrupt initialization
	EXTI_DeInit();
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	// NVIC initialization
	NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //Enable keypad external interrupt channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //Priority 0, 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //Sub priority 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //Enable external interrupt channel 
  NVIC_Init(&NVIC_InitStructure);
}

void Init_PWMTIM(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	int pwmFreq = 10000;
	RCC_ClocksTypeDef clocks;
	
	TIM_DeInit(TIM4);
	GPIO_DeInit(GPIOB);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	// Tricks to get the PWM frequency right
	RCC_GetClocksFreq(&clocks);
	pwmPeriod = (int) clocks.PCLK1_Frequency/pwmFreq - 1;
	
	TIM_TimeBaseInitStruct.TIM_Prescaler = (clocks.PCLK1_Frequency/(pwmPeriod*pwmFreq)) - 1; // PCLK1 / (PSC * period) = 1khZ
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = pwmPeriod;                
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = (int) (pwmPeriod - 1)/4;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM4, &TIM_OCInitStruct);
	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;               // Pin PB9 is TIM4 Ch4
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void setPWMDutyCycle(int dutyCycle) {
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_Pulse = (int) (pwmPeriod*dutyCycle)/100.0;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM4, &TIM_OCInitStruct);
	delay(1);
}



void Init_USART1(int baudrate){
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART1, &USART_InitStruct);
	
	/* Tx pin PA9 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Rx pin PA10 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	USART_Cmd(USART1, ENABLE);
	
	// Configure USART1 interrupts and enable global interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_SendData(char data) {
	USART_SendData(USART1, (uint16_t) data);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

uint16_t USART1_ReceiveData(){
	uint16_t data = USART_ReceiveData(USART1);
	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	return data;
}

void USART1_SendTerminator() {
	//USART1_SendData('\r');
	USART1_SendData('\n');
}

void USART1_SendFloat(float data){
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%f", data); // Print the data to a string buffer
	USART1_SendString(buffer); // Send the string buffer
}

void USART1_SendInt(int16_t data){
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%i", data); // Print the data to a string buffer
	USART1_SendString(buffer); // Send the string buffer
}

void USART1_SendUInt(uint16_t data){
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%i", data); // Print the data to a string buffer
	USART1_SendString(buffer); // Send the string buffer
}

void USART1_SendString(char string[]){
	int i = 0;
	for (i = 0; i < strlen(string); i++) {
		USART1_SendData(string[i]);
	}
	USART1_SendTerminator();
}

void USART1_SendLong(uint32_t data) {
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%u", data); // Print the data to a string buffer
	USART1_SendString(buffer); // Send the string buffer
}

void USART1_IRQHandler(void){
/* RXNE handler */
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		char letter = USART1_ReceiveData();
		
		if(letter == 't'){
			LEDOn(BLUE);
			measurementFlag = 1;
			startTime = millis();
			LEDOff(BLUE);
		}
		
		if(letter == 'c'){
			LEDOn(BLUE);
			USART1_SendInt(channels);
			LEDOff(BLUE);
		}
		
		if(letter == 'C'){
			LEDOn(BLUE);
			channels = USART1_ReceiveData() - '0';
			USART1_SendInt(channels);
			LEDOff(BLUE);
		}
		
		if(letter == 'd'){
			LEDOn(BLUE);
			measurementFlag = 0;
			LEDOff(BLUE);
		}
		
		if(letter == 'b'){
			LEDOn(BLUE);
			uint16_t measThousands = USART1_ReceiveData() - '0';
			uint16_t measHundreds = USART1_ReceiveData() - '0';
			uint16_t measTens = USART1_ReceiveData() - '0';
			uint16_t measOnes = USART1_ReceiveData() - '0';
			measurements = 1000*measThousands + 100*measHundreds + 10*measTens + measOnes;
			USART1_SendInt(measurements);
			LEDOff(BLUE);
		}
		
		if(letter == 's'){
			LEDOn(BLUE);
			uint16_t samplerateThousands = USART1_ReceiveData() - '0';
			uint16_t samplerateHundreds = USART1_ReceiveData() - '0';
			uint16_t samplerateTens = USART1_ReceiveData() - '0';
			uint16_t samplerateOnes = USART1_ReceiveData() - '0';
			samplerate = 1000*samplerateThousands + 100*samplerateHundreds + 10*samplerateTens + samplerateOnes;
			configADS122(1, samplerate, 0);
			USART1_SendInt(samplerate);
			LEDOff(BLUE);
		}
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

void Init_USART2(int baudrate){
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	
	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	
	USART_Init(USART2, &USART_InitStruct);
	
	/* Tx pin PA2 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Rx pin PA3 */
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	USART_Cmd(USART2, ENABLE);
	
	// Configure USART2 interrupts and enable global interrupt
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_SendData(char data) {
	USART_SendData(USART2, (uint16_t) data);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

uint16_t USART2_ReceiveData(){
	uint16_t data = USART_ReceiveData(USART2);
	while(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);
	return data;
}

void USART2_SendTerminator() {
	USART2_SendData('\r');
	USART2_SendData('\n');
}

void USART2_SendDouble(double data){
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%f", data); // Print the data to a string buffer
	USART2_SendString(buffer); // Send the string buffer
}

void USART2_SendInt(int16_t data){
	char buffer[32]; // 32 bytes to be on the safe side
	sprintf(buffer, "%i", data); // Print the data to a string buffer
	USART2_SendString(buffer); // Send the string buffer
}

void USART2_SendString(char string[]){
	int i = 0;
	for (i = 0; i < strlen(string); i++) {
		USART2_SendData(string[i]);
	}
	USART2_SendTerminator();
}

void USART2_IRQHandler(void){
/* RXNE handler */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		//char letter = USART2_ReceiveData();
		//morse(letter);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

void configADS122(int channel, int samplerate, int temperature_sensor){
	char config_bytes[5] = {0,0,0,0,0};

	switch(channel) {
		case 0:
			/* Write configuration register 0 */
			config_bytes[0] += 128; // AINn=AVSS AINp=AIN0
			break;
		case 1:
			config_bytes[0] += 144; // AINn=AVSS AINp=AIN1
			break;
		case 2:
			config_bytes[0] += 160; // AINn=AVSS AINp=AIN2
			break;
		case 3:
			config_bytes[0] += 176; // AINn=AVSS AINp=AIN3
			break;
		case 99:
			config_bytes[0] = 224; // AINp = AINn = (AVdd + AVss)/2; used for calibration
			break;
	}
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x40); // Write conf register 0 command
	USART2_SendData(config_bytes[0]); // [3:1] gain=1, [0] PGA off
	
	
	/* Write configuration register 1 */
	switch(samplerate){
		case 20: config_bytes[1] += 0; break;
		case 45: config_bytes[1] += 32; break;
		case 90: config_bytes[1] += 64; break;
		case 175: config_bytes[1] += 96; break;
		case 330: config_bytes[1] += 128; break;
		case 600: config_bytes[1] += 160; break;
		case 1000: config_bytes[1] += 192; break;
	}
	//config_bytes[1] += (0 << 4) | config_bytes[1]; // Mode = normal;
	config_bytes[1] += (1 << 4) | config_bytes[1]; // Mode = Turbo (2x samplerate)
	config_bytes[1] += 0x00; // Conversion mode = single;
	
	/* Vref = AVdd - AVss; doesn't work for some reason, results are off by several orders of magnitude */
	//config_bytes[1] += 4; ref_voltage = 5.05;
	
	/* Vref = internal 2.048V */
	config_bytes[1] += 0; ref_voltage = 2.048;
	
	if(temperature_sensor == 1) {
		config_bytes[1] += 1; // Switch temperature sensor on
	}
	
	USART2_SendData(0x55); // Send sync word
	USART2_SendData(0x42); // Write conf register 1 command
	USART2_SendData(config_bytes[1]); // [7:5] Data rate=1000 S/s, [4] Mode=Turbo, [3] Conversion mode=single, [2:1] VREF set before, [0] Temperature sensor mode=0
	
	/* Write configuration register 2 */
	USART2_SendData(0x55); // Send sync word
	USART2_SendData(0x44); // Write conf register 2 command
	USART2_SendData(0x00); // [7] Conversion ready = 0, [6] Data counter = disable, [5:4] data integrity check = disable, [3] BCS = disable, [2:0] IDAC current = off
	
	/* Write configuration register 3 */
	USART2_SendData(0x55); // Send sync word
	USART2_SendData(0x45); // Write conf register 3 command
	USART2_SendData(0x00); // [7:5] IDAC1 = disabled, [4:2] IDAC2 = disabled, [1] RESERVED; always 0, [0] data output mode = manual
	
	/* Give the ADC some time to set the registers */
	delay(5);
}

