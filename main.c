#include <stm32f10x.h>
#include "system_stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "utility_functions.h"
#include "peripheral_drivers.h"

void EXTI0_IRQHandler(void);

float takeMeasurement_B(int channel, int measurementSamplerate);
float takeMeasurement_voltage(int channel, int measurementSamplerate);
uint32_t takeMeasurement_raw(int channel, int measurementSamplerate);
void stopMeasuring(void);

void getADS122Config(char *configArray);
void setChannel_ADS122(int channel);
float offsetCalibrationADS122(void);
void resetADS122(void);

int resolution = 24; // Bits
uint32_t divisor = 0;
int gain = 1;

float offset_voltage = 0.0;

int main(void) {
	DeInitAll();
	Init_LEDs();
	Init_DelayTIM();
	Init_Button_EXTI();
	SysTick_Init(1000);
	Init_USART1(460800); // PC communication
	Init_USART2(115200); // ADC communication
	
	divisor = (1 << resolution) - 1;
	
	resetADS122();
	configADS122(1, samplerate, 0);
	
	LEDOn(GREEN);
	delay(100);
	LEDOn(BLUE);
	delay(100);
	LEDOff(GREEN);
	delay(100);
	LEDOff(BLUE);

	while(1) {
		if(measurementFlag) {
			startTime = millis();
			for(uint32_t j = 0; j < measurements; j++) {
				LEDOn(GREEN);
				USART1_SendLong(millis() - startTime);
				for(int i = 0; i < channels; i++) {
					USART1_SendFloat(takeMeasurement_voltage(i, samplerate));
				}
			}
			measurementFlag = 0;
		}
		LEDOff(GREEN);
		stopMeasuring();
	}
}

void EXTI0_IRQHandler(){
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		channels = 1;
		measurements = 100;
		measurementFlag = !measurementFlag;;
		startTime = millis();
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}

float takeMeasurement_voltage(int channel, int measurementSampleRate){
	char data[3] = {0, 0, 0};
	uint32_t fulldata = 0;
	float voltage = 0.0;
	
	setChannel_ADS122(channel);
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x08); // Start conversion
	delayus(500); // Give the ADC some time to finish the conversion
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x10); // Read data command
	
	data[0] = USART2_ReceiveData();
	data[1] = USART2_ReceiveData();
	data[2] = USART2_ReceiveData();
	
	/* All the data bytes are reversed bitwise since the ADC sends the LSB first */
	fulldata = data[2] << 8; // Shift first byte 8 bits to the left to make room for the second byte
	fulldata = (fulldata | data[1]) << 8; // Add the second byte to the full data
	fulldata = fulldata | data[0]; // Add the LSB byte to the full data
	
	voltage = ref_voltage*fulldata/divisor - offset_voltage; // volts;
	return voltage;
}

uint32_t takeMeasurement_raw(int channel, int measurementSampleRate){
	char data[3] = {0, 0, 0};
	uint32_t fulldata = 0;
	
	configADS122(channel, measurementSampleRate, 0); // Chosen channel, sample rate set before main(), no temperature measurement
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x08); // Start conversion
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x10); // Read data command
	
	data[0] = USART2_ReceiveData();
	data[1] = USART2_ReceiveData();
	data[2] = USART2_ReceiveData();
	
	/* All the data bytes are reversed bitwise since the ADC sends the LSB first */
	fulldata = data[2] << 8; // Shift first byte 8 bits to the left to make room for the second byte
	fulldata = (fulldata | data[1]) << 8; // Add the second byte to the full data
	fulldata = fulldata | data[0]; // Add the LSB byte to the full data
	
	return fulldata;
}

float takeMeasurement_B(int channel, int measurementSamplerate){
	float B = takeMeasurement_voltage(channel, measurementSamplerate);
	B = (B - 1.25)*100000; // nT; The output of the sensor is divided in two to be usable with the internal voltage reference of the ADS122
	return B;
}

void getADS122Config(char *configArray){
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x20); // Read config register 0
	configArray[0] = USART2_ReceiveData();
	
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x22); // Read config register 1
	configArray[1] = USART2_ReceiveData();
	
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x24); // Read config register 2
	configArray[2] = USART2_ReceiveData();
	
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x26); // Read config register 3
	configArray[3] = USART2_ReceiveData();
	
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x28); // Read config register 4
	configArray[4] = USART2_ReceiveData();
	USART1_SendString(configArray);
}

float offsetCalibrationADS122(){
	int16_t measurements = 1000;
	int16_t measurementsDone = 0;
	float offset = 0.0;
	float offset_voltage_cumulative = 0.0;
	
	USART2_SendData(0x55); // Send sync word
	USART2_SendData(0x06); // Reset
	
	configADS122(99, 1000, 0); // Both inputs shorted to mid-supply for calibration
		
	/* Measures the voltage n times */
	while(measurementsDone < measurements) {
		float voltage = takeMeasurement_voltage(99, 1000);
		offset_voltage_cumulative += voltage;
		measurementsDone++;
	}
	
	/* Calculates the average */
	offset = offset_voltage_cumulative/measurements - 5.05/2;
	return offset;
}

void resetADS122(){
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x06); // Reset command
}

void stopMeasuring(void){
	USART2_SendData(0x55); // Sync word
	USART2_SendData(0x02); // Power down mode
}

void setChannel_ADS122(int channel) {
	char config_byte = 1 << 3; // Set fourth bit to 1
	/* Should work like this; channel selection is the 4 highest bits.
	Channel 0 in relation to AVSS is 1000; 0b1000 + 0
	Channel 1 in relation to AVSS is 1001; 0b1000 + 1
	Channel 2 in relation to AVSS is 1010; 0b1000 + 2
	Channel 3 in relation to AVSS is 1011; 0b1000 + 3
	Afterwards, shift everything left by 4 bits
	*/ 
	config_byte += channel;
	config_byte = config_byte << 4;
	
	USART2_SendData(0x55); // Sync communications
	USART2_SendData(0x40); // Write conf register 0 command
	USART2_SendData(config_byte); // [3:1] gain=1, [0] PGA off
	delayus(500); // Let the ADC write the configuration in peace
}
