/*
 * motor_demo.c
 *
 *  Created on: 12.12.2018 ã.
 *      Author: Vlado
 */

#include "main.h"

volatile uint16_t pot_val = 0;
volatile uint32_t disk_steps = 0;
float speed;
uint8_t display_data[4] = {0};

//Decoding maps taken from MultiFuncShield-Library for Arduino by Hacktronics
/* Segment byte maps for numbers 0 to 9 */
const uint8_t SEGMENT_MAP_DIGIT[10] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90};
/* Segment byte maps for alpha a-z */
const uint8_t SEGMENT_MAP_ALPHA[26] = {136, 131, 167, 161, 134, 142, 144, 139, 207, 241, 182, 199, 182, 171, 163, 140, 152, 175, 146, 135, 227, 182, 182, 182, 145, 182};
/* Byte maps to select digit 1 to 4 */
const uint8_t SEGMENT_SELECT[4] = {0xF1, 0xF2, 0xF4, 0xF8};
//Digit value is written before segment selection!

void calculate_speed(void)
{
	uint8_t i;
	uint16_t int_speed = 0;
	//To get the speed in RPM the number of rotations is multiplied by 4
	//because the speed is recalculated every 250ms.
	speed = 240 * ((float) disk_steps / STEPS_PER_ROTATION);
	disk_steps = 0;
	if (speed > 9999)
	{
		//GOTTAGOFAST!
		for (i = 0; i < 4; i++)
		{
			display_data[i] = 191;	// 191 => dash
		}
	}
	else
	{
		int_speed = (uint16_t) speed;
		for (i = 0; i < 4; i++)
		{
			display_data[i] = SEGMENT_MAP_DIGIT[int_speed % 10];
			int_speed /= 10;
		}
	}
}

void calculate_power(void)
{
	LL_TIM_OC_SetCompareCH1(TIM1, (pot_val * MAX_POWER) / ADC_MAX);
	if (pot_val > ALARM_THR_1)
	{
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
		LL_TIM_SetAutoReload(TIM3, FAST_ALARM);
		LL_TIM_OC_SetCompareCH1(TIM3, FAST_ALARM / 2);
		LL_TIM_SetAutoReload(TIM4, HIGH_PITCH);
		LL_TIM_OC_SetCompareCH1(TIM4, HIGH_PITCH / 2);
	}
	else if (pot_val > ALARM_THR_2)
	{
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
		LL_TIM_SetAutoReload(TIM3, MED_ALARM);
		LL_TIM_OC_SetCompareCH1(TIM3, MED_ALARM / 2);
		LL_TIM_SetAutoReload(TIM4, MED_PITCH);
		LL_TIM_OC_SetCompareCH1(TIM4, MED_PITCH / 2);
	}
	else if (pot_val > ALARM_THR_3)
	{
		LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
		LL_TIM_SetAutoReload(TIM3, SLOW_ALARM);
		LL_TIM_OC_SetCompareCH1(TIM3, SLOW_ALARM / 2);
		LL_TIM_SetAutoReload(TIM4, LOW_PITCH);
		LL_TIM_OC_SetCompareCH1(TIM4, LOW_PITCH / 2);
	}
	else
	{
		//Disables the timer output
		LL_TIM_CC_DisableChannel(TIM4, LL_TIM_CHANNEL_CH1);
		LL_TIM_GenerateEvent_UPDATE(TIM4);
	}
}

void refresh_7segm(void)
{
	static uint8_t digit = 0;
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		if (display_data[digit] & (1U << (7 - i)))
		{
			LL_GPIO_SetOutputPin(DATA_7SEG_GPIO_Port, DATA_7SEG_Pin);
		}
		else
		{
			LL_GPIO_ResetOutputPin(DATA_7SEG_GPIO_Port, DATA_7SEG_Pin);
		}
		LL_GPIO_SetOutputPin(CLK_7SEG_GPIO_Port, CLK_7SEG_Pin);
		LL_GPIO_ResetOutputPin(CLK_7SEG_GPIO_Port, CLK_7SEG_Pin);
	}
	for (i = 0; i < 8; i++)
	{
		if (SEGMENT_SELECT[3 - digit] & (1U << (7 - i)))
		{
			LL_GPIO_SetOutputPin(DATA_7SEG_GPIO_Port, DATA_7SEG_Pin);
		}
		else
		{
			LL_GPIO_ResetOutputPin(DATA_7SEG_GPIO_Port, DATA_7SEG_Pin);
		}
		LL_GPIO_SetOutputPin(CLK_7SEG_GPIO_Port, CLK_7SEG_Pin);
		LL_GPIO_ResetOutputPin(CLK_7SEG_GPIO_Port, CLK_7SEG_Pin);
	}
	LL_GPIO_SetOutputPin(LATCH_7SEG_GPIO_Port, LATCH_7SEG_Pin);
	LL_GPIO_ResetOutputPin(LATCH_7SEG_GPIO_Port, LATCH_7SEG_Pin);

	digit++;
	if (digit > 3)
	{
		digit = 0;
	}
}
