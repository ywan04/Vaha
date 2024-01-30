/*
 * hx711.c
 *
 *  Created on: Jan 10, 2024
 *      Author: ivan
 */
#include "hx711.h"

static TIM_HandleTypeDef *hx711_tim;

void hx711_set_timer(TIM_HandleTypeDef *tim)
{
	hx711_tim = tim;
}

void hx711_delay(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(hx711_tim, 0);
	while(__HAL_TIM_GET_COUNTER(hx711_tim) < us);
}

void hx711_init(hx711_t *adc, uint16_t dt_pin, uint16_t sck_pin)
{
	adc->dt_pin  = dt_pin;
	adc->sck_pin = sck_pin;
	adc->gain    = 1;
	HAL_GPIO_WritePin(GPIOA, sck_pin, GPIO_PIN_RESET);
}

void hx711_set_gain(hx711_t *adc, uint8_t gain)
{
	if (gain < 64) {
		adc->gain = 2; // B channel
	} else if (gain < 128) {
		adc->gain = 3; // A channel
	} else {
		adc->gain = 1; // A channel
	}
}

int32_t hx711_get_value(hx711_t *adc)
{
	while (!hx711_is_ready(adc))
		hx711_delay(1);

	int32_t data = 0;
	uint8_t dt;
	int32_t fill;

	for (uint8_t i = 0; i < 24; ++i) {
		HAL_GPIO_WritePin(GPIOA, adc->sck_pin, GPIO_PIN_SET);
		hx711_delay(1);
		dt = HAL_GPIO_ReadPin(GPIOA, adc->dt_pin);
		data <<= 1;
		if (dt)
			++data;
		HAL_GPIO_WritePin(GPIOA, adc->sck_pin, GPIO_PIN_RESET);
		hx711_delay(1);
	}

	for (uint8_t i = 0; i < adc->gain; ++i) {
		HAL_GPIO_WritePin(GPIOA, adc->sck_pin, GPIO_PIN_SET);
		hx711_delay(1);
		HAL_GPIO_WritePin(GPIOA, adc->sck_pin, GPIO_PIN_RESET);
		hx711_delay(1);
	}

	if (data & 0x800000) {
		fill = 0xff000000;
	} else {
		fill = 0x00000000;
	}

	return fill + data;
}

uint8_t hx711_is_ready(hx711_t *adc)
{
	return HAL_GPIO_ReadPin(GPIOA, adc->dt_pin) == GPIO_PIN_RESET;
}
