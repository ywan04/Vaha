/*
 * hx711.h
 *
 *  Created on: Jan 10, 2024
 *      Author: ivan
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "main.h"

typedef struct {
	uint16_t dt_pin;
	uint16_t sck_pin;
	uint8_t gain;
} hx711_t;

void hx711_set_timer(TIM_HandleTypeDef *tim);
void hx711_delay(uint16_t us);
void hx711_init(hx711_t *adc, uint16_t dt_pin, uint16_t sck_pin);
void hx711_set_gain(hx711_t *adc, uint8_t gain);
int32_t hx711_get_value(hx711_t *adc);
uint8_t hx711_is_ready(hx711_t *adc);

#endif /* INC_HX711_H_ */
