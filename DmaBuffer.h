/*
 * DmaBuffer.h
 *
 *  Created on: 01-Nov-2020
 *      Author: Dhananjay
 */

#ifndef MAIN_DMABUFFER_H_
#define MAIN_DMABUFFER_H_
#include "stdint.h"
#include "stdbool.h"
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32/rom/lldesc.h"


typedef struct DMABufferI2S
{
	lldesc_t descriptor;
	uint8_t *buffer; //uint8_t
	int index;
}DMABufferI2S;

DMABufferI2S *allocate(int bytes, bool clear,int index);
bool initI(uint8_t *buffer, int bytes, bool clear,int index);
bool init(int bytes, bool clear,int Index);
void next(DMABufferI2S *next,int index);
int sampleCount(int index);
void destroy(int index);

#endif /* MAIN_DMABUFFER_H_ */
