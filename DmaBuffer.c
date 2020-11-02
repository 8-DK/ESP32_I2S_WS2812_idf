/*
 * DmaBuffer.c
 *
 *  Created on: 01-Nov-2020
 *      Author: Dhananjay
 */

#include "DmaBuffer.h"

//
//typedef struct DMABufferI2S
//lldesc_t descriptor[2];
//uint8_t *buffer[2]; //uint8_t
//}DMABufferI2S;

DMABufferI2S *dmaBuffer[2];

DMABufferI2S *allocate(int bytes, bool clear,int index)
{
	dmaBuffer[index] = (DMABufferI2S *)heap_caps_malloc(sizeof(DMABufferI2S), MALLOC_CAP_DMA);
	if (!dmaBuffer[index])
	{
		ESP_LOGE("DMABuffer","Failed to alloc DMABuffer class");
		//Serial.println("impossible");
	}
	init(bytes, clear, index);
	return dmaBuffer[index];
}

bool initI(uint8_t *buffer, int bytes, bool clear,int index) //uint8_t
{
	if (!buffer)
		return false;
	dmaBuffer[index]->buffer = buffer;
	if (clear)
		for (int i = 0; i < bytes*4; i++)
			buffer[i] = 0;
	dmaBuffer[index]->descriptor.length = bytes*4;//bytes
	dmaBuffer[index]->descriptor.size = dmaBuffer[index]->descriptor.length/4;
	dmaBuffer[index]->descriptor.owner = 1;
	dmaBuffer[index]->descriptor.sosf = 1;
	dmaBuffer[index]->descriptor.buf = (uint8_t *)buffer; //uint8_t
	dmaBuffer[index]->descriptor.offset = 0;
	dmaBuffer[index]->descriptor.empty = 0;
	dmaBuffer[index]->descriptor.eof = 1;
	dmaBuffer[index]->descriptor.qe.stqe_next = 0;
	return true;
}

bool init(int bytes, bool clear,int index)
{
	return initI((uint8_t *)heap_caps_malloc(bytes*4, MALLOC_CAP_DMA), bytes, clear,index); //uint8_t
}

void next(DMABufferI2S *next,int index)
{
	dmaBuffer[index]->descriptor.qe.stqe_next = &(next->descriptor);
}

int sampleCount(int index)
{
	return dmaBuffer[index]->descriptor.length / 4;
}

void destroy(int index)
{
	if (dmaBuffer[index]->buffer)
	{
		free(dmaBuffer[index]->buffer);
		dmaBuffer[index]->buffer = 0;
	}
	free(dmaBuffer[index]);
}
