/*
 * I2S.h
 *
 *  Created on: 01-Nov-2020
 *      Author: Dhananjay
 */

#ifndef MAIN_I2S_H_
#define MAIN_I2S_H_

#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp32/rom/lldesc.h"
#include "esp_log.h"
#include "DMABuffer.h"

//#include "FastLed.h"

typedef union {
	uint8_t bytes[24];
	uint32_t shorts[8];
	uint32_t raw[2];
} Lines;

typedef struct CHSV {
    union {
		struct {
		    union {
		        uint8_t hue;
		        uint8_t h; };
		    union {
		        uint8_t saturation;
		        uint8_t sat;
		        uint8_t s; };
		    union {
		        uint8_t value;
		        uint8_t val;
		        uint8_t v; };
		};
		uint8_t raw[3];
	};
}CHSV;

typedef struct CRGB {
	union {
		struct {
            union {
                uint8_t r;
                uint8_t red;
            };
            union {
                uint8_t g;
                uint8_t green;
            };
            union {
                uint8_t b;
                uint8_t blue;
            };
        };
		uint8_t raw[3];
		uint32_t num;
	};
}CRGB;

/// hardware index [0, 1]
void I2S(const int m_i2sIndex);
void setLedBuff(CRGB* newBuff);
void reset();

void showPixels();
void putPixelinBuffer(Lines *pixel,uint32_t *buf);
void transpose24x1_noinline(unsigned char *A, uint32_t *B);

void i2sStop();
void startTX();
void startRX();

void resetDMA();
void resetFIFO();
bool initParallelOutputMode(const int *pinMap);
//bool initParallelInputMode(const int *pinMap, long sampleRate = 100000, int baseClock = -1, int wordSelect = -1);

void allocateDMABuffers(int count, int bytes);
void deleteDMABuffers();
void stop();
void setBrightness(uint8_t b);
void initled(CRGB *m_leds,int * m_Pins,int m_num_strips,int m_nun_led_per_strip);
void empty ( uint32_t *buf);
void transpose24x1_noinline(unsigned char *A, uint32_t *B);
void putPixelinBuffer(Lines *pixel,uint32_t *buf);
void showPixels();

//void IRAM_ATTR interrupt(void *arg);
void interrupt();


#endif /* MAIN_I2S_H_ */
