#include "I2S.h"
#define TAG_I2s "I2S_Dvr"
//#include <soc/rtc.h>
#define I2S_BASE_CLK (1600000000L)
i2s_dev_t *i2sDevices[] = {&I2S0, &I2S1};

int i2sIndex;
intr_handle_t interruptHandle;

int dmaBufferActive;
DMABufferI2S **dmaBuffers;
volatile bool stopSignal;
volatile bool runningPixel=false;

int ledToDisplay;
CRGB *leds;
int dmaBufferCount=2; //we use two buffers
volatile  int num_strips;
volatile  int nun_led_per_strip;
int *Pins;
int brigthness;
int ledType;


void IRAM_ATTR  callback()
{
	Lines pixel[3];
	// i2s_dev_t *i2sDevices[] = {&I2S0, &I2S1};
	//i2s_dev_t &i2s = *i2sDevices[i2sIndex];

	if(stopSignal)
	{
		//delay(0);*
		I2S0.conf.tx_start = 0;
		//i2sStop();
		runningPixel=false;
		return;
	}
	if(ledToDisplay<nun_led_per_strip)
	{

		for(int i = 0; i <num_strips; i++) {
			pixel[0].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].g/brigthness;
			pixel[1].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].r/brigthness;
			pixel[2].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].b/brigthness;
		}
		ledToDisplay++;

		putPixelinBuffer(pixel,(uint32_t*)dmaBuffers[dmaBufferActive]->buffer);
		dmaBufferActive = (dmaBufferActive + 1)% dmaBufferCount;

	}
	else
	{
		if(ledToDisplay<=nun_led_per_strip+1)
		{
			//cont->ledbuff[cont->ledToDisplay]=cont->dmaBufferActive;
			CRGB *poli=leds+ledToDisplay;
			for(int i = 0; i <num_strips; i++) {
				//Serial.println((uint32_t)int_leds);
				pixel[0].bytes[i] = 0x00;
				pixel[1].bytes[i] = 0x00;
				pixel[2].bytes[i] = 0x00;
				poli+=nun_led_per_strip;
			}
			putPixelinBuffer(pixel,(uint32_t*)dmaBuffers[dmaBufferActive]->buffer);



			ledToDisplay++;
			dmaBufferActive = (dmaBufferActive + 1)% dmaBufferCount;
		}
		else
			stopSignal=true;
	}
}


void IRAM_ATTR interrupt(void *arg)
{
	//volatile i2s_dev_t &i2s = *i2sDevices[((I2S *)arg)->i2sIndex];
	//i2s->int_clr.val = i2s->int_raw.val;

	if(GET_PERI_REG_BITS(I2S_INT_ST_REG(0), I2S_OUT_EOF_INT_ST_V,  I2S_OUT_EOF_INT_ST_S))
		callback();
	//	((I2S *)arg)->callback();
	REG_WRITE(I2S_INT_CLR_REG(0), (REG_READ(I2S_INT_RAW_REG( 0 )) & 0xffffffc0) | 0x3f);


	//((I2S *)arg)->interrupt();
}

void I2S(const int m_i2sIndex)
{
	i2sIndex = m_i2sIndex;
	interruptHandle = 0;
	dmaBufferCount = 0;
	dmaBufferActive = 0;
	dmaBuffers = 0;
	stopSignal = false;
}

/*
void interrupt()
{
	////Serial.println("interupt");
	//i2sStop();
	//return;
	////Serial.println("interupt");
	//two buufer

	dmaBufferActive = (dmaBufferActive + 1);// % dmaBufferCount;
	if(dmaBufferActive==2)
	{
		// //Serial.println("on finit");
		i2sStop();
		return;
	}
}
*/

void setLedBuff(CRGB* newBuff)
{
	leds = newBuff;
}

void reset()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
	i2s->lc_conf.val |= lc_conf_reset_flags;
	i2s->lc_conf.val &= ~lc_conf_reset_flags;

	const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
	i2s->conf.val |= conf_reset_flags;
	i2s->conf.val &= ~conf_reset_flags;
	while (i2s->state.rx_fifo_reset_back)
		;
}

void i2sStop()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	esp_intr_disable(interruptHandle);
	reset();
	i2s->conf.rx_start = 0;
	i2s->conf.tx_start = 0;
}

void startTX()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	//ESP_LOGE(TAG_I2s,"I2S TX");
	////Serial.println("on transmet");
	esp_intr_disable(interruptHandle);
	reset();
	dmaBufferActive = 0;
	//ESP_LOGE(TAG_I2s,"Sample count %d",sampleCount(0));
	i2s->lc_conf.val=I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
	i2s->out_link.addr = (uint32_t) & (dmaBuffers[0]->descriptor);
	i2s->out_link.start = 1;
	////vTaskDelay(5);
	i2s->int_clr.val = i2s->int_raw.val;
	i2s->int_ena.val = 0;
	i2s->int_ena.out_eof = 1;
	// //vTaskDelay(5);
	i2s->int_ena.out_dscr_err = 1;
	//enable interrupt
	////vTaskDelay(5);
	esp_intr_enable(interruptHandle);
	// //vTaskDelay(5);
	//start transmission
	i2s->conf.tx_start = 1;
}

void startRX()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	//ESP_LOGE(TAG_I2s,"I2S RX");
	esp_intr_disable(interruptHandle);
	reset();
	dmaBufferActive = 0;
	//ESP_LOGE(TAG_I2s,"Sample count %d",sampleCount(0));
	i2s->rx_eof_num = sampleCount(0);
	i2s->in_link.addr = (uint32_t) & (dmaBuffers[0]->descriptor);
	i2s->in_link.start = 1;
	////vTaskDelay(5);
	i2s->int_clr.val = i2s->int_raw.val;
	i2s->int_ena.val = 0;
	i2s->int_ena.in_done = 1;
	esp_intr_enable(interruptHandle);
	// //vTaskDelay(5);
	i2s->conf.rx_start = 1;
}

void resetDMA()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	i2s->lc_conf.in_rst = 1;
	i2s->lc_conf.in_rst = 0;
	i2s->lc_conf.out_rst = 1;
	i2s->lc_conf.out_rst = 0;
}

void resetFIFO()
{
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	i2s->conf.rx_fifo_reset = 1;
	i2s->conf.rx_fifo_reset = 0;
	i2s->conf.tx_fifo_reset = 1;
	i2s->conf.tx_fifo_reset = 0;
}



bool initParallelOutputMode(const int *pinMap)
{
	 long sampleRate = 100000;
	 int baseClock = -1;
	 int wordSelect = -1;
	volatile i2s_dev_t *i2s;
	i2s = i2sDevices[i2sIndex];
	//ESP_LOGE(TAG_I2s,"in d");
	//route peripherals
	//in parallel mode only upper 16 bits are interesting in this case
	const int deviceBaseIndex[] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
	const int deviceClockIndex[] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
	const int deviceWordSelectIndex[] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
	const periph_module_t deviceModule[] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
	//works only since indices of the pads are sequential
	for (int i = 0; i < 24; i++)
		if (pinMap[i] > -1)
		{
			PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pinMap[i]], PIN_FUNC_GPIO);
			gpio_set_direction((gpio_num_t)pinMap[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
			//pinMode(pinMap[i],OUTPUT);
			gpio_config_t io_conf;
			io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
			//set as output mode
			io_conf.mode = GPIO_MODE_OUTPUT;
			//bit mask of the pins that you want to set,e.g.GPIO18/19
			io_conf.pin_bit_mask = (1ULL<<pinMap[i]);
			//disable pull-down mode
			io_conf.pull_down_en = 1;
			//disable pull-up mode
			io_conf.pull_up_en = 0;
			//configure GPIO with the given settings
			gpio_config(&io_conf);
			gpio_set_level(pinMap[i], 0);
			gpio_matrix_out(pinMap[i], deviceBaseIndex[i2sIndex] + i, false, false);
		}
	if (baseClock > -1)
		gpio_matrix_out(baseClock, deviceClockIndex[i2sIndex], false, false);
	if (wordSelect > -1)
		gpio_matrix_out(wordSelect, deviceWordSelectIndex[i2sIndex], false, false);
	//ESP_LOGE(TAG_I2s,"in d1");
	//enable I2S peripheral
	periph_module_enable(deviceModule[i2sIndex]);
	//ESP_LOGE(TAG_I2s,"in d2");
	//reset i2s
	i2s->conf.tx_reset = 1;
	i2s->conf.tx_reset = 0;
	i2s->conf.rx_reset = 1;
	i2s->conf.rx_reset = 0;
	//ESP_LOGE(TAG_I2s,"in d3");
	resetFIFO();
	resetDMA();
	//ESP_LOGE(TAG_I2s,"in d4");
	//parallel mode
	i2s->conf2.val = 0;
	i2s->conf2.lcd_en = 1;
	i2s->conf2.lcd_tx_wrx2_en = 0; // HN
	i2s->conf2.lcd_tx_sdx2_en = 0; // HN
	//from technical datasheet figure 64
	//i2s->conf2.lcd_tx_sdx2_en = 1;
	//i2s->conf2.lcd_tx_wrx2_en = 0;

	i2s->sample_rate_conf.val = 0;
	i2s->sample_rate_conf.tx_bits_mod =32; //16
	//ESP_LOGE(TAG_I2s,"in d5");
	//clock setup
	//xtal is 40M
	//chip revision 0
	//fxtal * (sdm2 + 4) / (2 * (odir + 2))
	//chip revision 1
	//fxtal * (sdm2 + (sdm1 / 256) + (sdm0 / 65536) + 4) / (2 * (odir + 2))
	//fxtal * (sdm2 + (sdm1 / 256) + (sdm0 / 65536) + 4) needs to be btween 350M and 500M
	//rtc_clk_apll_enable(enable, sdm0, sdm1, sdm2, odir);
	//                           0-255 0-255  0-63  0-31
	//sdm seems to be simply a fixpoint number with 16bits frational part
	//0xA7f00 is the highest value I was able to use. it's just shy of 580MHz. That's a max freq of 145MHz
	//freq = 40000000L * (4 + sdm) / 4
	//sdm = freq / 10000000L - 4;
	int rate=sampleRate;
	int bits=16;
	int clkmInteger, clkmDecimals, bck = 0;
	double denom = (double)1 / 63;
	int channel = 2;

	//    double mclk;
	double clkmdiv;

	int factor;

	if (bits == 8) {
		factor = 120;
	} else {
		factor = (256 % bits) ? 384 : 256;
	}

	clkmdiv = (double)I2S_BASE_CLK / (rate* factor);
	if (clkmdiv > 256) {
		//ESP_LOGE(TAG_I2s,"rate is too low");
		return ESP_FAIL;
	}
	//I2S[bus_num].rate = rate;

	clkmInteger = clkmdiv;
	clkmDecimals = ((clkmdiv - clkmInteger) / denom);

	if (bits == 8) {
		//        mclk = rate* factor;
		bck = 60;
		bits = 16;
	} else {
		//        mclk = (double)clkmInteger + (denom* clkmDecimals);
		bck = factor/(bits* channel);
	}
	//ESP_LOGE(TAG_I2s,"%u",(unsigned int)sampleRate);
	//long freq = min(sampleRate, 36249999L) * 8; //there are two 1/2 factors in the I2S pipeline for the frequency and another I missed
	//long sdm = 320000;//long(freq * 0.0065536) - 0x40000;
	////Serial.println(sdm);
	/*long freq = sampleRate * 4; //there are two 1/2 factors in the I2S pipeline for the frequency and another I missed
    int sdm, sdmn;
    int odir = -1;
    do
    {
        odir++;
        sdm = long((double(freq) / (20000000. / (odir + 2))) * 0x10000) - 0x40000;
        sdmn = long((double(freq) / (20000000. / (odir + 2 + 1))) * 0x10000) - 0x40000;
    }while(sdm < 0x8c0ecL && odir < 31 && sdmn < 0xA1fff); //0xA7fffL doesn't work on all mcus
        //    sdm = 0x8c0ecL;
    //    odir = 3;
    rtc_clk_apll_enable(true, sdm & 255, (sdm >> 8) & 255, sdm >> 16, odir);*/
	//ESP_LOGE(TAG_I2s,"in d6");
	//rtc_clk_apll_enable(true, sdm & 255, (sdm >> 8) & 255, sdm >> 16, 0);
	//ESP_LOGE(TAG_I2s,"in d7");
	/*i2s->clkm_conf.val = 0;
	i2s->clkm_conf.clka_en = 0;
    i2s->clkm_conf.clkm_div_num = 40000000L/sampleRate; //; clkmInteger;//80000000L/32000000L;//clkmInteger;//80000000L/100000; //clockN;
	i2s->clkm_conf.clkm_div_a = 3;   //clockA;
    i2s->clkm_conf.clkm_div_b = 2;   //clockB;
    i2s->sample_rate_conf.tx_bck_div_num =3;  //bck;*/

	long freq = sampleRate ;//* 2 * (16 / 8);
	int sdm, sdmn;
	int odir = -1;
	do
	{
        odir++;
        sdm = (long)(((double)(freq) / (20000000. / (odir + 2))) * 0x10000) - 0x40000;
        sdmn = (long)(((double)(freq) / (20000000. / (odir + 2 + 1))) * 0x10000) - 0x40000;
	}while(sdm < 0x8c0ecL && odir < 31 && sdmn < 0xA1fff); //0xA7fffL doesn't work on all mcus
	//DEBUG_PRINTLN(sdm & 255);
	//DEBUG_PRINTLN((sdm >> 8) & 255);
	//DEBUG_PRINTLN(sdm >> 16);
	//DEBUG_PRINTLN(odir);
	//sdm = 0xA1fff;
	//odir = 0;
	if(sdm > 0xA1fff) sdm = 0xA1fff;
	// rtc_clk_apll_enable(true, sdm & 255, (sdm >> 8) & 255, sdm >> 16, odir);
	//ESP_LOGE(TAG_I2s,"%d",(int)sdm & 255);
	//ESP_LOGE(TAG_I2s,"%d",(int)(sdm >> 8) & 255);
	//ESP_LOGE(TAG_I2s,"%d",(int)sdm >> 16);
	//ESP_LOGE(TAG_I2s,"%d",(int)odir);
	// rtc_clk_apll_enable(true, 1, 0,0 , 0);
	i2s->clkm_conf.val = 0;
	i2s->clkm_conf.clka_en = 0;
	//config 3,2 Mhz
	//i2s->clkm_conf.clkm_div_num = 25;//33;//1; //clockN;
	//i2s->clkm_conf.clkm_div_a = 1;   //clockA;
	// i2s->clkm_conf.clkm_div_b =0;   //clockB;

	//config2,4Mhz
	i2s->clkm_conf.clkm_div_num = 33;//33;//1; //clockN;
	i2s->clkm_conf.clkm_div_a = 3;   //clockA;
	i2s->clkm_conf.clkm_div_b =1;

	i2s->sample_rate_conf.tx_bck_div_num = 1;

	//ESP_LOGE(TAG_I2s,"div_b %d\n",clkmDecimals);
	//ESP_LOGE(TAG_I2s,"bck_div_num %d\n",bck);
	i2s->fifo_conf.val = 0;
	i2s->fifo_conf.tx_fifo_mod_force_en = 1;
	i2s->fifo_conf.tx_fifo_mod = 3;  //byte packing 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3,
	i2s->fifo_conf.tx_data_num = 32; //fifo length
	i2s->fifo_conf.dscr_en = 1;		//fifo will use dma

	i2s->conf1.val = 0;
	i2s->conf1.tx_stop_en = 0;
	i2s->conf1.tx_pcm_bypass = 1;

	i2s->conf_chan.val = 0;
	i2s->conf_chan.tx_chan_mod = 2;

	//high or low (stereo word order)
	i2s->conf.tx_right_first = 0;//1;

	i2s->timing.val = 0;

	//clear serial mode flags
	/*i2s->conf.tx_msb_right = 1;
	i2s->conf.tx_msb_shift = 0;
	i2s->conf.tx_mono = 1;
	i2s->conf.tx_short_sync = 0;*/

	//allocate disabled i2s interrupt
	const int interruptSource[] = {ETS_I2S0_INTR_SOURCE, ETS_I2S1_INTR_SOURCE};
	//ESP_LOGE(TAG_I2s,"in d5");
	esp_intr_alloc(interruptSource[i2sIndex], ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM, &interrupt, &callback, &interruptHandle);
	//ESP_LOGE(TAG_I2s,"in d6");
	return true;
}

/// simple ringbuffer of blocks of size bytes each
void allocateDMABuffers(int count, int bytes)
{
	dmaBufferCount = count;
	dmaBuffers = (DMABufferI2S **)malloc(sizeof(DMABufferI2S *) * dmaBufferCount);
	if (!dmaBuffers)
		//ESP_LOGE(TAG_I2s,"Failed to allocate DMABuffer array");
	for (int i = 0; i < dmaBufferCount; i++)
	{
		dmaBuffers[i] = allocate(bytes,true,i);
		if (i)
			next(dmaBuffers[i],i - 1);
	}
	next(dmaBuffers[0],dmaBufferCount - 1);
}

void deleteDMABuffers()
{
	if (!dmaBuffers)
		return;
	for (int i = 0; i < dmaBufferCount; i++)
		destroy(i);
	free(dmaBuffers);
	dmaBuffers = 0;
	dmaBufferCount = 0;
}

void stop()
{
	stopSignal = true;
	while (stopSignal);
}


void setBrightness(uint8_t b)
{
	brigthness=255/b;
}

void initled(CRGB *m_leds,int * m_Pins,int m_num_strips,int m_nun_led_per_strip)
{
	int m_ledType=1;
	//initialization of the pins
	int m_dmaBufferCount=2;
	leds=m_leds;
	nun_led_per_strip=m_nun_led_per_strip;
	num_strips=m_num_strips;
	Pins=m_Pins;
	brigthness=2;
	runningPixel=false;
	ledType=m_ledType;
	int pinMap[24];
	for(int i=0;i<24;i++)
	{
		if(i>=num_strips)
		{
			pinMap[i]=-1;
		}
		else
		{
			if(Pins[i]>=0 && Pins[i]<=33)
				pinMap[i]=Pins[i]; //we could add remove 6,7,8,9,10,11,12
				else
					pinMap[i]=-1;
		}
	}

	dmaBufferCount=m_dmaBufferCount;
	dmaBuffers = (DMABufferI2S **)malloc(sizeof(DMABufferI2S *) * dmaBufferCount);
	if(!dmaBuffers)
	{
		//ESP_LOGE(TAG_I2s,"Not enough memory soory...");
		return;
	}
	initParallelOutputMode(pinMap);

	for (int i = 0; i < dmaBufferCount; i++)
	{
		//this->dmaBuffers[i] = DMABuffer::allocate(96); //we need 24 bit * 4 pulses per bit
		dmaBuffers[i] = allocate(24*3,true,i); //we need 24 bit * 4 pulses per bit
		if (i)
			next(dmaBuffers[i],i - 1);
//			dmaBuffers[i - 1]->next(this->dmaBuffers[i]);
		empty((uint32_t*)dmaBuffers[i]->buffer); //we do get the buffer prefilled with the 0 at the end and the 1
	}
	next(dmaBuffers[0],dmaBufferCount - 1);
//	this->dmaBuffers[dmaBufferCount - 1]->next(this->dmaBuffers[0]);

	//ESP_LOGE(TAG_I2s,"Controller ready");
}

void empty ( uint32_t *buf)
{
	for(int i=0;i<24;i++)
	{
		//buf[4*i]=0xffffffff;
		//buf[4*i+3]=0;
		buf[3*i]=0xffffffff;
		buf[3*i+2]=0;
	}
}

void  transpose24x1_noinline(unsigned char *A, uint32_t *B) {
	uint32_t  x, y, x1,y1,t,x2,y2;

	y = *(unsigned int*)(A);
	x = *(unsigned int*)(A+4);
	y1 = *(unsigned int*)(A+8);
	x1 = *(unsigned int*)(A+12);

	y2 = *(unsigned int*)(A+16);
	x2 = *(unsigned int*)(A+20);


	// pre-transform x
	t = (x ^ (x >> 7)) & 0x00AA00AA;  x = x ^ t ^ (t << 7);
	t = (x ^ (x >>14)) & 0x0000CCCC;  x = x ^ t ^ (t <<14);

	t = (x1 ^ (x1 >> 7)) & 0x00AA00AA;  x1 = x1 ^ t ^ (t << 7);
	t = (x1 ^ (x1 >>14)) & 0x0000CCCC;  x1 = x1 ^ t ^ (t <<14);

	t = (x2 ^ (x2 >> 7)) & 0x00AA00AA;  x2 = x2 ^ t ^ (t << 7);
	t = (x2 ^ (x2 >>14)) & 0x0000CCCC;  x2 = x2 ^ t ^ (t <<14);

	// pre-transform y
	t = (y ^ (y >> 7)) & 0x00AA00AA;  y = y ^ t ^ (t << 7);
	t = (y ^ (y >>14)) & 0x0000CCCC;  y = y ^ t ^ (t <<14);

	t = (y1 ^ (y1 >> 7)) & 0x00AA00AA;  y1 = y1 ^ t ^ (t << 7);
	t = (y1 ^ (y1 >>14)) & 0x0000CCCC;  y1 = y1 ^ t ^ (t <<14);

	t = (y2 ^ (y2 >> 7)) & 0x00AA00AA;  y2 = y2 ^ t ^ (t << 7);
	t = (y2 ^ (y2 >>14)) & 0x0000CCCC;  y2 = y2 ^ t ^ (t <<14);

	// final transform
	t = (x & 0xF0F0F0F0) | ((y >> 4) & 0x0F0F0F0F);
	y = ((x << 4) & 0xF0F0F0F0) | (y & 0x0F0F0F0F);
	x = t;

	t = (x1 & 0xF0F0F0F0) | ((y1 >> 4) & 0x0F0F0F0F);
	y1 = ((x1 << 4) & 0xF0F0F0F0) | (y1 & 0x0F0F0F0F);
	x1 = t;

	t = (x2 & 0xF0F0F0F0) | ((y2 >> 4) & 0x0F0F0F0F);
	y2 = ((x2 << 4) & 0xF0F0F0F0) | (y2 & 0x0F0F0F0F);
	x2 = t;



	*((uint32_t*)B) = (uint32_t)((y & 0xff) |  (  (y1 & 0xff) << 8 )  |  (  (y2 & 0xff) << 16 ) )   ;
	*((uint32_t*)(B+1)) = (uint32_t)(((y & 0xff00) |((y1&0xff00) <<8)  |((y2&0xff00) <<16)  )>>8    );
	*((uint32_t*)(B+2)) =(uint32_t)(  (  (y & 0xff0000) >>16)|((y1&0xff0000) >>8)   |((y2&0xff0000))     );
	*((uint32_t*)(B+3)) = (uint32_t)(((y & 0xff000000) >>16 |((y1&0xff000000)>>8 ) |((y2&0xff000000) )  )>>8);

	*((uint32_t*)B+4) =(uint32_t)( (x & 0xff) |((x1&0xff) <<8)  |((x2&0xff) <<16) );
	*((uint32_t*)(B+5)) = (uint32_t)(((x & 0xff00) |((x1&0xff00) <<8)    |((x2&0xff00) <<16)    )>>8);
	*((uint32_t*)(B+6)) = (uint32_t)(  (  (x & 0xff0000) >>16)|((x1&0xff0000) >>8)   |((x2&0xff0000))     );
	*((uint32_t*)(B+7)) = (uint32_t)(((x & 0xff000000) >>16 |((x1&0xff000000)>>8 )    |((x2&0xff000000) )    )>>8);

}


void  putPixelinBuffer(Lines *pixel,uint32_t *buf)
{
	Lines b2,b;

	for (int color=0;color<3;color++)
	{

		b=pixel[color];
		b2=b;
		transpose24x1_noinline(b.bytes,b2.shorts);
		for(int i=0;i<8;i++)
		{
			//buf[color*32+4*i+1]=(b2.shorts[7-i] << 8); //the <<8 is to free up the first byte
			// buf[color*32+4*i+2]=ledType*(b2.shorts[7-i] << 8);

			buf[color*24+3*i+1]=(b2.shorts[7-i] << 8);
		}
	}
}

void showPixels() {

	ledToDisplay=0;
	stopSignal=false;
	//pixelsToDisplay(allpixels);
	Lines firstPixel[3];
	Lines secondPixel[3];
	for(int i = 0; i < num_strips; i++) {
		firstPixel[0].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].g/brigthness;
		firstPixel[1].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].r/brigthness;
		firstPixel[2].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].b/brigthness;
	}


	ledToDisplay++;
	putPixelinBuffer(firstPixel,(uint32_t*)dmaBuffers[0]->buffer);

	for(int i = 0; i < num_strips; i++) {

		secondPixel[0].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].g/brigthness;
		secondPixel[1].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].r/brigthness;
		secondPixel[2].bytes[i] = leds[ledToDisplay+nun_led_per_strip*i].b/brigthness;

	}


	ledToDisplay++;
	putPixelinBuffer(secondPixel,(uint32_t*)dmaBuffers[1]->buffer);
	dmaBufferActive=0;
	runningPixel=true;
	startTX();
	while(runningPixel==true);
	//delay(0);
	//ESP_LOGE(TAG_I2s,"pixel done");


}
