#include "I2S.h"

//example use
void main()
{
	int Pins[2]={19};
	CRGB leds[24]; //24 led
	I2S(0);
	initled(leds,Pins,1,24); //1 strip 24 led
	for(int i = 0 ; i < 24 ; i++)
	{
		CRGB c = {.r=0,.g=0xff,.b=0};
		leds[i] = c;
	}
	
	//setLedBuff(leds);
	showPixels();
}
