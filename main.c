#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f0xx.h"
#include "own_std.h"

#define LED_ON()  {GPIOF->BSRR = 1UL;}
#define LED_OFF() {GPIOF->BSRR = 1UL<<16;}

#define EN_GATE()  {GPIOA->BSRR = 1UL<<11;}
#define DIS_GATE() {GPIOA->BSRR = 1UL<<(11+16);}

#define EN_DCCAL()  {GPIOA->BSRR = 1UL<<12;}
#define DIS_DCCAL() {GPIOA->BSRR = 1UL<<(12+16);}

#define HALL_C() (GPIOB->IDR & (1UL<<8))
#define HALL_B() (GPIOB->IDR & (1UL<<7))
#define HALL_A() (GPIOB->IDR & (1UL<<6))

int bldc = 1;

const int sine[256] =
{
0,804,1607,2410,3211,4011,4807,5601,6392,7179,7961,8739,9511,10278,11038,11792,
12539,13278,14009,14732,15446,16150,16845,17530,18204,18867,19519,20159,20787,21402,22004,22594,
23169,23731,24278,24811,25329,25831,26318,26789,27244,27683,28105,28510,28897,29268,29621,29955,
30272,30571,30851,31113,31356,31580,31785,31970,32137,32284,32412,32520,32609,32678,32727,32757,
32767,32757,32727,32678,32609,32520,32412,32284,32137,31970,31785,31580,31356,31113,30851,30571,
30272,29955,29621,29268,28897,28510,28105,27683,27244,26789,26318,25831,25329,24811,24278,23731,
23169,22594,22004,21402,20787,20159,19519,18867,18204,17530,16845,16150,15446,14732,14009,13278,
12539,11792,11038,10278,9511,8739,7961,7179,6392,5601,4807,4011,3211,2410,1607,804,
0,-804,-1607,-2410,-3211,-4011,-4807,-5601,-6392,-7179,-7961,-8739,-9511,-10278,-11038,-11792,
-12539,-13278,-14009,-14732,-15446,-16150,-16845,-17530,-18204,-18867,-19519,-20159,-20787,-21402,-22004,-22594,
-23169,-23731,-24278,-24811,-25329,-25831,-26318,-26789,-27244,-27683,-28105,-28510,-28897,-29268,-29621,-29955,
-30272,-30571,-30851,-31113,-31356,-31580,-31785,-31970,-32137,-32284,-32412,-32520,-32609,-32678,-32727,-32757,
-32767,-32757,-32727,-32678,-32609,-32520,-32412,-32284,-32137,-31970,-31785,-31580,-31356,-31113,-30851,-30571,
-30272,-29955,-29621,-29268,-28897,-28510,-28105,-27683,-27244,-26789,-26318,-25831,-25329,-24811,-24278,-23731,
-23169,-22594,-22004,-21402,-20787,-20159,-19519,-18867,-18204,-17530,-16845,-16150,-15446,-14732,-14009,-13278,
-12539,-11792,-11038,-10278,-9511,-8739,-7961,-7179,-6392,-5601,-4807,-4011,-3211,-2410,-1607,-804
};

void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 7;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i)
{
	while(i--)
	{
		delay_us(1000);
	}
}


void error(int code)
{
	int i = 0;
	while(1)
	{
		LED_ON();
		delay_ms(200);
		LED_OFF();
		delay_ms(200);
		i++;
		if(i == code)
		{
			delay_ms(800);
			i = 0;
		}
	}
}

void adc_int_handler()
{
	LED_ON();
	delay_ms(50);
	LED_OFF();
	ADC1->ISR |= 1UL<<7;
}

#define MAX_PROT_LIM 40000 // mA
#define MIN_PROT_LIM 1000 // mA
void set_prot_lim(int ma)
{
	// With Rds(on) = 6 mOhm
	// 1 mA = 0.006mV
	// 1 DAC unit = 3.3V/4096 = 0.806 mV
	// 1 DAC unit = 134.3 mA

	if(ma>MAX_PROT_LIM)
		ma = MAX_PROT_LIM;
	else if(ma<MIN_PROT_LIM)
		ma = MIN_PROT_LIM;

	ma/=134;
	DAC->DHR12R1 = ma;
}


void forward(uint16_t speed)
{
	if(speed > 400)
		speed = 400;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = speed;
}

void backward(uint16_t speed)
{
	if(speed > 400)
		speed = 400;
	TIM1->CCR1 = speed;
	TIM1->CCR2 = 0;
}

typedef struct
{
	int16_t cur_c;
	int16_t cur_b;
} adc_data_t;

int16_t dccal_c;
int16_t dccal_b;


#define NUM_ADC_SAMPLES 1
adc_data_t latest_adc;

volatile uint16_t dbg = 0;
volatile uint16_t dbg_in = 0;

void spi_inthandler()
{
	dbg_in = SPI1->DR & 0x7f;
	SPI1->DR = dbg;
/*	static int send_cnt = 1;
	uint16_t msg = SPI1->DR;
	uint16_t cmd = (msg&(0b111111<<10)) >> 10;
	uint16_t param = msg&0x3FF;
	switch(cmd)
	{
		case 11:
			forward(param);
		break;
		case 12:
			backward(param);
		break;
		default:
		break;
	}
	// Write some new stuff for the next round.

	uint16_t data_out = 0;
	switch(send_cnt)
	{
		case 1:
		data_out = latest_adc.cur_c - dccal_c + 512;
		break;
		case 2:
		data_out = latest_adc.cur_b - dccal_b + 512;
		break;
		default:
		break;
	}

	SPI1->DR = send_cnt<<10 | data_out;
	send_cnt++;
	if(send_cnt > 2)
		send_cnt = 1;
*/
}

void run_dccal()
{
	int i;
	EN_DCCAL();
	int c = 0;
	int b = 0;
	delay_ms(10);
	for(i = 0; i < 256; i++)
	{
		while(!(DMA1->ISR & 1)) ; // Wait for DMAch1 transfer complete
		b += latest_adc.cur_b;
		c += latest_adc.cur_c;
	}
	DIS_DCCAL();
	dccal_b = b>>8;
	dccal_c = c>>8;
	delay_ms(10);
}

int main()
{
	int i;
	RCC->CFGR = 0b1010UL << 18; // PLL x12 (because of /2 prediv)  --> 48 MHz
	RCC->CR |= 1UL << 24; // PLL on
	RCC->CFGR |= 0b10; // Change PLL to system clock

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL

	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.


	RCC->AHBENR |= 1UL<<22 /* PORTF clock */ | 1UL<<18 /* PORTB clock */ | 1UL<<17 /* PORTA clock */ | 1UL<<0 /*DMA*/;
	RCC->APB2ENR |= 1UL<<11 /*TIM1*/ | 1UL<<9 /*ADC*/ | 1UL<<12 /*SPI1*/;
	RCC->APB1ENR |= 1UL<<29 /*DAC*/;
	             // Mode:
		     // 00 = General Purpose In
	             // 01 = General Purpose Out
	             // 10 = Alternate Function (in/out controlled by peripheral)
	             // 11 = Analog in (to ADC)

	             // Speed:
	             // 00 = low, 01 = medium, 11 = high
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOA->MODER   = 0b10000001011010101011111111111111;
	GPIOA->OSPEEDR = 0b00000000000101010100000000000000;
	GPIOA->PUPDR   = 0b00010100000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b00000000000000000000101010001010;
	GPIOB->OSPEEDR = 0b00000000000000000000000100000101;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOF->MODER   = 0b00000000000000000000000000000001;
	GPIOF->OSPEEDR = 0b00000000000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |

	GPIOA->AFR[0] = 2UL<<28 /*PA7 = TIM1*/;
	GPIOA->AFR[1] = 2UL<<0 /*PA8 = TIM1*/ | 2UL<<4 /*PA9 = TIM1*/ | 2UL<<8 /*PA10 = TIM1*/;
	GPIOB->AFR[0] = 2UL<<0 /*PB0 = TIM1*/ | 2UL<<4 /*PB1 = TIM1*/;
	// SPI1 is at AF0, so defaults OK.



	TIM1->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM1->CR2 = 0b010UL<<4 /*Update event sends trigger output TRGO*/;

	TIM1->CCMR1 = 1UL<<3 /*OC1 Preload enable*/ | 0b110UL<<4 /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/| 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM1->CCMR2 = 1UL<<3 /*OC3 Preload enable*/ | 0b110UL<<4 /*OC3 PWMmode 1*/;
	TIM1->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/;

	TIM1->ARR = 1024; // 23.4 kHz

	if(bldc)
		TIM1->CCER |= 1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;

	TIM1->CCR1 = 512;
	TIM1->CCR2 = 512;
	TIM1->CCR3 = 512;
	TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM1->EGR |= 1; // Generate Reinit+update
	TIM1->CR1 |= 1; // Enable

	set_prot_lim(2000);
	DAC->CR |= 1; // enable, defaults good otherwise.

	SPI1->CR2 = 0b1111UL<<8 /*16-bit*/ | 1UL<<6 /*RX buffer not empty interrupt*/;
	SPI1->CR1 = 1UL<<6; // Enable SPI - zeroes good otherwise.

	// Enable DMA: channel 1 for ADC.
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(&latest_adc);
	DMA1_Channel1->CNDTR = 2*NUM_ADC_SAMPLES;
	DMA1_Channel1->CCR = 0b011010110101000UL; // very high prio, 16b->16b, MemIncrement, circular, transfer error interrupt, enable.

	// Enable and self-calibrate ADC.
	ADC1->CFGR2 = 0b10UL << 30; // PCLK/4, 12 MHz clock
	delay_us(100);
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0);

	ADC1->CFGR1 = ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN | 0b01UL<<10 /*01:HW trigger rising edge*/ | 0UL<<6 /*TIM1 trigger*/ | 01UL<<3 /*10-bit reso*/;
	ADC1->SMPR = 0b001UL; // Sampling time = 7.5 ADC clock cycles

	ADC1->CHSELR = 1UL<<6 | 1UL<<5;
	DMA1_Channel1->CCR |= 1UL;
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);

	ADC1->CR |= 1UL<<2; // start

	NVIC_EnableIRQ(SPI1_IRQn);
	__enable_irq();
//	EN_GATE();

	LED_OFF();

	delay_ms(100);
//	run_dccal();

	// todo: pullup in NSS (PA15)


	
	int sine_cnt = 0;
	int prev_hall = 0;
	int hall_setpoint = 80;
	int delay=1500;
	int power=1;
	while(1)
	{
/*		LED_ON();
		delay_ms(50);
		LED_OFF();
		delay_ms(50);
*/

		sine_cnt++;
		if(sine_cnt > 255)
			sine_cnt = 0;

		int a = sine_cnt;
		int b = sine_cnt+85;
		int c = sine_cnt+171;

		if(b>255) b-= 256;
		if(c>255) c-= 256;

		if(power == 1)
		{		
			TIM1->CCR1 = 512 + (sine[a]>>9);
			TIM1->CCR2 = 512 + (sine[b]>>9);
			TIM1->CCR3 = 512 + (sine[c]>>9);
		}
		else if(power == 2)
		{
			TIM1->CCR1 = 512 + (sine[a]>>8);
			TIM1->CCR2 = 512 + (sine[b]>>8);
			TIM1->CCR3 = 512 + (sine[c]>>8);
		}
		else if(power == 3)
		{
			TIM1->CCR1 = 512 + (sine[a]>>7);
			TIM1->CCR2 = 512 + (sine[b]>>7);
			TIM1->CCR3 = 512 + (sine[c]>>7);
		}
		else
		{		
			TIM1->CCR1 = 512 + (sine[a]>>10);
			TIM1->CCR2 = 512 + (sine[b]>>10);
			TIM1->CCR3 = 512 + (sine[c]>>10);
		}


		int hall_error = 0;
		int hall = HALL_B();
		if(!hall && prev_hall) // got the pulse
		{
			int hall_location = sine_cnt;
			hall_error = hall_location-hall_setpoint;
		}
		prev_hall = hall;

		if(delay < 200)
			delay += hall_error*1;
		else if(delay < 300)
			delay += hall_error*2;
		else if(delay < 400)
			delay += hall_error*3;
		else if(delay < 450)
			delay += hall_error*4;
		else if(delay < 500)
			delay += hall_error*5;
		else if(delay < 600)
			delay += hall_error*6;
		else if(delay < 800)
			delay += hall_error*10;
		else if(delay < 1200)
			delay += hall_error*15;
		else if(delay < 1900)
			delay += hall_error*25;
		else if(delay < 2500)
			delay += hall_error*40;
		else
			delay += hall_error*60;

		if(delay < 150) delay=150;
		if(delay > 4000) delay=4000;

		dbg = delay;

		delay_us(delay);

		if(dbg_in == 'a')
		{
			LED_ON();
			EN_GATE();
		}

		if(dbg_in == 's')
		{
			delay=1500;
			power=1;
			LED_OFF();
			DIS_GATE();
		}

		if(dbg_in == '1') hall_setpoint = 85-16;
		if(dbg_in == '2') hall_setpoint = 85-8;
		if(dbg_in == '3') hall_setpoint = 85;
		if(dbg_in == '4') hall_setpoint = 85+8;
		if(dbg_in == '5') hall_setpoint = 85+16;
		if(dbg_in == '7') {if(power==3) delay*=8; else if(power==2) delay*=4; else if(power==1) delay*=2;  power = 0;}
		if(dbg_in == '8') {if(power==3) delay*=4; else if(power==2) delay*=2; else if(power==0) delay/=2;  power = 1;}
		if(dbg_in == '9') {if(power==3) delay*=2; else if(power==0) delay/=4; else if(power==1) delay/=2;  power = 2;}
		if(dbg_in == '0') {if(power==0) delay/=8; else if(power==1) delay/=4; else if(power==2) delay/=2;  power = 3;}


	
/*		int val = 0;
		if(HALL_A()) val |= 1;
		if(HALL_B()) val |= 2;
		if(HALL_C()) val |= 4;
		dbg = val;
*/

//		if(ADC1->ISR & 2)
//			LED_ON();

/*		char buffer[200];
		char* buf = buffer;
		buf = o_str_append(buf, "kakka=");
		buf = o_utoa16(123, buf);
		buf = o_str_append(buf, "\n\r");*/
	}



}
