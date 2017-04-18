#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f0xx.h"
#include "own_std.h"

#define LED_ON()  {} //{GPIOF->BSRR = 1UL;}
#define LED_OFF() {GPIOF->BSRR = 1UL<<16;}

#define EN_GATE()  {GPIOA->BSRR = 1UL<<11;}
#define DIS_GATE() {GPIOA->BSRR = 1UL<<(11+16);}

#define EN_DCCAL()  {GPIOA->BSRR = 1UL<<12;}
#define DIS_DCCAL() {GPIOA->BSRR = 1UL<<(12+16);}

#define HALL_C() (GPIOB->IDR & (1UL<<8))
#define HALL_B() (GPIOB->IDR & (1UL<<7))
#define HALL_A() (GPIOB->IDR & (1UL<<6))

#define OVERCURR() (!(GPIOA->IDR & (1UL<<14)))

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

void delay_us(uint32_t i) __attribute__((section(".flasher")));
void delay_us(uint32_t i)
{
	if(i==0) return;
	i *= 7;
	i -= 7;
	while(i--)
		__asm__ __volatile__ ("nop");
}

void delay_ms(uint32_t i) __attribute__((section(".flasher")));
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

// 1024 equals 3V3; after gain=40, it's 82.5mV; with 1mOhm, it's 82.5A.
// i.e., 81mA per unit.

volatile int neg_curr_lim = -20;
volatile int pos_curr_lim = 20;

void set_curr_lim(int ma)
{
	// Protection limit set at 1.25x soft limit + 2A constant extra.
	set_prot_lim(((ma*5)>>2)+2000);

	neg_curr_lim = ma/-81;
	pos_curr_lim = ma/81;
}

volatile int new_mult=0;

volatile int timeout = 50;

volatile int reverse = 0;

#define PWM_MID 512
#define MIN_FREQ 1*65536
#define MAX_FREQ 100*65536

volatile uint32_t freq = MIN_FREQ*10;
volatile uint32_t hall_aim = 5000*65536;


void forward(uint16_t speed)
{
	EN_GATE();
	timeout = 50;
	if(speed > 400)
		speed = 400;

	reverse = 0;
	if(bldc)
	{
		new_mult = speed>>1;		
	}
	else
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = speed;
	}
}

void backward(uint16_t speed)
{
	EN_GATE();
	timeout = 50;
	if(speed > 400)
		speed = 400;

	reverse = 1;
	if(bldc)
	{
		new_mult = speed>>1;
	}
	else
	{
		TIM1->CCR1 = speed;
		TIM1->CCR2 = 0;
	}
}

typedef struct
{
	int16_t cur_c;
	int16_t cur_b;
} adc_data_t;

int16_t dccal_c;
int16_t dccal_b;


#define NUM_ADC_SAMPLES 2
adc_data_t latest_adc[2];

//volatile uint16_t dbg = 0;
//volatile uint16_t dbg_in = 0;

volatile uint8_t sin_mult;

volatile int latest_current;

extern void flasher() __attribute__((section(".flasher")));

void run_flasher()
{
	DIS_GATE();
	__disable_irq();
	// Reconfig SPI to run without interrupts.
	SPI1->CR1 = 0; // Disable SPI
	delay_us(10);
	SPI1->CR2 = 0b1111UL<<8 /*16-bit*/;
	SPI1->CR1 = 1UL<<6; // Enable SPI

	flasher();
}

void spi_inthandler()
{
	static int flasher_sequence = 0;
	static int send_cnt = 1;
	uint16_t msg = SPI1->DR;
	uint16_t cmd = (msg&(0b111111<<10)) >> 10;
	uint16_t param = msg&0x3FF;

	if(flasher_sequence) flasher_sequence--;

	switch(cmd)
	{
		case 11:
		forward(param);
		break;

		case 12:
		backward(param);
		break;

		case 60:
		if(param == 234 && flasher_sequence == 0)
			flasher_sequence=2;
		break;

		case 55:
		if(param == 345 && flasher_sequence)
			run_flasher();
		break;

		default:
		break;
	}
	// Write some new stuff for the next round.

	uint16_t data_out = 0;
	switch(send_cnt)
	{
		case 1:
		data_out = latest_current&0x3FF;
		break;
		case 2:
		data_out = (freq>>15)&0x3FF;
		break;
		default:
		break;
	}

	SPI1->DR = send_cnt<<10 | data_out;
	send_cnt++;
	if(send_cnt > 2)
		send_cnt = 1;

}

void run_dccal()
{
	int i;
	EN_DCCAL();
	int c = 0;
	int b = 0;
	delay_ms(10);
	for(i = 0; i < 128; i++)
	{
		while(!(DMA1->ISR & 1)) ; // Wait for DMAch1 transfer complete
		b += latest_adc[0].cur_b + latest_adc[1].cur_b;
		c += latest_adc[0].cur_c + latest_adc[1].cur_c;
	}
	DIS_DCCAL();
	dccal_b = b>>8;
	dccal_c = c>>8;
	delay_ms(10);
}

/*
	Sine table is 256 long. It is pointed to by the MSByte of uint32_t which rolls over, for
	good resolution of the fundamental frequency.
*/

volatile uint32_t sine_loc = 0;


volatile int d_shift = 13;
volatile int pi_shift = 32;
int hall_aim_f_comp = 0;

#define PHSHIFT_1 (1431655765UL)
#define PHSHIFT_2 (2863311531UL)

volatile int led_short = 0;

void tim1_inthandler()
{
	static int prev_halls[3];
	static int ignore_halls[3] = {0,0,0};
	static int prev_err;
	static int currlim_mult = 255;
	static int cnt = 0;
	static int prev_hall_cnt;


	TIM1->SR = 0; // Clear interrupt flags
	uint32_t loc = sine_loc;

	int idxa = ((sine_loc)&0xff000000)>>24;
	int idxb = ((sine_loc+PHSHIFT_1)&0xff000000)>>24;
	int idxc = ((sine_loc+PHSHIFT_2)&0xff000000)>>24;

	uint8_t mult = ((uint16_t)sin_mult * (uint16_t)currlim_mult)>>8;
	TIM1->CCR1 = (PWM_MID) + ((mult*sine[idxa])>>14);
	TIM1->CCR2 = (PWM_MID) + ((mult*sine[idxb])>>14);
	TIM1->CCR3 = (PWM_MID) + ((mult*sine[idxc])>>14);

	int halls[3];
	halls[0] = HALL_A();
	halls[1] = HALL_B();
	halls[2] = HALL_C();

//	int err = 0;
//	int derr = 0;

	cnt++;

	int f = freq;
	int i;

	int hall_aims[3];
	hall_aims[0] = hall_aim+PHSHIFT_2;
	hall_aims[1] = hall_aim;
	hall_aims[2] = hall_aim+PHSHIFT_1;

	if(ignore_halls[0]) ignore_halls[0]--;
	if(ignore_halls[1]) ignore_halls[1]--;
	if(ignore_halls[2]) ignore_halls[2]--;

	int f_tentative = (PHSHIFT_1)/((cnt-prev_hall_cnt));

	if(f_tentative < (MIN_FREQ)) f = (MIN_FREQ);

	for(i=0; i < 3; i++)
	{
		if(!halls[i] && prev_halls[i] && !ignore_halls[i]) // Got a pulse from one of the hall sensors.
		{
//			if(reverse)
//				err = (int)loc - (hall_aims[i] + hall_aim_f_comp*f);
//			else
//				err = (hall_aims[i] + hall_aim_f_comp*f) - (int)loc;
//			derr = (err - prev_err)>>d_shift; // D term
			ignore_halls[i] = 5; // ignore the said sensor for some time (for debouncing)

			loc = hall_aims[i];

			f = f_tentative;
			prev_hall_cnt = cnt;
			break; // Only one hall can trigger at once; otherwise is an error condition (todo: maybe check&recover)
		}
	}



//	if(derr > (MAX_FREQ)) derr = (MAX_FREQ);
//	if(derr < -(MAX_FREQ)) derr = -(MAX_FREQ);

//	f += (((int64_t)f * (int64_t)err)>>pi_shift) /* PI term */ + derr;
//	prev_err = err;

	if(f<(MIN_FREQ)) f = (MIN_FREQ);
	else if(f>(MAX_FREQ)) f = (MAX_FREQ);

	if(reverse)
		sine_loc = loc-f;
	else
		sine_loc = loc+f;
	freq = f;

	prev_halls[0] = halls[0];
	prev_halls[1] = halls[1];
	prev_halls[2] = halls[2];

	int current = latest_adc[1].cur_b-dccal_b; // Temporary solution, adc timing must be improved

	latest_current = current;

	if(OVERCURR())
	{
		LED_ON(); led_short = 0;
		currlim_mult-=50;
	}
	else if(current < neg_curr_lim || current > pos_curr_lim)
	{
		LED_ON(); led_short = 1;
		currlim_mult-=2;
	}
	else if(currlim_mult < 255)
		currlim_mult++;

	if(currlim_mult < 5) currlim_mult = 5;

//	dbg = current;

//	int vasen = latest_adc[0].cur_b-512;
//	int oikea = latest_adc[1].cur_b-512;

//	dbg = ((int8_t)(vasen>>2))<<8 | (int8_t)(oikea>>2);
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


	/*
		TIM1:

		OC4 is used to trigger the ADC.
	*/

	TIM1->CR1 = 0b01UL<<5 /*centermode 1*/;
	TIM1->CR2 = 0b010UL<<4 /*Update event sends trigger output TRGO*/;
//	TIM1->CR2 = 0b111UL<<4 /*OC4REF --> TRGO*/;

	TIM1->CCMR1 = 1UL<<3 /*OC1 Preload enable*/ | 0b110UL<<4 /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/| 0b110UL<<12 /*OC2 PWMmode 1*/;
	TIM1->CCMR2 = 1UL<<3 /*OC3 Preload enable*/ | 0b110UL<<4 /*OC3 PWMmode 1*/ |
	              0b011UL<<12 /*OC4 toggle-on-match mode*/;
	TIM1->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/;

	TIM1->ARR = 1024; // 23.4 kHz

	if(bldc)
		TIM1->CCER |= 1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;

	TIM1->CCR1 = 512;
	TIM1->CCR2 = 512;
	TIM1->CCR3 = 512;
	TIM1->CCR4 = 1020; // Generate the ADC trigger right after the start of the switch period.
	TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM1->EGR |= 1; // Generate Reinit+update
	TIM1->DIER = 1UL /*Update interrupt enable*/;
	TIM1->CR1 |= 1; // Enable the timer

	set_prot_lim(5000);
	DAC->CR |= 1; // enable, defaults good otherwise.

	SPI1->CR2 = 0b1111UL<<8 /*16-bit*/ | 1UL<<6 /*RX buffer not empty interrupt*/;
	SPI1->CR1 = 1UL<<6; // Enable SPI - zeroes good otherwise.

	// Enable DMA: channel 1 for ADC.
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(latest_adc);
	DMA1_Channel1->CNDTR = 2*NUM_ADC_SAMPLES;
	DMA1_Channel1->CCR = 0b011010110101000UL; // very high prio, 16b->16b, MemIncrement, circular, transfer error interrupt, enable.

	// Enable and self-calibrate ADC.
	ADC1->CFGR2 = 0b10UL << 30; // PCLK/4, 12 MHz clock
	delay_us(100);
	ADC1->CR |= ADC_CR_ADCAL;
	while((ADC1->CR & ADC_CR_ADCAL) != 0);

	ADC1->CFGR1 = ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN | 0b01UL<<10 /*10:HW trigger rising edge*/ | 0UL<<6 /*TIM1 trigger*/ | 01UL<<3 /*10-bit reso*/ | 1UL<<2 /*dir*/;
	ADC1->SMPR = 0b001UL; // Sampling time = 7.5 ADC clock cycles 

	ADC1->CHSELR = 1UL<<6 | 1UL<<5;
	DMA1_Channel1->CCR |= 1UL;
	ADC1->CR |= ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);

	ADC1->CR |= 1UL<<2; // start

	NVIC_EnableIRQ(SPI1_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	__enable_irq();
	EN_GATE();
	delay_ms(50);
	run_dccal();
	DIS_GATE();
	delay_ms(10);	
	LED_OFF();

	// todo: pullup in NSS (PA15)

	set_curr_lim(20000);

	int cnt = 0;
	while(1)
	{
		cnt++;
		if(led_short && cnt > 2)
		{
			LED_OFF();
			cnt = 0;
		}
		else if(cnt > 100)
		{
			LED_OFF();
			cnt = 0;
		}

		delay_ms(2);

		int mult = sin_mult;
		if(mult < new_mult) mult++;
		else if(mult > new_mult) mult--;
		sin_mult = mult;

		if(timeout) timeout--;
		else
		{
			new_mult = 0;
			sin_mult = 0;
			DIS_GATE();
		}

/*
		if(dbg_in == 'a')
		{
//			LED_ON();
			EN_GATE();
		}

		if(dbg_in == 's')
		{
//			LED_OFF();
			DIS_GATE();
			freq = MIN_FREQ;
			sin_mult = 40;
		}

		if(dbg_in == 'q') new_mult=10;
		if(dbg_in == 'w') new_mult=14;
		if(dbg_in == 'e') new_mult=20;
		if(dbg_in == 'r') new_mult=28;
		if(dbg_in == 't') new_mult=40;
		if(dbg_in == 'y') new_mult=56;
		if(dbg_in == 'u') new_mult=80;
		if(dbg_in == 'i') new_mult=110;
		if(dbg_in == 'o') new_mult=160;
		if(dbg_in == 'p') new_mult=220;

		int mult = sin_mult;
		if(mult < new_mult) mult++;
		else if(mult > new_mult) mult--;
		sin_mult = mult;

		if(dbg_in == '1') hall_aim=0*65536;
		if(dbg_in == '2') hall_aim=1000*65536;
		if(dbg_in == '3') hall_aim=2000*65536;
		if(dbg_in == '4') hall_aim=3000*65536;
		if(dbg_in == '5') hall_aim=4000*65536;
		if(dbg_in == '6') hall_aim=5000*65536;
		if(dbg_in == '7') hall_aim=6000*65536;
		if(dbg_in == '8') hall_aim=7000*65536;
		if(dbg_in == '9') hall_aim=8000*65536;
		if(dbg_in == '0') hall_aim=9000*65536;
		if(dbg_in == '+') hall_aim=10000*65536;

		if(dbg_in == 'Z') set_curr_lim(2000);
		if(dbg_in == 'X') set_curr_lim(4000);
		if(dbg_in == 'C') set_curr_lim(8000);
		if(dbg_in == 'V') set_curr_lim(16000);

		if(dbg_in == 'B') run_dccal();

		if(dbg_in == 'R') reverse = 1;
		if(dbg_in == 'F') reverse = 0;

//		dbg = (freq&0xffff0000)>>16;

//		dbg = 0;
//		if(HALL_A()) dbg |= 1;
//		if(HALL_B()) dbg |= 2;
//		if(HALL_C()) dbg |= 4;

//		if(ADC1->ISR & 2)
//			LED_ON();
*/

	}



}
