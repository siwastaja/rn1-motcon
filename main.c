#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "main.h"
#include "ext_include/stm32f0xx.h"
#include "own_std.h"
#include "flash.h"

//#define LEDS_FOR_DEBUG

#define PWM_MID 512
#define MIN_FREQ 1*65536
#define MAX_FREQ 100*65536

#ifdef LEDS_FOR_DEBUG
#define LED_ON()  {}
#define LED_OFF() {}
#define DLED_ON()  {GPIOF->BSRR = 1UL;}
#define DLED_OFF() {GPIOF->BSRR = 1UL<<16;}
#else
#define LED_ON()  {GPIOF->BSRR = 1UL;}
#define LED_OFF() {GPIOF->BSRR = 1UL<<16;}
#define DLED_ON()  {}
#define DLED_OFF() {}
#endif

#define EN_GATE()  {GPIOA->BSRR = 1UL<<11;}
#define DIS_GATE() {GPIOA->BSRR = 1UL<<(11+16);}

#define EN_DCCAL()  {GPIOA->BSRR = 1UL<<12;}
#define DIS_DCCAL() {GPIOA->BSRR = 1UL<<(12+16);}

#define HALL_C() (GPIOB->IDR & (1UL<<8))
#define HALL_B() (GPIOB->IDR & (1UL<<7))
#define HALL_A() (GPIOB->IDR & (1UL<<6))

#define HALL_ABC() ((GPIOB->IDR & (0b111<<6))>>6)

#define OVERCURR() (!(GPIOA->IDR & (1UL<<14)))


int bldc = 1;
int timing_shift = 5000*65536;

volatile int neg_curr_lim = -20;
volatile int pos_curr_lim = 20;
volatile int new_mult=0;
volatile int timeout = 50;
//volatile int do_resync = 10;

typedef struct __attribute__ ((packed))
{
	int16_t cur_c;
	int16_t cur_b;
} adc_data_t;

#define NUM_ADC_SAMPLES 2

volatile adc_data_t latest_adc[2];
int16_t dccal_c;
int16_t dccal_b;

#define SPI_DATAGRAM_LEN 8
typedef union 
{
	struct __attribute__ ((packed))
	{
		uint16_t status;
		int16_t speed;
		int16_t current;
		int16_t pos;
		uint16_t res4;
		uint16_t res5;
		uint16_t res6;
		uint16_t magic;
	};
	uint16_t u16[8];
	int16_t s16[8];
} spi_tx_t;

typedef union
{
	struct __attribute__ ((packed))
	{
		uint16_t state;
		int16_t speed;
		int16_t cur_limit;
		uint16_t res3;
		uint16_t res4;
		uint16_t res5;
		uint16_t res6;
		uint16_t magic;
	};
	uint16_t u16[8];
	int16_t s16[8];
} spi_rx_t;

volatile spi_tx_t spi_tx_data;
volatile spi_rx_t spi_rx_data;

volatile int latest_current;
volatile uint16_t lost_count, minfreq_count;


volatile int d_shift = 13;
volatile int pi_shift = 32;
int hall_aim_f_comp = 0;

volatile int led_short = 0;

/*
	three adjacent hall io pins (active low) form a 3-bit number which can be used to index hall_loc table,
	to determine the 6-step position of the rotor.
*/
const int hall_loc[8] =
{
 -1, // 000  ERROR
 1, // 001
 3, // 010
 2, // 011
 5, // 100
 0, // 101  Middle hall active - we'll call this zero position
 4, // 110
 -1  // 111  ERROR
};

#define PH120SHIFT (1431655765UL)  // 120 deg shift between the phases in the 32-bit range.
#define PH90SHIFT (1073741824UL)   // 90 deg shift between the phases in the 32-bit range.
#define PH45SHIFT (PH90SHIFT/2)

const int base_hall_aims[6] =
{
	0,
	PH120SHIFT/2,
	PH120SHIFT,
	PH120SHIFT+PH120SHIFT/2,
	PH120SHIFT*2,
	PH120SHIFT*2+PH120SHIFT/2
};


/*const int base_hall_aims[6] =
{
	PH120SHIFT,
	PH120SHIFT+PH120SHIFT/2,
	PH120SHIFT*2,
	PH120SHIFT*2+PH120SHIFT/2,
	0,
	PH120SHIFT/2
};*/


#define HALL_LOC() (hall_loc[HALL_ABC()])

/*
The sine table is indexed with 8 MSBs of uint32; this way, a huge resolution is implemented for the frequency,
since the frequency is a term added to the indexing each round.
*/

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
	DIS_GATE();
	__disable_irq();
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

/*
	Protection current limit: the FET driver measures the MOSFET Vds while the FETs are turned on
	and compares it to an analog input pin; in case of overcurrent, the driver instantly blanks the
	FETs out (and gives overcurrent signal to the MCU). This is only meant for protection; not normal
	operation.
*/
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


/*
	set_curr_lim sets the ADC based normal current limit, and also the protection limit.
	The protection limit is set considerably higher so that it should never be hit if the ADC-based
	works as it should.
*/

// 1024 equals 3V3; after gain=40, it's 82.5mV; with 1mOhm, it's 82.5A.
// i.e., 81mA per unit.
void set_curr_lim(int ma)
{
	// Protection limit set at 1.25x soft limit + 2A constant extra.
	set_prot_lim(((ma*5)>>2)+2000);

	neg_curr_lim = ma/-81;
	pos_curr_lim = ma/81;
}

void forward(uint16_t speed)
{
	EN_GATE();
	timeout = 200;
	if(speed > 400)
		speed = 400;

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
	timeout = 200;
	if(speed > 400)
		speed = 400;

	if(bldc)
	{
		new_mult = -1*(int)(speed>>1);
	}
	else
	{
		TIM1->CCR1 = speed;
		TIM1->CCR2 = 0;
	}
}

extern void flasher() __attribute__((section(".flasher")));

void run_flasher()
{
	DIS_GATE();
	__disable_irq();
	DMA1_Channel3->CCR = 0;
	DMA1_Channel2->CCR = 0;

	// Reconfig SPI to run without interrupts.
	// There seems to be no way of emptying the TX fifo, so we'll be happy sending some false data for the first two transactions.

	SPI1->CR1 = 0; // Disable SPI
	SPI1->CR2 = 0;

	spi1_empty_rx();

	delay_ms(1);
	// sclk must be at idle before enabling
	SPI1->CR2 = 0b1111UL<<8 /*16-bit*/;
	SPI1->CR1 = 1UL<<6; // Enable SPI

	flasher();
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

int floop_pi_term = 12;
int floop_d_term = 128;

void tim1_inthandler()
{
	static int reverse = 0;
	static int prev_reverse = 1;
	static int mult = 0;
	static int currlim_mult = 255;
	static int32_t cnt = 0;
	static int expected_next_hall_pos;
	static int32_t expected_next_hall_cnt;
	static int cnt_at_prev_hall_valid = 0;
	static int cnt_at_prev_hall;
	static int prev_hall_pos;
	static int resync = 5;
	static int16_t pos_info;
	static int prev_ferr = 0;
	static int f = 0;
	static int loc;
	static int pid_f_set;
	static int64_t pid_integral = 0;

//	static int accel_dir = 0;

	DLED_ON();
	TIM1->SR = 0; // Clear interrupt flags

	spi_tx_data.magic = HALL_ABC();

	int hall_pos = hall_loc[HALL_ABC()];
	if(hall_pos == -1) hall_pos = prev_hall_pos;

	if(pid_f_set < 0 ) reverse = 1; else reverse = 0;
	if(reverse != prev_reverse)
	{
		resync = 2;
		f = 0;
		pid_integral = 0;
	}
	prev_reverse = reverse;


	if(resync)
	{
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		cnt_at_prev_hall_valid = 0;
		expected_next_hall_cnt = cnt+22000;
		f = 0;
	}
	else if(cnt_at_prev_hall_valid && hall_pos == prev_hall_pos) // We haven't advanced a step yet, do sine interpolation
	{
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		// Interpolation freezes if the hall sync is not received on time.
		if((cnt - expected_next_hall_cnt) < 0) // this comparison works with counter wrapping around.
		{
			// Sine interpolation hasn't advanced to the next hall sync point, run it
//			if(reverse)
//				loc -= f;
//			else
//				loc += f;
		}
		else // We are overdue: recalculate the freq.
		{
			f = (PH120SHIFT)  /  ((cnt-cnt_at_prev_hall));
		}

		// TODO: prev_error feedforward to frequency generation.
	}
	else if(hall_pos == expected_next_hall_pos) 
	{
		// We have advanced one step in the right dirction - we can synchronize the sine phase, 
		// and calculate a valid frequency from the delta time.
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));

		if(cnt_at_prev_hall_valid)
		{
//			f = (reverse?(-PH120SHIFT):(PH120SHIFT))  /  ((cnt-cnt_at_prev_hall));
			f = (PH120SHIFT)  /  ((cnt-cnt_at_prev_hall));
			expected_next_hall_cnt = cnt+(cnt-cnt_at_prev_hall);
		}
		else
		{
			f = 0;
			expected_next_hall_cnt = cnt+22000; // at smallest possible freq, we'll wait for 0.5 sec for the next hall pulse.
		}

		cnt_at_prev_hall_valid = 1;
		cnt_at_prev_hall = cnt;
	}
	else // We are lost - synchronize the phase to the current hall status
	{
		lost_count++;
		loc = (base_hall_aims[hall_pos] + timing_shift + (reverse?(-PH90SHIFT):(PH90SHIFT)));
		cnt_at_prev_hall_valid = 0;
		expected_next_hall_cnt = cnt+22000;
		f = 0;
	}

	if(resync) resync--;

	if(hall_pos == expected_next_hall_pos)
	{
		if(reverse) pos_info--; else pos_info++;
		spi_tx_data.pos = pos_info;
	}

	prev_hall_pos = hall_pos;

	expected_next_hall_pos = hall_pos;
	if(!reverse) { expected_next_hall_pos++; if(expected_next_hall_pos > 5) expected_next_hall_pos = 0; }
	else         { expected_next_hall_pos--; if(expected_next_hall_pos < 0) expected_next_hall_pos = 5; }


	int idxa = (((uint32_t)loc)&0xff000000)>>24;
	int idxb = (((uint32_t)loc+PH120SHIFT)&0xff000000)>>24;
	int idxc = (((uint32_t)loc+2*PH120SHIFT)&0xff000000)>>24;

	// Ramp the setpoint.
	int next_pid_f_set = (spi_rx_data.speed*100);

	if(next_pid_f_set > pid_f_set) pid_f_set+=256;
	else if(next_pid_f_set < pid_f_set) pid_f_set-=256;
	if((next_pid_f_set - pid_f_set) > -256 && (next_pid_f_set - pid_f_set) < 256) next_pid_f_set = pid_f_set;

	int pid_f_meas = f>>3;
	if(reverse) pid_f_meas *= -1;

	int ferr;

	ferr = pid_f_set - pid_f_meas;

	spi_tx_data.speed = pid_f_meas>>8;
	spi_tx_data.res4 = pid_f_set>>8;
	spi_tx_data.res5 = ferr>>8;


#define PID_I_MAX 2000000000LL
#define PID_I_MIN -PID_I_MAX

	if(!(cnt & 15))
	{

	
		int dferr = (ferr - prev_ferr);
		prev_ferr = ferr;

		pid_integral += ferr;

		if(pid_integral > (PID_I_MAX)) pid_integral = PID_I_MAX;
		else if(pid_integral < (PID_I_MIN)) pid_integral = PID_I_MIN;

		mult = ((30*(int64_t)pid_f_set)>>10) /* feedforward */
			+ ((50*(int64_t)ferr)>>12) /* P */
			+ ((50*(int64_t)pid_integral)>>19)  /* I */
			+ ((50*(int64_t)dferr)>>12);

	}

	spi_tx_data.res6 = mult;
	
	int max_mult = 150*256;
	int min_mult = -150*256;

	if(mult > max_mult) mult = max_mult;
	if(mult < min_mult) mult = min_mult;
//	if(mult > -2*256 && mult < 2*256) resync = 5;

	int sin_mult = mult>>8;
//	spi_tx_data.res4 = sin_mult;
//	spi_tx_data.res5 = lost_count; 
//	spi_tx_data.res6 = minfreq_count; 

	if(reverse) sin_mult *= -1;	

	if(sin_mult < 0) sin_mult = 0;

#define MIN_MULT_OFFSET 9
	if(sin_mult != 0)
		sin_mult += MIN_MULT_OFFSET;

//	spi_tx_data.magic = lost_count; //sin_mult;

	uint8_t m = (sin_mult * currlim_mult)>>8; // 254 max
	TIM1->CCR1 = (PWM_MID) + ((m*sine[idxa])>>14); // 4 to 1019. 
	TIM1->CCR2 = (PWM_MID) + ((m*sine[idxb])>>14);
	TIM1->CCR3 = (PWM_MID) + ((m*sine[idxc])>>14);

	cnt++;

	int current = latest_adc[1].cur_b-dccal_b; // Temporary solution, adc timing must be improved

	latest_current = current;

	if(OVERCURR())
	{
//		LED_ON(); led_short = 0;
		currlim_mult-=50;
	}
	else if(current < neg_curr_lim || current > pos_curr_lim)
	{
//		LED_ON(); led_short = 1;
		currlim_mult-=2;
	}
	else if(currlim_mult < 255)
		currlim_mult++;

	if(currlim_mult < 5) currlim_mult = 5;




//	dbg = current;

//	int vasen = latest_adc[0].cur_b-512;
//	int oikea = latest_adc[1].cur_b-512;

//	dbg = ((int8_t)(vasen>>2))<<8 | (int8_t)(oikea>>2);
	DLED_OFF();
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

	// DMA: channel 3 for SPI TX.
	DMA1_Channel3->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t)(&spi_tx_data);
	DMA1_Channel3->CNDTR = SPI_DATAGRAM_LEN;
	DMA1_Channel3->CCR = 0b10<<12 /*hi prio*/ | 0b01<<10 /*16-bit mem*/ | 0b01<<8 /*16-bit periph*/ | 1<<7 /* mem increment*/ |
	                     1<<5 /*circular*/ | 1<<4 /*dir: mem->spi*/;

	// DMA: channel 2 for SPI RX
	DMA1_Channel2->CPAR = (uint32_t)&(SPI1->DR);
	DMA1_Channel2->CMAR = (uint32_t)(&spi_rx_data);
	DMA1_Channel2->CNDTR = SPI_DATAGRAM_LEN;
	DMA1_Channel2->CCR = 0b10<<12 /*hi prio*/ | 0b01<<10 /*16-bit mem*/ | 0b01<<8 /*16-bit periph*/ | 1<<7 /* mem increment*/ |
	                     1<<5 /*circular*/ | 0<<4 /*dir: spi->mem*/;

	delay_ms(100); // let the main cpu boot, so that nCS is definitely up.

	DMA1_Channel3->CCR |= 1; // enable
	DMA1_Channel2->CCR |= 1; // enable

	SPI1->CR2 = 0b1111UL<<8 /*16-bit*/ | 1UL<<1 /* TX DMA enable */ | 1UL<<0 /* RX DMA enable*/;
	SPI1->CR1 = 1UL<<6; // Enable SPI - zeroes good otherwise.


	// DMA: channel 1 for ADC.
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
	DMA1_Channel1->CMAR = (uint32_t)(latest_adc);
	DMA1_Channel1->CNDTR = 2*NUM_ADC_SAMPLES;
	DMA1_Channel1->CCR = 0b011010110101000UL; // very high prio, 16b->16b, MemIncrement, circular, transfer error interrupt

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

	set_curr_lim(15000);

	int cnt = 0;
	while(1)
	{
		cnt++;
		if(led_short && cnt > 4)
		{
			LED_OFF();
			cnt = 0;
		}
		else if(cnt > 200)
		{
			LED_OFF();
			cnt = 0;
		}

		delay_ms(1);

		if(	spi_rx_data.u16[0] == 0xfaaa &&
			spi_rx_data.u16[1] == 0x1234 &&
			spi_rx_data.u16[2] == 0xabcd &&
			spi_rx_data.u16[3] == 0x420b &&
			spi_rx_data.u16[4] == 0xacdc &&
			spi_rx_data.u16[5] == 0xabba &&
			spi_rx_data.u16[6] == 0x1337 &&
			spi_rx_data.u16[7] == 0xacab)
		{
			run_flasher();
		}


		switch(spi_rx_data.state & 0b111)
		{
			case 0:
			case 1:
			DIS_GATE();
			break;

			case 2:
			case 3:
			case 4:
			case 5:
			EN_GATE();
			break;


			default: break;
		}

//		timing_shift = spi_rx_data.s16[7]<<16;
//		spi_tx_data.s16[7] = timing_shift>>16;

//		floop_pi_term = spi_rx_data.res3&0xff;
//		floop_d_term = (spi_rx_data.res3&0xff00)>>8;

		if(timeout) timeout--;
		else
		{
			new_mult = 0;
//			DIS_GATE();

		}
	}



}
