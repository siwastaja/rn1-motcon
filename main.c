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

void uart_rx_handler()
{
	//char byte = USART1->RDR;

	if(USART1->ISR & USART_ISR_ORE)
	{
		USART1->ICR |= USART_ICR_ORECF;
	}
}

void adc_int_handler()
{
	LED_ON();
	delay_ms(50);
	LED_OFF();
	ADC1->ISR |= 1UL<<7;
}

void usart_print(const char *buf)
{
	while(buf[0] != 0)
	{
		while((USART1->ISR & (1UL<<7)) == 0) ;
		USART1->TDR = buf[0];
		buf++;
	}
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



int main()
{
	RCC->CFGR = 0b1010UL << 18; // PLL x12 (because of /2 prediv)  --> 48 MHz
	RCC->CR |= 1UL << 24; // PLL on
	RCC->CFGR |= 0b10; // Change PLL to system clock

	while(!(RCC->CR & 1UL<<25)) ; // Wait for PLL

	while((RCC->CFGR & (0b11UL<<2)) != (0b10UL<<2)) ; // Wait for switchover to PLL.


	RCC->AHBENR |= 1UL<<22 /* PORTF clock */ | 1UL<<18 /* PORTB clock */ | 1UL<<17 /* PORTA clock */;
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
	GPIOA->MODER   = 0b00000001011010101011111111111111;
	GPIOA->OSPEEDR = 0b00000000000101010100000000000000;
	GPIOA->PUPDR   = 0b00010100000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOB->MODER   = 0b00000000000000000000101010001010;
	GPIOB->OSPEEDR = 0b00000000000000000000000000000101;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |
	GPIOF->MODER   = 0b00000000000000000000000000000001;
	GPIOF->OSPEEDR = 0b00000000000000000000000000000000;
	             //    15141312111009080706050403020100
	             //     | | | | | | | | | | | | | | | |


	GPIOA->AFR[0] = 2UL<<28 /*PA7 = TIM1*/;
	GPIOA->AFR[1] = 2UL<<0 /*PA8 = TIM1*/ | 2UL<<4 /*PA9 = TIM1*/ | 2UL<<8 /*PA10 = TIM1*/;
	GPIOB->AFR[0] = 2UL<<0 /*PB0 = TIM1*/ | 2UL<<4 /*PB1 = TIM1*/;

	// ARR = freq
	// CCR = duty
	// 110 (mode 1) or 111 (mode 2) OCxM bits in CCMRx
	// OCxPE must be enabled
	//ARPE in CR1
	// just before starting: set UG in EGR
	// mode = 1, maybe

	TIM1->CR1 = 0b01<<5 /*centermode 1*/;
	// TIM1->CR2 defaults ok

	TIM1->CCMR1 = 1UL<<3 /*OC1 Preload enable*/ | 0b110UL<<4 /*OC1 PWMmode 1*/ |
	              1UL<<11 /*OC2 Preload Enable*/| 0b110UL<<12 /*OC1 PWMmode 1*/;
	TIM1->CCMR2 = 1UL<<3 /*OC3 Preload enable*/ | 0b110UL<<4 /*OC4 PWMmode 1*/;
	TIM1->CCER =  1UL<<0 /*OC1 on*/ | 1UL<<2 /*OC1 complementary output enable*/ |
	              1UL<<4 /*OC2 on*/ | 1UL<<6 /*OC2 complementary output enable*/ |
	              1UL<<8 /*OC3 on*/ | 1UL<<10 /*OC3 complementary output enable*/;

	TIM1->ARR = 1024; // 23.4 kHz
	TIM1->CCR1 = 100;
	TIM1->CCR2 = 200;
	TIM1->CCR3 = 300;
	TIM1->BDTR = 1UL<<15 /*Main output enable*/ | 1UL /*21ns deadtime*/;
	TIM1->EGR |= 1; // Generate Reinit+update
	TIM1->CR1 |= 1; // Enable

	set_prot_lim(2000);
	DAC->CR |= 1; // enable, defaults good otherwise.


//	NVIC_EnableIRQ(USART1_IRQn);
//	__enable_irq();

	EN_GATE();

	while(1)
	{
		LED_ON();
		delay_ms(500);
		LED_OFF();
		delay_ms(500);

/*		char buffer[200];
		char* buf = buffer;
		buf = o_str_append(buf, "kakka=");
		buf = o_utoa16(123, buf);
		buf = o_str_append(buf, "\n\r");*/
	}



}
