
/*
 * RccConfig.c
 *
 *  Created on: Jul 17, 2022
 *      Author: vanish
 */
 
#include "Delay.h"

void TIM6Config(void)
{
	RCC->APB1ENR1 |= 1<<4; /* TIM6EN = 1 \\ Wlaczenie timera 6 */
	TIM6->PSC = 80-1; /* PSC = 100 1111B \\ Ustawienie preskalera timera 6 na 80 (aby okres timera = 1 us)*/
	TIM6->ARR = 0xFFFF; /* ARR =  1111 1111 1111 1111B \\ Ustawienie najwyzszej czestotliwosci odswierzania */
	TIM6->CR1 |= 1<<0; /* CEN = 1 \\ Wlaczenie timera */
	while(!(TIM6->SR & 1<<0)); /* Oczekiwanie na gotowosc timera */
}

void Delay_us(uint16_t us)
{
	TIM6->CNT = 0; /* CNT = 0 \\ Ustawienie wartosci timera na 0 */
	while(TIM6->CNT < us); /* CNT < us \\ Oczekiwanie na osiagniecie przez timer wyznaczonej wartosci */
}

void Delay_ms(uint16_t ms)
{
	uint16_t i; /* zmienna inkrementowana */
	for (i=0; i<ms; i++)
	{
		Delay_us(1000); /* Odczekanie 1 milisekundy wyznaczona ilosc razy */
	}
}
