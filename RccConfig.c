
/*
 * RccConfig.c
 *
 *  Created on: Jul 17, 2022
 *      Author: vanish
 */
 
 #include "RccConfig.h"

void SysClockConfig(void)
{
	RCC->APB2ENR |= 1<<0;	/* SYSCFGEN = 1 \\ Wlaczenie zegara systemowego */
	RCC->APB1ENR1 |= 1<<28;	/* PWREN = 1 \\ Wlaczenie zegara interfejsu zasilania */
	FLASH->ACR |= 4<<0;	/* LATENCY = 0x100B \\ Ustawienie czterech okresow HCLK jako czas dostepu do pamieci Flash */
	while((FLASH->ACR & 4<<0) != 4);	/* LATENCY \\ Oczekiwanie na zmiane ustawien ACR */
	PWR->CR1 |= 1<<9;	/* VOS = 1 \\ Ustawienie pierwszego zakresu napiecia */
	RCC->CR |= 1<<8;	/* HSION = 1 \\ Wlaczenie wewnetrznego zegara wysokiej czestotliwosci */
	while(!(RCC->CR & 1<<10));	/* HSIRDY \\ Oczekiwanie na gotowosc zegara */
	RCC->PLLCFGR = (2<<0) | (0<<4) | (10<<8) | (0<<25);	/* PLLSRC/M/N/R \\ Konfiguracja preskalera zegara systemowego (80 Mhz) */
	RCC->PLLCFGR |= 1<<24; /* PLLREN = 1 \\ Ustawienie wyjscia PLLR jako wyjscia prescalera */
	RCC->CR |= 1<<24; /* PLLON = 1 \\ Wlaczenie prescalera */
	while(!(RCC->CR & 1<<25)); /* PLLRDY \\ Oczekiwanie na gotowosc prescalera */
	RCC->CFGR |= 0<<4; /* HPRE = 0 \\ Wylaczenie prescalera AHB */
	RCC->CFGR |= 0<<8; /* PPRE1 = 0 \\ Wylaczene prescalera APB1 */
	RCC->CFGR |= 0<<11; /* PPRE2 = 0 \\ Wylaczenie prescalera APB2 */
	RCC->CFGR |= (3<<0); /* SW = 0x11B \\ Ustawienie PLL jako zegar systemowy */
	while (!(RCC->CFGR & (3<<2))); /* SW \\ Oczekiwanie na gotowosc zegara */
}
