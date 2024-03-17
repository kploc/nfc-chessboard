
/*
 * main.c
 *
 *  Created on: Jul 17, 2022
 *      Author: vanish
 */
 
#include "Delay.h"
#include "RccConfig.h"
#include "mfrc522.h"
#include "kts1622.h"

/* Funkcje programu */
void SPI2_Config(void);
void SPI2_Enable(void);
void SPI2_Disable(void);
void UART2_Config(void);
void UART2_SendChar(uint8_t c);
void UART2_SendStrig(char *string);
void I2C1_Config(void);
void GPIO_Init(void);
void send_board(uint8_t *buffer, int bufferSize);
void dump_byte_array(uint8_t *buffer, int bufferSize);

/* Deklaracja zmiennych globalnych */
MIFARE_Key key; /* Klucz umozliwiajacy odczytanie danych z tagu */
Uid uid;	/* Aktualny adres tagu */

typedef struct  {
	uint8_t		uidByte[8];	/* Adres danego tagu */
	uint8_t		figure;	/* Figura jaka reprezentuje */
} Memo;

int main()
{
	uint8_t board[64];	/* Tablica zapamietujaca pozycje na szachownicy */
	uint8_t i;	/* Zmienna, odpowiedzialna za przemieszczanie pomiedzy polami */
	Memo memo[64];	/* Tablica struktur zapamietanych tagow */
	uint8_t k = 0;	/* Ilosc tagow zapisanych w pamieci */
	
	SysClockConfig();	/* Konfiguracja systemu */
	TIM6Config();	/* Konfiguracja timera */
	GPIO_Init();	/* Inicjalizacja wyprowadzen GPIO */
	SPI2_Config();	/* Konfiguracja komunikacji SPI */
	UART2_Config();	/* Konfiguracja komunikacji UART */
	I2C1_Config();	/* Konfiguracja komunikacji I2C */
	
	GPIOC->ODR |= 3<<10;	/* MFRC522_RST = 1 | KTS1622_RST = 1 \\ Przygotowanie komponentów */
	Delay_us(50);
	
	KTS_Init();	/* Konfiguracja KTS */
	PCD_Init();	/* Konfiguracja MFRC */
	
  for(i=0; i<6; i++) {
		key.keyByte[i] = 0xFF;	/* FFFFFFFFFF \\ Klucz do tagow */
	}
	i = 0;	/* Zerowanie zmiennej przed petla */
	
	/* 
	 *	POCZATEK PETLI 
	 */
	while(1)
	{
		PCD_AntennaOff();
		switchAnt(i);
		Delay_us(40);
		PCD_AntennaOn();	
		if (PICC_IsNewCardPresent())
		{
			if (PICC_ReadCardSerial())
			{	
				uint8_t piccType;	/* Typ tagu */	
				
				piccType = PICC_GetType(uid.sak);
				if (	piccType != PICC_TYPE_MIFARE_MINI
						&&  piccType != PICC_TYPE_MIFARE_1K
						&&  piccType != PICC_TYPE_MIFARE_4K) {
					board[i] = 0x08;
				}
				else
				{	
					uint8_t status;	/* flaga statusu */
					uint8_t j = 0;	/* Zmienna, do wyszukiwania w pamieci zapisanych figur */
					uint8_t n;
					
					status = STATUS_ERROR;
					while(j<k && status!=STATUS_OK)
					{
						status = STATUS_OK;
						for(n=0; n<8; n++)
						{
							if(uid.uidByte[n] != memo[j].uidByte[n])
							{
								status = STATUS_ERROR;
							}
						}
						if(status == STATUS_OK)
						{
							board[i] = memo[j].figure;
						}
						j++;
					}
					if(status == STATUS_ERROR)
					{
						uint8_t trailerBlock   = 7; /* Adres bloku zawierajacego klucz */
						
						status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock);
						if (status == STATUS_OK) 
						{
							uint8_t blockAddr = 4;	/* Adres bloku z którego odczytywane sa dane */
							uint8_t buffer[18];	/* Tablica przechowywujaca dane odczytane z tagu */
							uint8_t size = sizeof(buffer);	/* dlugosc ablicy */
							
							status = MIFARE_Read(blockAddr, buffer, &size);
							if (status == STATUS_OK) 
							{
								board[i] = buffer[0];
								if(k>63)
								{
									k = 0;
								}
								memo[k].figure = buffer[0];
								for(n=0; n<8; n++)
								{
									memo[k].uidByte[n] = uid.uidByte[n];
								}
								k++;
							}
						}
					}
				}
			}
			else
			{
				board[i] = 0x00;
			}
			PICC_HaltA();
			PCD_StopCrypto1();
		}
		else
		{
			board[i] = 0x00;
		}
		i++;
		if(i > 63)
		{
			i=0;
			send_board(board,64);
		}
	}
	/* 
	 *	KONIEC PETLI 
	 */
	
	/* 
	 *	POCZATEK TEST1
	 */
//	while(1)
//	{
//		PCD_AntennaOff();
//		Delay_ms(50);
//		PCD_AntennaOn();	
//		if (PICC_IsNewCardPresent())
//		{		
//			if (PICC_ReadCardSerial())
//			{	
//				dump_byte_array(uid.uidByte,uid.size);
//			}
//			PICC_HaltA();
//			PCD_StopCrypto1();
//		}
//		else
//		{
//			UART2_SendStrig("no\n\r");
//		}
//	}
	/* 
	 *	KONIEC TEST
	 */
	 
	 /* 
	 *	POCZATEK TEST2
	 */
//	 while(1)
//	{
//		PCD_AntennaOff();
//		Delay_ms(1000);
//		PCD_AntennaOn();	
//		if (PICC_IsNewCardPresent())
//		{
//			uint8_t piccType;
//			UART2_SendStrig("TAK\r\n");
//			if (PICC_ReadCardSerial())
//			{
//				UART2_SendStrig("Card UID: ");
//				dump_byte_array(uid.uidByte, uid.size);
//			}
//			dump_byte_array(uid.uidByte, uid.size);		
//			piccType = PICC_GetType(uid.sak);
//			if (	piccType != PICC_TYPE_MIFARE_MINI
//			    &&  piccType != PICC_TYPE_MIFARE_1K
//			    &&  piccType != PICC_TYPE_MIFARE_4K) {
//				UART2_SendStrig("This sample only works with MIFARE Classic cards.\r\n");
//			}
//			else
//			{			
//				uint8_t blockAddr = 4;
//				uint8_t dataBlock[] = {
//					0x0E, 0x00, 0x00, 0x00, /* F - P,S,G,W,H,K */
//					0x00, 0x00, 0x00, 0x00, /* B - 1,2,3,4,5,6 */
//					0x00, 0x00, 0x00, 0x00, /* C - 9,A,B,C,D,E */
//					0x00, 0x00, 0x00, 0x00
//				};

//				uint8_t trailerBlock   = 7;
//				uint8_t status;
//				uint8_t buffer[18];
//				uint8_t size = sizeof(buffer);

//				UART2_SendStrig("OK\r\n");
//				status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock);

//				if (status != STATUS_OK) {
//					UART2_SendStrig("PCD_Authenticate() failed.\r\n");
//				}
//				else
//				{
//					UART2_SendStrig("Current data in sector.\r\n");
//					status = MIFARE_Read(blockAddr, buffer, &size);

//					if (status != STATUS_OK) {
//						UART2_SendStrig("MIFARE_Read() failed.\r\n");
//					}
//					else
//					{
//						dump_byte_array(buffer, 16);
//					}

//				}
//				Delay_ms(10);
//				status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, trailerBlock);

//				if (status != STATUS_OK) {
//					UART2_SendStrig("PCD_Authenticate() failed.\r\n");
//				}
//				else
//				{
//					status = MIFARE_Write(blockAddr, dataBlock, 16);

//					if (status != STATUS_OK) {
//						UART2_SendStrig("MIFARE_Write() failed.\r\n");
//					}
//					else
//					{
//						UART2_SendStrig("Current data in sector.\r\n");
//					}

//				}

//			}
//			PICC_HaltA();
//			PCD_StopCrypto1();
//		}
//		else
//		{
//			UART2_SendStrig("NIE\r\n");
//		}
//	}
	/* 
	 *	KONIEC TEST2
	 */
}

/*SPI*/
void SPI2_Config(void)
{
	RCC->APB1ENR1 |= 1<<14;	/* SPI2EN = 1 \\ Wlaczenie zegara SPI2 */
	SPI2->CR1 &= ~(unsigned int)((1<<0) | (1<<1));	/* CPHA/OL = 0 \\ Ustawienie pierwszej krawedzi zegara jako poczatek wymiany danych, oraz bezczynnsci zegara na stan niski */
	SPI2->CR1 |= 1<<2;	/* MSTR = 1 \\ Wlaczenie konfiguracji Master */
	SPI2->CR1 |= 2<<3;	/* BR = 10B \\ Ustawienie czestotliwosci przesylu na 10MHz */
	SPI2->CR1 &= ~(unsigned int)(1<<7);	/* LSBFIRST = 0 \\ Usttawienie pierwszego bitu ramki jako MSB */
	SPI2->CR1 |= (1<<8) | (1<<9);	/* SSI/SM = 1 \\ Ustawienie wyboru slave-a */
	SPI2->CR1 &= ~(unsigned int)((1<<10) | (1<<11) | (1<<13));	/* RXONLY CRCL/EN = 0 \\ Ustawienie trybu Full-duplex, dlugosci crc na 8 bit, wylaczenie obliczania CRC */
	SPI2->CR2 = 0;	/* CR2 = 0 \\ Zerownie rejestru */
	SPI2->CR2 |= 1<<12;	/* FRXTH = 1 \\ Ustawienie progu odbioru danych FIFO na 8 bit */
	SPI2->CR1 |= 1<<6;	/* SPE = 1 \\ Wlaczenie komunikacji SPI */
}

void SPI2_Enable(void)
{
	SPI2->CR1 |= 1<<6;	/* SPE = 1 \\ Wlaczenie komunikacji SPI */
}

void SPI2_Disable(void)
{
	SPI2->CR1 &= ~(unsigned int)(1<<6);	/* SPE = 0 \\ Wylaczenie komunikacji SPI */
}

/*UART*/
void UART2_Config(void)
{
	RCC->APB1ENR1 |= 1<<17;	/* USART2EN = 1 \\ Wlaczenie zegara USART2 */
	USART2->CR1 = 0;	/* CR1 = 0 \\ Zerowanie rejestru */
	USART2->BRR |= 694<<0;	/* BRR = 10 1011 0110B \\ Ustawienie szybkosci transmisji na 115200 */
	USART2->CR2 &= ~(unsigned int)(1<<12) | (1<<13);	/* STOP = 0 \\ Ustawienie 1 stop bit */
	USART2->CR1 |= 1<<0;	/* UE = 1 \\ Wlaczenie komunikacji USART */
	USART2->CR1 |= 1<<2;	/* RE = 1 \\ Wlaczenie funkcji odbioru danch */
	USART2->CR1 |= 1<<3;	/* TE = 1 \\ Wlaczenie funkcji wysylu danych */
}

void UART2_SendChar(uint8_t c)
{
	USART2->TDR = c;	/* TDR = c \\ Wpisanie znaku do rejestru wysylu */
	while(!(USART2->ISR & 1<<6));	/* TC \\ Oczekiwanie na zakonczenie transmisji */
}

void UART2_SendStrig(char *string)
{
	while (*string) UART2_SendChar(*string++);	/* Wykonanie wysylu dla kolejnych znakow ciagu */
}

/*I2C*/
void I2C1_Config(void)
{
	RCC->APB1ENR1 |= (1<<21);	/* I2C1EN = 1 \\ Wlaczenie zegara I2C1 */
	I2C1->CR1 &= ~(unsigned int)(1<<0);	/* PE = 0 \\ Wylaczenie komunikacji I2C */
	I2C1->CR1 &= ~(unsigned int)(1<<12);	/* ANFOFF = 0 \\ Wlaczenie filtracji sygnalu */
	I2C1->CR1 |= 0xF<<8;	/* DNF = 1111B \\ Ustawienie najwyzszej zdolnosci filtrowania */
	I2C1->TIMINGR = 0x00300F33;	/* TIMINGR \\ Ustawienie czestotliwosci przesylu (Wygenerowane w STM32CubeMX) */
	I2C1->CR1 &= ~(unsigned int)(1<<17);	/* NOSRETCH = 0 \\ Wlaczenie rozciagniecia zegara */
	I2C1->CR1 |= 1<<0;	/* PE = 1 \\ Wlaczenie komunikacji I2C */
	SYSCFG->CFGR1 |= 1<<20; /* I2C1_FMP = 1 \\ Wlaczenie trybu FastMode + (czestotliwosci 1MHz) */
}

/*GPIO*/
void GPIO_Init(void)
{
	/*GPIO FOR SPI*/
	/* MISO -> PC2 | MOSI -> PC3 | CS -> PC4 | SCK -> PB10*/
	RCC->AHB2ENR |= (1<<1) | (1<<2);	/* GPIOB/C = 1 \\ Wlaczenie zegarow wyprowadzen GPIO portu B i C */
	GPIOC->MODER &= ~(unsigned int)(0x3f<<4);	/* MODE2/3/4 = 0 \\ Wyzerowanie trybu dla wyprowadzen PC2/3/4 */
	GPIOC->MODER |= (2<<4) | (2<<6) | (1<<8);	/* MODE2/3 = 10B /4 = 1 \\ Ustawienie trybu funkcji wyprowadzen 2,3, trybu wyjscia 4 */
	GPIOC->OSPEEDR |= (3<<4) | (3<<6) | (3<<8);	/* OSPEED2/3/4 = 11B \\ Ustawienie bardzo duzej predkosci wyprowadzen 2,3,4 */
	GPIOC->AFR[0] |= (5<<8) | (5<<12);	/* AFSEL2/3 = AF5 \\ Ustawienie alternatywnaj funkcji na SPI2 */
	GPIOB->MODER &= ~(unsigned int)(3<<20);	/* MODE10 = 0 \\ Wyzerowanie trybu dla wyprowadzenia PB10 */
	GPIOB->MODER |= 2<<20;	/* MODE10 = 10B \\ Ustawienie trybu funkcji wyprowadzenia 10 */
	GPIOB->OSPEEDR |= 3<<20;	/* OSPEED10 = 11B \\ Ustawienie bardzo duzej predkosci wyprowadzenia 10 */
	GPIOB->AFR[1] |= 5<<8;	/* AFSEL10 = AF5 \\ Ustawienie alternatywnaj funkcji na SPI2 */
	
	/*GPIO FOR UART*/
	/* TX -> PA2 | RX -> PA3 */
	RCC->AHB2ENR |= 1<<0; /* GPIOA = 1 \\ Wlaczenie zegarow wyprowadzen GPIO portu A */
	GPIOA->MODER &= ~(unsigned int)(0xf<<4);	/* MODE2/3 = 0 \\ Wyzerowanie trybu dla wyprowadzen PA2/3 */ 
	GPIOA->MODER |= (2<<4) | (2<<6);	/* MODE2/3 = 10B \\ Ustawienie trybu funkcji wyprowadzen 2,3 */ 
	GPIOA->OSPEEDR |= (3<<4) | (3<<6);	/* OSPEED2/3 = 11B \\ Ustawienie bardzo duzej predkosci wyprowadzen 2,3 */
	GPIOA->AFR[0] |= (7<<8) | (7<<12);	/* AFSEL2/3 = AF7 \\ Ustawienie alternatywnaj funkcji na USART2 */
	
	/*GPIO FOR I2C*/
	/* SDA -> PB7 | SCL -> PB6 */
	GPIOB->MODER &= ~(unsigned int)(0xf<<12);	/* MODE6/7 = 0 \\ Wyzerowanie trybu dla wyprowadzen PB6/7 */ 
	GPIOB->MODER |= (2<<12) | (2<<14);	/* MODE6/7 = 10B \\ Ustawienie trybu funkcji wyprowadzen 6,7 */
	GPIOB->OTYPER |= (1<<6) | (1<<7);	/* OT6/7 = 1 \\ Ustawienie trybu open-drain 6,7 */
	GPIOB->OSPEEDR |= (3<<12) | (3<<14);	/* OSPEED6/7 = 11B \\ Ustawienie bardzo duzej predkosci wyprowadzen 6,7*/
	GPIOB->PUPDR |= (1<<12) | (1<<14);	/* PUPD6/7 = 1 \\ Ustawienie trybu pull-up wyprowadzen 6,7 */
	GPIOB->AFR[0] |= (4<<24) | (4<<28);	/* AFSEL6/7 = AF7 \\ Ustawienie alternatywnaj funkcji na I2C1 */
	
	/*OTHER*/
	/* MFRC522_RST -> PC10 | KTS1622_RST -> PC11 */
	GPIOC->MODER &= ~(unsigned int)(0xf<<20);	/* MODE10/11 = 0 \\ Wyzerowanie trybu dla wyprowadzen PC10/11 */ 
	GPIOC->MODER |= 5<<20;	/* MODE10/11 = 1 \\ Ustawiene trybu wyjscia dla wyprowadzen 10,11 */
}

/*OTHER*/
void send_board(uint8_t *buffer, int bufferSize)
{
	uint8_t digits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','\r','\n'};
	int i;
	for (i = 0; i < bufferSize; i++) 
	{
    UART2_SendChar(digits[buffer[i]]);
	}
	UART2_SendChar(digits[16]);
	UART2_SendChar(digits[17]);
}

void dump_byte_array(uint8_t *buffer, int bufferSize) 
{
	uint8_t digits[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F',' ','\r','\n'};
	int i;
	char str[2];
  for (i = 0; i < bufferSize; i++) 
	{
    if(buffer[i]<0x10)
    {
    	str[0] = '0';
    	str[1] = digits[buffer[i]];
    }
    else
    {
    	str[0] = digits[(buffer[i] & 0xF0)>>4];
    	str[1] = digits[(buffer[i] & 0x0F)];
    }
    UART2_SendChar(str[0]);
		UART2_SendChar(str[1]);
    UART2_SendChar(digits[16]);
	}
	UART2_SendChar(digits[17]);
	UART2_SendChar(digits[18]);
}
