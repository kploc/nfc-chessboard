
/*
 * struct.h
 *
 *  Created on: Jul 31, 2022
 *      Author: vanish
 */
 
#pragma once
 
#include "stm32l476xx.h"

typedef struct  {
	uint8_t		size;
	uint8_t		uidByte[10];
	uint8_t		sak;
} Uid;

extern Uid uid;

typedef struct {
	uint8_t		keyByte[6];
} MIFARE_Key;

extern MIFARE_Key key;

typedef enum { false, true } bool;
