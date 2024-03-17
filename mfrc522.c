
/*
 * mfrc522.c
 *
 *  Created on: Jul 31, 2022
 *      Author: vanish
 */

#include "mfrc522.h"

extern Uid uid;
extern MIFARE_Key key;

void CS_Enable(void)
{
	GPIOC->ODR &= ~(unsigned int)(1<<4);	/* SPI_CS = 0 \\ Rozpoczecie wymiany danych ze slave-m */
}

void CS_Disable(void)
{
	GPIOC->ODR |= 1<<4;	/* SPI_CS = 1 \\ Zakonczenie wymiany danych ze slave-m */
}

uint8_t ReadWrite(uint8_t data)
{
  while(!(SPI2 -> SR & 1<<1));	/* TXE \\ Oczekiwanie na wyczyszczenie rejestru danych wysylanych*/
  *(volatile uint8_t *)&SPI2->DR = data;	/* DR \\ Wpisanie ramki do rejestru danych */
  while (!(SPI2 -> SR & 1<<0));	/* RXNE \\ Oczekiwanie na wyczyszczenie rejestru danych odbieranych*/
  return *(volatile uint8_t *)&SPI2 -> DR; /* DR \\ Odczytanie danych zwroconych przez slave-a */
}

uint8_t PCD_ReadRegister(uint8_t reg) {
  uint8_t value;
	
	reg = ((reg << 1) & 0x7E) | 0x80;
	CS_Enable();
	ReadWrite(reg);
	value = ReadWrite(0);
	CS_Disable();
	return value;
}

void PCD_ReadRegister_2(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign) {
	uint8_t address = 0x80 | ((reg << 1) & 0x7E); 		/* MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.*/
	uint8_t index = 0;
	
	if (count == 0) {
		return;
	}
	CS_Enable();
	count--;
	ReadWrite(address);
	if (rxAlign) {
		uint8_t mask = (0xFF << rxAlign) & 0xFF;
		uint8_t value = ReadWrite(address);
		
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = ReadWrite(address);
		index++;
	}
	values[index] = ReadWrite(0);
	CS_Disable();
}

void PCD_WriteRegister(uint8_t reg, uint8_t value) {
	reg = (reg << 1) & 0x7E; 
	CS_Enable();
	ReadWrite(reg);
	ReadWrite(value);
	CS_Disable();
}

void PCD_WriteRegister_2(uint8_t reg, uint8_t count, uint8_t *values) {
	int index;
	
  CS_Enable();
  reg = (reg << 1) & 0x7E;
  ReadWrite(reg);																		/* MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.*/
	for (index = 0; index < count; index++) {
		ReadWrite(values[index]);
	}
	CS_Disable();
}

void PCD_SetRegisterBitMask(uint8_t reg, uint8_t mask) {
	uint8_t tmp;
	
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);
}

uint8_t PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result) {
	uint16_t i = 500;	
	
	PCD_WriteRegister(CommandReg, PCD_Idle);
	PCD_WriteRegister(DivIrqReg, 0x04);
	PCD_WriteRegister(FIFOLevelReg, 0x80);
	PCD_WriteRegister_2(FIFODataReg, length, data);
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);
	
	do {																							/* DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved */
		uint8_t n = PCD_ReadRegister(DivIrqReg);
		
		if (n & 0x04) {																	/* CRCIRq bit set - calculation done */
			PCD_WriteRegister(CommandReg, PCD_Idle);			/* Stop calculating CRC for new content in the FIFO. */																									
			result[0] = PCD_ReadRegister(CRCResultRegL);	/* Transfer the result from the registers to the result buffer */
			result[1] = PCD_ReadRegister(CRCResultRegH);
			return STATUS_OK;
		}
		i--;
	}
	while (i!=0);																			/* 500 passed and nothing happened. Communication with the MFRC522 might be down. */
	return STATUS_TIMEOUT;
}

void PCD_AntennaOn() {
    uint8_t value;

    value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03) PCD_WriteRegister(TxControlReg, value | 0x03);
}

void PCD_Reset() {
	uint8_t count = 0;
	
	PCD_WriteRegister(CommandReg, PCD_SoftReset);			/* Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74µs. Let us be generous: 50ms. */
	do {
		Delay_ms(50);																		/* Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms) */
	} while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
}


void PCD_Init() {
	PCD_Reset();
	PCD_WriteRegister(TxModeReg, 0x00);								/* Reset baud rates */
	PCD_WriteRegister(RxModeReg, 0x00);
	PCD_WriteRegister(ModWidthReg, 0x26);							/* Reset ModWidthReg */
	/* When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg. */
	PCD_WriteRegister(TModeReg, 0x80);								/* TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds */
	PCD_WriteRegister(TPrescalerReg, 0xA9);						/* TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25µs. */
	PCD_WriteRegister(TReloadRegH, 0x03);							/* Reload timer with 0x3E8 = 1000, ie 25ms before timeout. */
	PCD_WriteRegister(TReloadRegL, 0xE8);
	PCD_WriteRegister(TxASKReg, 0x40);								/* Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting */
	PCD_WriteRegister(ModeReg, 0x3D);									/* Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4) */
	PCD_AntennaOn();																	/* Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset) */
}

uint8_t PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
	uint8_t txLastBits = validBits ? *validBits : 0;	/* Prepare values for BitFramingReg */
	uint8_t bitFraming = (uint8_t)(rxAlign << 4) + txLastBits;	/* RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0] */
	bool completed = false;
	uint8_t n;
	uint16_t i = 1200;
	uint8_t errorRegValue; /* ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr */
	uint8_t _validBits = 0;

	PCD_WriteRegister(CommandReg, PCD_Idle);					/* Stop any active command. */
	PCD_WriteRegister(ComIrqReg, 0x7F);								/* Clear all seven interrupt request bits */
	PCD_WriteRegister(FIFOLevelReg, 0x80);						/* FlushBuffer = 1, FIFO initialization */
	PCD_WriteRegister_2(FIFODataReg, sendLen, sendData);	/* Write sendData to the FIFO */
	PCD_WriteRegister(BitFramingReg, bitFraming);			/* Bit adjustments */
	PCD_WriteRegister(CommandReg, command);						/* Execute the command */
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);		/* StartSend=1, transmission of data starts */
	}
	/* In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the PCD stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out. */
	do {
		n = PCD_ReadRegister(ComIrqReg);								/* ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq */
		if (n & waitIRq) {															/* One of the interrupts that signal success has been set. */
			completed = true;
			break;
		}
		if (n & 0x01) {																	/* Timer interrupt - nothing received in 25ms */
			return STATUS_TIMEOUT;
		}
		i--;
	}
	while (i!=0);																			/* 1000 and nothing happened. Communication with the MFRC522 might be down. */
	if (!completed) {																	/* Stop now if any errors except collisions were detected. */
		return STATUS_TIMEOUT;
	}
	errorRegValue = PCD_ReadRegister(ErrorReg);
	if (errorRegValue & 0x13) {	 											/* BufferOvfl ParityErr ProtocolErr */
		return STATUS_ERROR;
	}
	if (backData && backLen) {												/* If the caller wants data back, get it from the MFRC522. */
		n = PCD_ReadRegister(FIFOLevelReg);							/* Number of bytes in the FIFO */
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;																		/* Number of bytes returned */
		PCD_ReadRegister_2(FIFODataReg, n, backData, rxAlign);	/* Get received data from FIFO */
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		/* RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid. */
		if (validBits) {
			*validBits = _validBits;
		}
	}
	if (errorRegValue & 0x08) {												/* Tell about collisions */
		return STATUS_COLLISION;
	}
	if (backData && backLen && checkCRC) {						/* Perform CRC_A validation if requested. */
		uint8_t controlBuffer[2];												/* Verify CRC_A - do our own calculation and store the control in controlBuffer. */
		uint8_t status;
		
		if (*backLen == 1 && _validBits == 4) {					/* In this case a MIFARE Classic NAK is not OK. */
			return STATUS_MIFARE_NACK;
		}
		if (*backLen < 2 || _validBits != 0) {					/* We need at least the CRC_A value and all 8 bits of the last byte must be received. */
			return STATUS_CRC_WRONG;
		}
		status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	return STATUS_OK;
}

void PCD_ClearRegisterBitMask(uint8_t reg, uint8_t mask) {
	uint8_t tmp;
	
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));						/* clear bit mask */
}

uint8_t PCD_TransceiveData(uint8_t *sendData,	uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC) {
	uint8_t waitIRq = 0x30;
	
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}

uint8_t PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA,uint8_t *bufferSize) {
	uint8_t validBits;
	uint8_t status;

	if (bufferATQA == nullptr || *bufferSize < 2) {		/* The ATQA response is 2 bytes long. */
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);					/* ValuesAfterColl=1 => Bits received after collision are cleared. */
	validBits = 7;																		/* For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0] */
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {					/* ATQA must be exactly 16 bits. */
		return STATUS_ERROR;
	}
	return STATUS_OK;
}

uint8_t PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

bool PICC_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	uint8_t result = PICC_RequestA(bufferATQA, &bufferSize);

	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	PCD_WriteRegister(ModWidthReg, 0x26);							/* Reset ModWidthReg */
	return (result == STATUS_OK || result == STATUS_COLLISION);
}

void PCD_AntennaOff() {
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}

uint8_t PICC_Select(uint8_t validBits) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	uint8_t result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;																	/* The first index in uid->uidByte[] that is used in the current Cascade Level. */
	int currentLevelKnownBits;												/* The number of known UID bits in the current Cascade Level. */
	uint8_t buffer[9];																/* The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A */
	uint8_t bufferUsed;																/* The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO. */
	uint8_t rxAlign;																	/* Used in BitFramingReg. Defines the bit position for the first bit received. */
	uint8_t txLastBits = 0;														/* Used in BitFramingReg. The number of valid bits in the last transmitted byte. */
	uint8_t *responseBuffer = 0;
	uint8_t responseLength = 0;
	uint8_t bytesToCopy;
	
	if (validBits > 80) {
		return STATUS_INVALID;
	}																									/* Prepare MFRC522 */
	PCD_ClearRegisterBitMask(CollReg, 0x80);					/* ValuesAfterColl=1 => Bits received after collision are cleared. */
	uidComplete = false;
	while (!uidComplete) {														/* Repeat Cascade Level loop until we have a complete UID. */
		switch (cascadeLevel) {													/* Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2. */
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid.size > 4;	/* When we know that the UID has more than 4 bytes */
				break;
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid.size > 7;	/* When we know that the UID has more than 7 bytes */
				break;
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;											/* Never used in CL3. */
				break;
			default:
				return STATUS_INTERNAL_ERROR;
		}
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		index = 2;
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		bytesToCopy = (uint8_t)currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); /* The number of bytes needed to represent the known bits for this level. */
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; 		/* Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag */
			
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid.uidByte[uidIndex + count];
			}
		}
		if (useCascadeTag) {														/* Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits */
			currentLevelKnownBits += 8;
		}
		selectDone = false;															/* Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations. */
		while (!selectDone) {														/* Find out how many bits and bytes to send and receive. */
			if (currentLevelKnownBits >= 32) { 						/* All UID bits in this Cascade Level are known. This is a SELECT. */
				buffer[1] = 0x70; 													/* NVB - Number of Valid Bits: Seven whole bytes */
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];  /* Calculate BCC - Block Check Character */
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);  /* Calculate CRC_A */
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits = 0;															/* 0 => All 8 bits are valid. */
				bufferUsed = 9;															/* Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx) */
				responseBuffer = &buffer[6];
				responseLength = 3;
			}
			else { 																				/* This is an ANTICOLLISION. */
				txLastBits = (uint8_t)currentLevelKnownBits % 8;
				count = (uint8_t)currentLevelKnownBits / 8;  /* Number of whole bytes in the UID part. */
				index = 2 + count;													/* Number of whole bytes: SEL + NVB + UIDs */
				buffer[1] = (uint8_t)(index << 4) + txLastBits;  /* NVB - Number of Valid Bits */
				bufferUsed = index + (txLastBits ? 1 : 0);
				responseBuffer = &buffer[index];						/* Store response in the unused part of buffer */
				responseLength = sizeof(buffer) - index;
			}																							/* Set bit adjustments */
			rxAlign = txLastBits;													/* Having a separate variable is overkill. But it makes the next line easier to read. */
			PCD_WriteRegister(BitFramingReg, (uint8_t)(rxAlign << 4) + txLastBits);  /* RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0] */
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);  /* Transmit the buffer and receive the response. */
			if (result == STATUS_COLLISION) { 						/* More than one PICC in the field => collision. */
				uint8_t valueOfCollReg = PCD_ReadRegister(CollReg);  /* CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0] */
				uint8_t collisionPos = valueOfCollReg & 0x1F;  /* Values 0-31, 0 means bit 32. */
				
				if (valueOfCollReg & 0x20) { 								/* CollPosNotValid */
					return STATUS_COLLISION;                  /* Without a valid collision position we cannot continue */
				}	
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { /* No progress - should not happen */
					return STATUS_INTERNAL_ERROR;
				}
				currentLevelKnownBits	= collisionPos;				/* Choose the PICC with the bit set. */
				count = (uint8_t)currentLevelKnownBits % 8;  /* The bit to modify */
				checkBit = (uint8_t)(currentLevelKnownBits - 1) % 8;
				index = 1 + (uint8_t)(currentLevelKnownBits / 8) + (count ? 1 : 0);  /* First byte is index 0. */
				buffer[index] |= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { 																				/* STATUS_OK */
				if (currentLevelKnownBits >= 32) { 					/* This was a SELECT. */
					selectDone = true; 												/* No more anticollision */
				}																						/* We continue below outside the while. */
				else { 																			/* This was an ANTICOLLISION. */
					currentLevelKnownBits = 32;								/* We now have all 32 bits of the UID in this Cascade Level */
				}																						/* Run loop again to do the SELECT. */		
			}
		} 																							/* End of while (!selectDone)
		// We do not check the CBB - it was constructed by us above.
		// Copy the found UID bytes from buffer[] to uid->uidByte[] */
		index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; 		/* source index in buffer[] */
		bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid.uidByte[uidIndex + count] = buffer[index++];
		}																								/* Check response SAK (Select Acknowledge) */
		if (responseLength != 3 || txLastBits != 0) {		/* SAK must be exactly 24 bits (1 byte + CRC_A). */
			return STATUS_ERROR;
		}																								/* Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore. */
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { 								/* Cascade bit set - UID not complete yes */
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid.sak = responseBuffer[0];
		}
	} 																								/* End of while (!uidComplete) */
	uid.size = 3 * cascadeLevel + 1;									/* Set correct uid->size */
	return STATUS_OK;
}

bool PICC_ReadCardSerial() {
	uint8_t result = PICC_Select(0x00);
	
	return (result == STATUS_OK);
}

uint8_t PICC_GetType(uint8_t sak) {
	/* http://www.nxp.com/documents/application_note/AN10833.pdf
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A) */
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;			/* UID not complete */
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
}

uint8_t PCD_Authenticate(uint8_t command,uint8_t blockAddr) {
	uint8_t waitIRq = 0x10;														/* IdleIRq */
	int i;																						/* Build command buffer */
	uint8_t sendData[12];
	
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (i = 0; i < MF_KEY_SIZE; i++) {								/* 6 key bytes */
		sendData[2+i] = key.keyByte[i];
	}
	/* Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid. */
	for (i = 0; i < 4; i++) {													/* The last 4 bytes of the UID */
		sendData[8+i] = uid.uidByte[i+uid.size-4];
	}
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), nullptr, nullptr, nullptr, 0, false);  /* Start the authentication. */
}

uint8_t PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, bool acceptTimeout) {
	uint8_t result;
	uint8_t cmdBuffer[18];
	uint8_t waitIRq = 0x30;
	uint8_t cmdBufferSize;
	uint8_t validBits = 0;
	
	if (sendData == nullptr || sendLen > 16) {
		return STATUS_INVALID;
	}
	memcpy(cmdBuffer, sendData, sendLen);
	result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
	if (result != STATUS_OK) {
		return result;
	}
	sendLen += 2;
	cmdBufferSize = sizeof(cmdBuffer);
	result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, false);
	if (acceptTimeout && result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result != STATUS_OK) {
		return result;
	}
	if (cmdBufferSize != 1 || validBits != 4) {
		return STATUS_ERROR;
	}
	if (cmdBuffer[0] != MF_ACK) {
		return STATUS_MIFARE_NACK;
	}
	return STATUS_OK;
}

uint8_t MIFARE_Read(uint8_t blockAddr, uint8_t *buffer,	uint8_t *bufferSize) {
	uint8_t result;

	if (buffer == nullptr || *bufferSize < 18) {			/* Sanity check */
		return STATUS_NO_ROOM;
	}
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
}

uint8_t MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t bufferSize) {
	uint8_t result;
	uint8_t cmdBuffer[2];
	
	if (buffer == nullptr || bufferSize < 16) {				/* Sanity check */
		return STATUS_INVALID;
	}
	cmdBuffer[0] = PICC_CMD_MF_WRITE;
	cmdBuffer[1] = blockAddr;
	result = PCD_MIFARE_Transceive(cmdBuffer, 2, false);
	if (result != STATUS_OK) {
		return result;
	}
	result = PCD_MIFARE_Transceive(buffer, bufferSize, false);
	if (result != STATUS_OK) {
		return result;
	}
	return STATUS_OK;
}

uint8_t PICC_HaltA() {
	uint8_t result;
	uint8_t buffer[4];
	
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0, nullptr, 0, true);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) {
		return STATUS_ERROR;
	}
	return result;
}

void PCD_StopCrypto1() {
	PCD_ClearRegisterBitMask(Status2Reg, 0x08);
}
