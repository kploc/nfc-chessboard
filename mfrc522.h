
/*
 * mfrc522.h
 *
 *  Created on: Jul 31, 2022
 *      Author: vanish
 */
 
#include "struct.h"
#include "Delay.h"
#include <string.h>

#define	STATUS_OK  							0x01
#define	STATUS_ERROR						0x02
#define	STATUS_COLLISION				0x03
#define	STATUS_TIMEOUT					0x04
#define	STATUS_NO_ROOM					0x05
#define STATUS_INTERNAL_ERROR		0x06
#define	STATUS_INVALID					0x07
#define	STATUS_CRC_WRONG				0x08
#define	STATUS_MIFARE_NACK			0xFF

/*PICC Type*/
#define PICC_TYPE_UNKNOWN				0x01
#define	PICC_TYPE_ISO_14443_4		0x02	/* PICC compliant with ISO/IEC 14443-4 */
#define	PICC_TYPE_ISO_18092			0x03 	/* PICC compliant with ISO/IEC 18092 (NFC) */
#define	PICC_TYPE_MIFARE_MINI		0x04	/* MIFARE Classic protocol, 320 bytes */
#define	PICC_TYPE_MIFARE_1K			0x05	/* MIFARE Classic protocol, 1KB */
#define	PICC_TYPE_MIFARE_4K			0x06	/* MIFARE Classic protocol, 4KB */
#define	PICC_TYPE_MIFARE_UL			0x07	/* MIFARE Ultralight or Ultralight C */
#define	PICC_TYPE_MIFARE_PLUS		0x08	/* MIFARE Plus*/
#define	PICC_TYPE_MIFARE_DESFIRE	0x09	/* MIFARE DESFire*/
#define	PICC_TYPE_TNP3XXX				0x0A	/* Only mentioned in NXP AN 10833 MIFARE Type Identification Procedure */
#define	PICC_TYPE_NOT_COMPLETE	0xFF

#define CommandReg			0x01
#define ComIrqReg				0x04
#define DivIrqReg				0x05
#define ErrorReg				0x06
#define Status2Reg			0x08
#define FIFODataReg			0x09
#define FIFOLevelReg		0x0A
#define ControlReg			0x0C
#define BitFramingReg		0x0D
#define CollReg					0x0E
#define ModeReg 				0x11
#define TxModeReg 			0x12
#define RxModeReg 			0x13
#define TxControlReg		0x14
#define TxASKReg 				0x15
#define ModWidthReg 		0x24
#define RFCfgReg				0x26
#define TModeReg 				0x2A
#define TPrescalerReg 	0x2B
#define TReloadRegH 		0x2C
#define TReloadRegL 		0x2D

#define PCD_Idle				0x00
#define PCD_CalcCRC			0x03
#define PCD_Transceive	0x0C
#define PCD_MFAuthent		0x0E
#define PCD_SoftReset		0x0F

#define PICC_CMD_REQA						0x26
#define PICC_CMD_MF_READ				0x30
#define PICC_CMD_HLTA						0x50
#define PICC_CMD_MF_AUTH_KEY_A	0x60
#define PICC_CMD_CT							0x88
#define	PICC_CMD_SEL_CL1				0x93
#define	PICC_CMD_SEL_CL2				0x95
#define	PICC_CMD_SEL_CL3				0x97
#define PICC_CMD_MF_WRITE				0xA0

#define CRCResultRegH		0x21
#define CRCResultRegL		0x22

#define MF_KEY_SIZE			0x06
#define MF_ACK					0x0A

#define nullptr				((void*)0)

/*Public*/
extern void PCD_Init(void);
extern void PCD_AntennaOn(void);
extern void PCD_AntennaOff(void);
extern bool PICC_IsNewCardPresent(void);
extern bool PICC_ReadCardSerial(void);
extern uint8_t PICC_GetType(uint8_t sak);
extern uint8_t PCD_Authenticate(uint8_t command, uint8_t blockAddr);
extern uint8_t MIFARE_Read(uint8_t blockAddr, uint8_t *buffer,	uint8_t *bufferSize);
extern uint8_t MIFARE_Write(uint8_t blockAddr, uint8_t *buffer, uint8_t bufferSize);
extern uint8_t PICC_HaltA(void);
extern void PCD_StopCrypto1(void);

/*Internal*/
void CS_Enable(void);
void CS_Disable(void);
void byte_array(uint8_t buffer);
uint8_t ReadWrite(uint8_t data);
uint8_t PCD_ReadRegister(uint8_t reg);
void PCD_WriteRegister(uint8_t reg, uint8_t value);
void PCD_WriteRegister_2(uint8_t reg, uint8_t count, uint8_t *values);
void PCD_ReadRegister_2(uint8_t reg, uint8_t count, uint8_t *values, uint8_t rxAlign);
void PCD_SetRegisterBitMask(uint8_t reg, uint8_t mask);
uint8_t PCD_CalculateCRC(uint8_t *data, uint8_t length, uint8_t *result);
void PCD_Reset(void);
uint8_t PCD_CommunicateWithPICC(uint8_t command, uint8_t waitIRq, uint8_t *sendData, uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC);
void PCD_ClearRegisterBitMask(uint8_t reg, uint8_t mask);
uint8_t PCD_TransceiveData(uint8_t *sendData,	uint8_t sendLen, uint8_t *backData, uint8_t *backLen, uint8_t *validBits, uint8_t rxAlign, bool checkCRC);
uint8_t PICC_REQA_or_WUPA(uint8_t command, uint8_t *bufferATQA,uint8_t *bufferSize);
uint8_t PICC_RequestA(uint8_t *bufferATQA, uint8_t *bufferSize);
uint8_t PICC_Select(uint8_t validBits);
uint8_t PCD_MIFARE_Transceive(uint8_t *sendData, uint8_t sendLen, bool acceptTimeout);
