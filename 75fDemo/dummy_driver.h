/*
 * dummy_driver.h
 *
 *  Author: Chris Baumler
 * 
 *  This file contains functions for working with the SPI on
 *  the Atmel Mega ATXMEGA64B1. See source file for details.
 *  
 */ 

#ifndef DUMMY_DRIVER_H_
#define DUMMY_DRIVER_H_

/* Function Declarations */

/* Documentation Found in Source File */

void configureSpiMaster(void);

void transceiveData(const uint8_t *transmitData,
                    uint8_t *receiveData,
                    uint8_t bytesToTransceive);

void transmitByte(uint8_t byte);

uint8_t receiveByte();

uint8_t tranceiveByte(uint8_t byte);

void sendInitComplete(void);


#endif /* DUMMY_DRIVER_H_ */