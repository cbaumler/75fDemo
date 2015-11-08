/*
 *  main.c
 *
 *  Created: 11/7/2015
 *  Author : Chris Baumler
 * 
 *  This file contains a skeleton application for communicating via
 *  Serial Peripheral Interface (SPI) on the Atmel Mega ATXMEGA64B1
 *  processor. 
 */ 

#include "avr_compiler.h"
#include "spi_driver.h"

/* An initialization message to send on power-up */
#define INIT_MESSAGE        "power on message - init complete"

/* The size of the initialization message (include null terminator) */
#define INIT_MESSAGE_SIZE   33

/* SPI master on PORT C */
static SPI_Master_t spiMasterC;

/* Data structure to track transmitted and received data */
static SPI_DataPacket_t dataPacket;

/* 
 *  This function is an interrupt service routine for the SPI interrupt vector.
 *  It calls an interrupt handler function that reads incoming data in the
 *  SPI data register and writes outgoing data to the same register.
 *
 */
ISR(SPIC_INT_vect)
{
  SPI_MasterInterruptHandler(&spiMasterC);
}

/* 
 *  This function configures the ATXMEGA64B1 SPI peripheral module for communication 
 *  as a master. 
 *  
 *  This design makes the following assumptions in the absence of hard requirements:
 *
 *    There is only one other peripheral component, and it's attached to the 
 *    slave select (SS) on port C.
 *    Rationale: This simplifies implementation and test.
 *   
 *    A transfer mode is selected that sets the leading edge of a clock cycle to
 *    rising and the trailing edge to falling. Also, data sample occurs on the
 *    leading edge, and data setup occurs on the falling edge.
 *    Rationale: Clock phase and polarity must be compatible with the slave
 *    peripheral component. Since nothing is known about the slave, choose a 
 *    commonly used mode.
 *   
 *    The interrupt priority is set to low.
 *    Rationale: Since the criticality of SPI communication is unknown, select
 *    the lowest priority to avoid interfering with potentially critical functions.
 *    If necessary, round-robin scheduling can be enabled to avoid starvation. 
 *    This is done by setting PMIC_RREN_bm in the PMIC.CTRL register.
 *
 *    SCK Frequency is set to Clk_per/4, and double speed mode is disabled.
 *    Rationale: Since the requirements do not specify a priority between
 *    speed and power consumption, and the limits of the peripheral device
 *    are not known, a balanced clock speed is chosen. 
 *
 */
void configureSpiMaster(void)
{
  /* Configure the SS on pin 4 of port C as an output (Datasheet: Table 32-3) */
  PORTC.DIRSET = PIN4_bm;

  /* Configure pin 4 on port C for wired AND with pull-up resistor (Datasheet: Figure 15-6) */
  PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

  /* Set SS output to high to initially deselect the slave peripheral */
  PORTC.OUTSET = PIN4_bm;

  /* Initialize SPI module on port C as a master */
  SPI_MasterInit(&spiMasterC,
  &SPIC,                       /* Base address of the SPI on port C (Datasheet: Table 33-1) */
  &PORTC,                      /* Base address of port C (Datasheet: Table 33-1) */
  false,                       /* Specify MSB first */
  SPI_MODE_0_gc,               /* Specify SPI mode 0 (Manual: Table 20-2) */
  SPI_INTLVL_LO_gc,            /* Specify low priority interrupt level (Manual: Table 11-1) */
  false,                       /* Disable double speed mode (Manual: Section 21.3.3) */
  SPI_PRESCALER_DIV4_gc);      /* Specify a prescaler of /4 (Manual: Table 20-3) */

  /* Enable low, medium, and high level interrupts in the interrupt controller */
  PMIC.CTRL |= PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm;

  /* Set the global interrupt enable bit */
  sei();
}

/* 
 *  This function initiates a data transmission to the slave peripheral
 *  by setting SS low and writing the first byte to the data register. 
 *  It then loops until the remaining bytes have been transmitted/received.
 *  
 *  Once a byte has been shifted from the master shift register to the
 *  slave shift register, and likewise, a byte has been simultaneously 
 *  shifted from the slave shift register to the master shift register,
 *  (full duplex) the SPI interrupt flag is set (Manual: Section 20.3),
 *  triggering the SPI ISR. The ISR reads the byte from the slave, and
 *  writes an additional byte if necessary.
 */
void transceiveData(const uint8_t *transmitData,
                           uint8_t *receiveData,
                           uint8_t bytesToTransceive)
{
  uint8_t status;

  /* Create data packet for communication with the slave peripheral */
  SPI_MasterCreateDataPacket(&dataPacket,
  transmitData,
  receiveData,
  bytesToTransceive,
  &PORTC,
  PIN4_bm);
  
  /* Initiate data transmission. */
  do {
    status = SPI_MasterInterruptTransceivePacket(&spiMasterC, &dataPacket);
  } while (status != SPI_OK);

  /* Wait for transmission/receipt of remaining bytes to complete (handled by ISR). */
  while (dataPacket.complete == false) {

  }
}

/* 
 *  This function transmits a single byte of data over SPI using interrupts.
 */
void transmitByte(uint8_t byte)
{
  uint8_t txByte = byte;
  uint8_t rxByte = 0;

  transceiveData(&txByte, &rxByte, sizeof(uint8_t));
}

/* 
 *  This function receives a single byte of data over SPI using interrupts.
 */
uint8_t receiveByte()
{
  uint8_t txByte = 0;
  uint8_t rxByte = 0;

  transceiveData(&txByte, &rxByte, sizeof(uint8_t));

  return (rxByte);
}

/* 
 *  This function transmits and receives a single byte of data over SPI
 *  using interrupts.
 */
uint8_t tranceiveByte(uint8_t byte)
{
  uint8_t txByte = byte;
  uint8_t rxByte = 0;

  transceiveData(&txByte, &rxByte, sizeof(uint8_t));

  return (rxByte);
}

/* 
 *  This function sends an initialization complete message
 *  via the SPI bus.
 */
void sendInitComplete(void)
{
  uint8_t txMessage[INIT_MESSAGE_SIZE] = INIT_MESSAGE;
  uint8_t rxMessage[INIT_MESSAGE_SIZE];

  transceiveData(txMessage, rxMessage, INIT_MESSAGE_SIZE);
}

/* 
 *  This function initializes the SPI peripheral.
 *
 */
int main(void)
{
  /* Configure this device as the master on the SPI bus */
  configureSpiMaster();

  /* Send an initialization message to the slave peripheral over the SPI bus */
  sendInitComplete();

  while (1) 
  {
    nop();
  }
}
