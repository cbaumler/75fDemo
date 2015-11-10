/*
 *  main.c
 *
 *  Author : Chris Baumler
 * 
 *  This file contains a skeleton application for communicating via
 *  Serial Peripheral Interface (SPI) on the Atmel Mega ATXMEGA64B1
 *  microcontroller. 
 */ 

#include "avr_compiler.h"
#include "dummy_driver.h"

/* 
 *  This function demonstrates use of the dummy_driver to configure
 *  the SPI and send an initialization message over the SPI bus.
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
