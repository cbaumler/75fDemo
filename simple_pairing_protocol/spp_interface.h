#ifndef SPP_INTERFACE_H
#define SPP_INTERFACE_H

#include <stdint.h>

#define SPP_SUCCESS   0
#define SPP_FAILURE   1

/* Enumeration of actions performed in the Simple Pairing Protocol */
typedef enum
{
  ACK,    /* Acknowledge a message from another device */
  HELLO,  /* Initialize a pairing with another device */
  BEACON, /* Transmit a sensor reading */
  GOODBYE /* Terminate a pairing */
} SPP_Action_t;

/* Simple Pairing Protocol Data Packet Structure */
typedef struct
{
  uint16_t     source;      /* Address of the source device */
  uint16_t     destination; /* Address of the destination device */
  SPP_Action_t action;      /* Purpose of the message */
  uint8_t      sensorData;  /* Arbitary 8 bit binary sensor data */
  uint16_t     checksum;    /* Computed for error detection */
} SPP_DataPacket_t;

int32_t SPP_InitiatePairing(uint16_t source, uint16_t destination, uint32_t comPort);

int32_t SPP_TerminatePairing(void);

int32_t SPP_TransmitBeacon(uint8_t sensorData);

int32_t SPP_Listen(SPP_DataPacket_t *receivePacket);

#endif /* SPP_INTERFACE_H */
