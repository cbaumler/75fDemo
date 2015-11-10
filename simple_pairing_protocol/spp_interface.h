#ifndef SPP_INTERFACE_H
#define SPP_INTERFACE_H

#include <stdint.h>

/* Simple Pairing Protocol Return Codes */
#define SPP_SUCCESS     0
#define SPP_FAILURE     1

/* Destination address to specify that any device may respond */
#define TO_ANY_DEVICE   0

/* Source address to specify during pairing initiation to be assigned a random address */
#define EPHEMERAL_SRC       0

int32_t SPP_InitiatePairing(uint16_t source, uint16_t destination, uint32_t comPort);

int32_t SPP_TerminatePairing(void);

bool SPP_IsPaired(void);

void SPP_UpdateSensorData(uint8_t sensorData);

int32_t SPP_ServiceProtocol(void);

#endif /* SPP_INTERFACE_H */
