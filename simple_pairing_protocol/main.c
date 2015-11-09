#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "spp_interface.h"

#define EPHEMERAL_SRC       0
#define ANY_DEST            0
#define TEST_BEACON_DATA   99

int main (int argc, char *argv[])
{
  int comPort = 2;
  char mode[3] = {'8', 'N', '1'};
  char testBuff[18] = "Testing this out!";
  SPP_DataPacket_t packet;
  bool paired = false;

  /* Get the RS-232 port from the argument list */
  if (argc == 2)
  {
    comPort = atoi(argv[1]);
    printf("Using COM%d\n", comPort);
  }
  else
  {
    printf("Defaulting to COM2\n");
  }

  if (SPP_InitiatePairing(EPHEMERAL_SRC, ANY_DEST, comPort) == SPP_FAILURE)
  {
    printf("Could not initiate Pairing. Listening for devices.\n");
  }

  while (1)
  {
    /* Send sensor data if paired to another device */
    if (paired)
    {
      if (SPP_TransmitBeacon(TEST_BEACON_DATA) == SPP_FAILURE)
      {
        printf("Failed to transmit sensor data\n");
      }
    }

    /* Listen for messages from other devices */
    if (SPP_Listen(&packet) == SPP_SUCCESS)
    {
      switch (packet.action)
      {
        case HELLO:
        {
          /* A pairing was initiated by another device */
          printf("Paired with device (address %d)\n", packet.source);
          paired = true;
          break;
        }
        case BEACON:
        {
          printf("Received sensor data: %d\n", packet.sensorData);
          break;
        }
        case GOODBYE:
        {
          printf("Pairing with device (address %d) was terminated\n", packet.source);
          break;
        }
        case ACK:
        default:
        {
          /* Ignore the message */
          break;
        }
      }
    }
  }

}
