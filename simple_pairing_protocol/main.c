#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "spp_interface.h"

#define SLEEP_PERIOD_SEC      1
#define COOLDOWN_PERIOD_SEC   5

int main (int argc, char *argv[])
{
  int32_t comPort = 2;
  uint8_t sensorData = 0;
  int32_t cooldown = 0;

  /* Get the RS-232 port from the argument list */
  if (argc == 2)
  {
    comPort = atoi(argv[1]);
    printf("MAIN: Using COM%d\n", comPort);
  }
  else
  {
    printf("MAIN: Defaulting to COM2\n");
  }

  if (SPP_InitiatePairing(EPHEMERAL_SRC, TO_ANY_DEVICE, comPort) == SPP_FAILURE)
  {
    printf("MAIN: Could not initiate pairing\n");
  }

  while (1)
  {
    if (SPP_IsPaired())
    {
      /* Update sensor data if paired to another device */
      sensorData++;
      SPP_UpdateSensorData(sensorData);
    }
    else
    {
      /* If not paired, try again after a cooldown */
      if (cooldown >= COOLDOWN_PERIOD_SEC)
      {
        printf("MAIN: Re-attempting to initiate pairing\n");

        /* First free up the RS-232 port */
        SPP_TerminatePairing();

        /* Then initiate a new pairing */
        if (SPP_InitiatePairing(EPHEMERAL_SRC, TO_ANY_DEVICE, comPort) == SPP_FAILURE)
        {
          printf("MAIN: Pairing attempt failed\n");
        }
        cooldown = 0;
      }
      else
      {
        cooldown++;
      }
    }

    /* Execute the simple pairing protocol */
    if (SPP_ServiceProtocol() == SPP_FAILURE)
    {
      //printf("SPP failure\n");
    }

    sleep(SLEEP_PERIOD_SEC);
  }

}
