#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include "spp_interface.h"

/* Macro Definitions */

/* Period of time to sleep between executing the main loop */
#define SLEEP_PERIOD_SEC        1

/* Period of time to wait before re-attempting pairing */
#define COOLDOWN_PERIOD_SEC    10

/* Maximum amount of time by which to vary the cooldown period */
#define MAX_COOLDOWN_RAND_SEC   5

/* Function Declarations */

/* This function tests the SPP interface. It accepts an integer representing
 * the RS-232 port to use as an input when the executable is launched from
 * the command line. If none is specified, it defaults to the 2nd port on
 * the host machine. This function services the SPP every SLEEP_PERIOD_SEC
 * seconds. When unpaired, it attempts to pair periodically as defined by
 * the cool down period. Once a pairing has been achieved, the sensor data
 * is periodically updated.
 */
int main (int argc, char *argv[])
{
  int32_t  comPort = 2;
  uint8_t  sensorData = 0;
  uint32_t cooldown = 0;

  srand(time(NULL));

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

  /*
   * Attempt to pair with any other device using an ephemeral address for
   * this device.
   */
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

        /*
         * Then initiate a new pairing with any device using a new ephemeral
         * address in case an address collision occurred previously.
         */
        if (SPP_InitiatePairing(EPHEMERAL_SRC, TO_ANY_DEVICE, comPort) == SPP_FAILURE)
        {
          printf("MAIN: Pairing attempt failed\n");
        }
        /*
         * Since pairing will fail if both devices attempt to initiate
         * simultaneously, introduce randomness into the cooldown period. Note
         * that a more robust random number generator would be preferred for a
         * production application. For example a cryptographically secure
         * seed-based generator could be used.
         */
        cooldown = ((uint32_t)(rand()) % MAX_COOLDOWN_RAND_SEC);
      }
      else
      {
        cooldown++;
      }
    }

    /*
     * Service the simple pairing protocol. This will receive messages from
     * other devices, send beacons, and handle lost connections. Error checking
     * is not performed here, because this implementation has opted to rely
     * primarily on console prints for error tracking rather than using a robust
     * set of error codes. This was done for convenience and would be changed in
     * a release version of the software.
     */
    SPP_ServiceProtocol();

    sleep(SLEEP_PERIOD_SEC);
  }

}
