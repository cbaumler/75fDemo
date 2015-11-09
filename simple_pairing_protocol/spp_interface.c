
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "spp_interface.h"

/* RS-232 Baud Rate */
#define BAUD_RATE   9600

/* A timeout to prevent waiting forever for an ACK message */
#define ACK_TIMEOUT   1000

/* Number of seconds in 10 minutes */
#define TEN_MINS_IN_SECS   600

/* The address of this device */
static uint16_t theSource = 0;

/* The address of the paired device */
static uint16_t theDestination = 0;

/* The RS-232 port to use for communication with the paired device */
static uint32_t theComPort = 0;

/* The RS-232 mode settings */
static const char theMode[3] = {'8', 'N', '1'};

/* The last time a BEACON message was sent by this device */
static time_t theLastBeaconTransmission = 0;

/*
 * Compute a 16-bit checksum on an array of bytes. The implementation is
 * based on RFC 1071 - Computing the Internet checksum. Returns the checksum.
 */
static uint16_t checksum(uint8_t *addr, uint32_t numBytes)
{
  uint32_t checksum = 0;

  /* Sum bytes */
  while (numBytes)
  {
    checksum += *addr;
    addr++;
    numBytes--;
  }

  /* Fold 32-bit sum to 16 bits */
  while (checksum >> 16)
  {
    checksum = (checksum & 0xFFFF) + (checksum >> 16);
  }

  return(~checksum);
}

/* Compute a checksum on an SPP packet. Returns the checksum. */
static uint16_t packetChecksum(SPP_DataPacket_t *packet)
{
  /* Compute a checksum over the packet (excluding the checksum field) */
  return(checksum((char*)(packet), (sizeof(SPP_DataPacket_t) - sizeof(uint16_t))));
}

/* Create an SPP ACK data packet, and send it to the specified destination */
static int32_t sendAck(uint16_t destination)
{
  int32_t retval = SPP_FAILURE;
  SPP_DataPacket_t packet;
  int32_t          bytesSent;

  /* Create an ACK packet */
  packet.source = theSource;
  packet.destination = destination;
  packet.action = ACK;
  packet.sensorData = 0;
  packet.checksum = packetChecksum(&packet);

  /* Send the ACK to the paired device */
  bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));
  if (bytesSent == sizeof(packet))
  {
    retval = SPP_SUCCESS;
  }

  return (retval);
}

static int32_t receiveAck(SPP_DataPacket_t *packet)
{
  int32_t          timeout = 0;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesReceived;

  /* Block until a valid ACK is received or a timeout occurs */
  while (timeout < ACK_TIMEOUT)
  {
    bytesReceived = RS232_PollComport(theComPort, packet, sizeof(SPP_DataPacket_t));

    /* Verify that an ACK was received */
    if ((bytesReceived == sizeof(SPP_DataPacket_t)) && (packet->action == ACK))
    {
        retval = SPP_SUCCESS;
        break; /* Break out of the while loop */
    }

    timeout++;
  }

  return (retval);
}

/*
 *
 *
 * Inputs:
 *   source      - optional address to use for this device (use 0 to have
 *                 one assigned at random)
 *   destination - optional address of device to pair with (use 0 to pair
 *                 with first device to respond)
 *   comPort     - The RS-232 port to use for communication
 *
 * Returns:
 *   SPP_SUCCESS if pairing is successful
 *   SPP_FAILURE if pairing fails
 *
 * Notes:
 *   When relying on the protocol implemention to randomly assign a source
 *   address, the possibility of address collisions exists. To avoid collisions,
 *   provide a known unique address. Managing an address space shared across
 *   multiple devices is outside the scope of this protocol.
 */
int32_t SPP_InitiatePairing(uint16_t source, uint16_t destination, uint32_t comPort)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesSent = -1;
  int32_t          bytesReceived = 0;
  int32_t          timeout = 0;

  /* Only allow a new pairing if not currently paired to a device */
  if (theDestination == 0)
  {
    if (source)
    {
      /* Use a provided source address if available */
      theSource = source;
    }
    else
    {
      /* Generate a random source address (similar to an ephemeral port) */
      srand(time(NULL));
      theSource = (uint16_t)(rand());
    }
    packet.source = theSource;

    /* Create the SPP packet */
    packet.source = theSource;
    packet.destination = destination;
    packet.action = HELLO;
    packet.sensorData = 0;
    packet.checksum = packetChecksum(&packet);

    /* Open the comm port */
    theComPort = comPort;
    if (RS232_OpenComport(theComPort, BAUD_RATE, theMode) != 0)
    {
      /* Send the HELLO message */
      bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));
    }
  }

  if (bytesSent == sizeof(packet))
  {
    /* Wait for an ACK */
    if (receiveAck(&packet) == SPP_SUCCESS)
    {
      /* If a device was specified, make sure the right device responded. */
      if ((destination == 0) || (destination == packet.source))
      {
        /* Send an ACK to complete the three-way handshake */
        if (sendAck(packet.source) == SPP_SUCCESS)
        {
          /* The devices are now effectively paired. Save the destination. */
          theDestination = packet.source;
          retval = SPP_SUCCESS;
        }
      }
    }
  }

  return (retval);
}

/*
 * Note: SPP_InitiatePairing opens the com port whether or not it succeeds.
 * Thus, this function should be called to close the com port even if no pairing
 * has taken place.
 */
int32_t SPP_TerminatePairing(void)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesSent;

  /* Check if a pairing has been initiated */
  if (theDestination != 0)
  {
    /* Create the SPP packet */
    packet.source = theSource;
    packet.destination = theDestination;
    packet.action = GOODBYE;
    packet.sensorData = 0;
    packet.checksum = packetChecksum(&packet);

    /* Send the GOODBYE message */
    bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));

    if (bytesSent == sizeof(packet))
    {
      /* Wait for an ACK */
      if (receiveAck(&packet) == SPP_SUCCESS)
      {
        /* Pairing has been successfully terminated */
        retval = SPP_SUCCESS;
      }
    }
  }
  else
  {
    /* If a pairing has not been initiated, just return success */
    retval = SPP_SUCCESS;
  }

  /* Close the RS-232 port to free up resources */
  RS232_CloseComport(theComPort);
}

int32_t SPP_TransmitBeacon(uint8_t sensorData)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  time_t           transmissionTime;
  time_t           currentTime;
  int32_t          bytesSent;

  /* Calculate the time at which a new BEACON should be sent */
  transmissionTime = theLastBeaconTransmission + TEN_MINS_IN_SECS;

  /* Get the current time */
  currentTime = time(NULL);

  /* Check if a pairing has been initiated and it's time to send a BEACON */
  if ((theDestination != 0) && (currentTime >= transmissionTime))
  {
    /* Create the SPP packet */
    packet.source = theSource;
    packet.destination = theDestination;
    packet.action = BEACON;
    packet.sensorData = sensorData;
    packet.checksum = packetChecksum(&packet);

    /* Send the BEACON message */
    bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));

    if (bytesSent == sizeof(packet))
    {
      /* Wait for an ACK */
      if (receiveAck(&packet) == SPP_SUCCESS)
      {
        /* BEACON was sent and received successfully. Update the time. */
        theLastBeaconTransmission = currentTime;
        retval = SPP_SUCCESS;
      }
    }
  }

  return (retval);
}

int32_t SPP_Listen(SPP_DataPacket_t *receivePacket)
{
  int32_t          bytesReceived;
  int32_t          bytesSent;
  int32_t          retval = SPP_FAILURE;
  SPP_DataPacket_t sendPacket;

  bytesReceived = RS232_PollComport(theComPort, receivePacket, sizeof(SPP_DataPacket_t));

  if ((bytesReceived == sizeof(SPP_DataPacket_t)))
  {
    switch (receivePacket->action)
    {
      case HELLO:
      {
        /* Only acknowledge a pairing if the sender is addressing the first
         * device to respond or this device specifically, and this device is
         * not already paired.
         */
        if ((receivePacket->destination == 0) || (receivePacket->destination == theSource) &&
            (theDestination == 0))
        {
          /* Send an ACK as the second part of the three-way handshake */
          if (sendAck(receivePacket->source) == SPP_SUCCESS)
          {
            /* Wait for an ACK to complete the handshake */
            if (receiveAck == SPP_SUCCESS)
            {
              /* Devices have been paired. Save the other device's address. */
              theDestination = receivePacket->source;
              retval = SPP_SUCCESS;
            }
          }
        }
        break;
      }
      case BEACON:
      {
        /* Make sure the packet was sent to this device from its paired device */
        if ((receivePacket->destination == theSource) && (receivePacket->source = theDestination))
        {
          /* Acknowledge receipt of the BEACON */
          if (sendAck(theDestination) == SPP_SUCCESS)
          {
            retval = SPP_SUCCESS;
          }
        }
        break;
      }
      case GOODBYE:
      {
        /* Make sure the packet was sent to this device from its paired device*/
        if ((receivePacket->destination == theSource) && (receivePacket->source = theDestination))
        {
          /* Acknowledge termination of the pairing */
          if (sendAck(receivePacket->source) == SPP_SUCCESS)
          {
            /* Discard the other device's address to free this device up for a new pairing */
            theDestination = 0;
            retval = SPP_SUCCESS;
          }
        }
        break;
      }
      case ACK:
      default:
      {
        /* Ignore message */
        break;
      }
    }
  }

  return (retval);
}
