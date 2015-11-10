
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "spp_interface.h"

/* RS-232 Baud Rate */
#define BAUD_RATE         9600

/* A timeout to prevent waiting forever for an ACK message */
#define ACK_TIMEOUT        5000

/* A timeout to prevent waiting forever to receive data on the RS-232 port */
#define READ_TIMEOUT       1000

/* Number of seconds in 10 minutes */
#define TEN_SECS             10

/* Return Codes for the RS-232 interface */
#define RS232_SUCCESS         0
#define RS232_FAILURE         1

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

/* Structure for buffering an SPP packet */
typedef struct
{
  SPP_DataPacket_t buf;
  int32_t          idx;
} PacketBuffer_t;

/* A buffer to store a partially received SPP packet */
PacketBuffer_t thisDeviceDataBuffer = {.idx = 0};

/* The address of this device */
static uint16_t thisDeviceAddress = 0;

/* The address of the paired device */
static uint16_t pairedDeviceAddress = 0;

/* Flag to keep track of whether the device has been paired */
static bool devicePaired = false;

/* Flag to prevent Beacon collisions where both devices are waiting for an ACK */
static bool clearToSendBeacon = false;

/* The RS-232 port to use for communication with the paired device */
static uint32_t theComPort = 0;

/* The RS-232 mode settings */
static const char theMode[4] = {'8', 'N', '1', '\0'};

/* The last time a BEACON was sent by this device */
static time_t thisDeviceLastBeaconTx = 0;

/* The last time a Beacon was received by this device */
static time_t thisDeviceLastBeaconRx = 0;

/* Sensor data reported by this device */
static uint8_t thisDeviceSensorData = 0;

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

/*
 * Without blocking, copy all or part of a packet from the RS-232 port into
 * thisDeviceDataBuffer. Return 0 if a whole packet is not saved in the buffer.
 * Return the packet size if a whole packet is saved in the buffer,
 * and copy the packet into the location provided by the caller.
 */
int32_t readComPortNonBlocking(SPP_DataPacket_t *buf)
{
  int32_t bytesRemaining;
  int32_t bytesReceived;
  int32_t retval = 0;
  unsigned char *writePtr;

  /* Set the write pointer to the location in the data buffer indicated by the index */
  writePtr = (unsigned char *)(&thisDeviceDataBuffer.buf) + thisDeviceDataBuffer.idx;

  /* Compute the space remaining in the data buffer */
  bytesRemaining = sizeof(SPP_DataPacket_t) - thisDeviceDataBuffer.idx;

  /* Attempt to receive data from the RS-232 port */
  bytesReceived = RS232_PollComport(theComPort, writePtr, bytesRemaining);
  thisDeviceDataBuffer.idx += bytesReceived;

  /* Check if the buffer has been filled yet */
  if (thisDeviceDataBuffer.idx >= sizeof(SPP_DataPacket_t))
  {
    /* Copy the data from the buffer into the packet provided as an argument */
    memcpy(buf, &thisDeviceDataBuffer.buf, sizeof(SPP_DataPacket_t));

    /* Clear the buffer */
    thisDeviceDataBuffer.idx = 0;

    /* Set the return value to indicate that a packet has been received */
    retval = sizeof(SPP_DataPacket_t);
  }

  return (retval);
}

/*
 * Read a message from the RS-232 port. Blocks until the entire message is
 * received or the timeout occurs. Returns the number of bytes received.
 */
int32_t readComPortBlocking(SPP_DataPacket_t *buf)
{
  int32_t timeout = 0;
  int32_t totalReceived = 0;
  int32_t bytesReceived;
  int32_t bytesRemaining = sizeof(SPP_DataPacket_t);
  unsigned char *writePtr = (unsigned char *)(buf);

  while ((bytesRemaining > 0) && (timeout < READ_TIMEOUT))
  {
    /* Read up to bytesRemaining bytes from the port and save them in buf */
    bytesReceived = RS232_PollComport(theComPort, writePtr, bytesRemaining);
    totalReceived += bytesReceived;
    bytesRemaining -= bytesReceived;
    writePtr += bytesReceived;
    timeout++;
  }

  return (totalReceived);
}

/* Create an SPP ACK data packet, and send it to the specified destination */
static int32_t sendAck(uint16_t destination)
{
  int32_t retval = SPP_FAILURE;
  SPP_DataPacket_t packet;
  int32_t          bytesSent;

  /* Create an ACK packet */
  packet.source = thisDeviceAddress;
  packet.destination = destination;
  packet.action = ACK;
  packet.sensorData = 0;
  packet.checksum = packetChecksum(&packet);

  /* Send the ACK to the destination device */
  bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));
  if (bytesSent == sizeof(packet))
  {
    retval = SPP_SUCCESS;
  }

  return (retval);
}

static int32_t receiveAck(uint16_t *sourceAddr)
{
  int32_t          timeout = 0;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesReceived;
  SPP_DataPacket_t packet;

  /* Block until a valid ACK is received or a timeout occurs */
  while (timeout < ACK_TIMEOUT)
  {
    bytesReceived = readComPortBlocking(&packet);

    /* Verify that an ACK was received */
    if ((bytesReceived == sizeof(packet)) && (packet.action == ACK))
    {
      /* Verify that the ACK was sent to this device */
      if (packet.destination = thisDeviceAddress)
      {
        /* If this device is paired, make sure the ACK came from the paired device */
        if ((devicePaired && (pairedDeviceAddress == packet.source)) || !devicePaired)
        {
          /* Return the source address of the ACK */
          *sourceAddr = packet.source;

          /* ACK was successful */
          retval = SPP_SUCCESS;
          break; /* Break out of the while loop */
        }
      }
    }

    timeout++;
  }

  return (retval);
}

static int32_t transmitBeacon(void)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  time_t           transmissionTime;
  time_t           currentTime;
  int32_t          bytesSent;
  uint16_t         ackSrcAddr;

  /* Calculate the time at which a new BEACON should be sent */
  transmissionTime = thisDeviceLastBeaconTx + TEN_SECS;

  /* Get the current time */
  currentTime = time(NULL);

  /* Check if it's time to send a BEACON */
  if (currentTime >= transmissionTime)
  {
    /* Create the SPP packet */
    packet.source = thisDeviceAddress;
    packet.destination = pairedDeviceAddress;
    packet.action = BEACON;
    packet.sensorData = thisDeviceSensorData;
    packet.checksum = packetChecksum(&packet);

    /* Send the BEACON */
    printf("SPP: Sending BEACON to %d\n", packet.destination);
    bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));

    if (bytesSent == sizeof(packet))
    {
      /* Wait for an ACK */
      if (receiveAck(&ackSrcAddr) == SPP_SUCCESS)
      {
        /* BEACON was sent and received successfully. Update the time. */
        thisDeviceLastBeaconTx = currentTime;
        retval = SPP_SUCCESS;
        printf("SPP: Got BEACON ACK from %d\n", ackSrcAddr);
      }
    }
  }

  return (retval);
}

static int32_t processIncomingMessages(void)
{
  int32_t          bytesReceived;
  int32_t          bytesSent;
  uint16_t         ackSrcAddr;
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;

  bytesReceived = readComPortNonBlocking(&packet);

  if ((bytesReceived == sizeof(packet)))
  {
    switch (packet.action)
    {
      case HELLO:
      {
        printf("SPP: Got HELLO from %d\n", packet.source);
        if (devicePaired)
        {
          /* Ignore HELLO messages when the device is already paired */
        }
        /* Only acknowledge a pairing if the sender is addressing the first
         * device to respond or this device specifically
         */
        else if ((packet.destination == TO_ANY_DEVICE) ||
                 (packet.destination == thisDeviceAddress))
        {
          /* Send an ACK as the second part of the three-way handshake */
          if (sendAck(packet.source) == SPP_SUCCESS)
          {
            printf("SPP: Sending HELLO ACK to %d\n", packet.source);

            /* Wait for an ACK to complete the handshake */
            if (receiveAck(&ackSrcAddr) == SPP_SUCCESS)
            {
              /* Make sure the ACK came from the same device that initiated pairing */
              if (ackSrcAddr == packet.source)
              {
                /* Devices have been paired. Save the paired device's address. */
                pairedDeviceAddress = packet.source;
                /* Reset the record of when the last BEACON was received */
                thisDeviceLastBeaconRx = time(NULL);
                devicePaired = true;
                retval = SPP_SUCCESS;
                printf("SPP: Successfully paired with %d\n", packet.source);
              }
            }
          }
        }
        break;
      }
      case BEACON:
      {
        /* Only accept BEACON messages if a pairing has been established */
        if (devicePaired)
        {
          /* Make sure the packet was sent to this device from its paired device */
          if ((packet.destination == thisDeviceAddress) &&
              (packet.source = pairedDeviceAddress))
          {
            printf("SPP: Got BEACON from %d with data: %d\n", packet.source, packet.sensorData);

            /* Acknowledge receipt of the BEACON */
            if (sendAck(pairedDeviceAddress) == SPP_SUCCESS)
            {
              /* Update the record of when the last BEACON was received */
              thisDeviceLastBeaconRx = time(NULL);

              /* Having received a BEACON from the pairing initiator, this
               * device is now clear to send Beacons as well
               */
              clearToSendBeacon = true;

              retval = SPP_SUCCESS;
            }
          }
        }
        break;
      }
      case GOODBYE:
      {
        printf("SPP: Got GOODBYE from %d\n", packet.source);

        /* Only accept GOODBYE messages if a pairing has been established */
        if (devicePaired)
        {
          /* Make sure the packet was sent to this device from its paired device*/
          if ((packet.destination == thisDeviceAddress) &&
              (packet.source = pairedDeviceAddress))
          {
            /* Acknowledge termination of the pairing */
            if (sendAck(packet.source) == SPP_SUCCESS)
            {
              /* Devices have been unpaired */
              pairedDeviceAddress = 0;
              devicePaired = false;
              retval = SPP_SUCCESS;
              printf("SPP: Terminated pairing with %d\n", packet.source);
            }
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

static void checkForLostConnection(void)
{
  time_t  currentTime;

  if (devicePaired)
  {
    currentTime = time(NULL);

    /* Check if the BEACON from the paired device is overdue */
    if (currentTime > (thisDeviceLastBeaconRx + TEN_SECS))
    {
      /* The other device is not sending BEACONS. Terminate the pairing. */
      printf("SPP: No BEACON received from %d\n", pairedDeviceAddress);
      SPP_TerminatePairing();
    }
  }
}

/*
 *
 *
 * Inputs:
 *   srcAddr  - optional address to use for this device (use 0 to have
 *              one assigned at random)
 *   destAddr - optional address of device to pair with (use 0 to pair
 *              with first device to respond)
 *   comPort  - The RS-232 port to use for communication
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
int32_t SPP_InitiatePairing(uint16_t srcAddr, uint16_t destAddr, uint32_t comPort)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesSent = -1;
  int32_t          bytesReceived = 0;
  int32_t          timeout = 0;
  uint16_t         ackSrcAddr;

  /* Only allow a new pairing if not currently paired to a device */
  if (!devicePaired)
  {
    if (srcAddr)
    {
      /* Use a provided source address if available */
      thisDeviceAddress = srcAddr;
    }
    else
    {
      /* Generate a random source address (similar to an ephemeral port) */
      srand(time(NULL));
      thisDeviceAddress = (uint16_t)(rand());
    }

    printf("SPP: This device has been assigned address %d\n", thisDeviceAddress);

    /* Create the SPP packet */
    packet.source = thisDeviceAddress;
    packet.destination = destAddr;
    packet.action = HELLO;
    packet.sensorData = 0;
    packet.checksum = packetChecksum(&packet);

    /* Open the comm port */
    theComPort = comPort;
    if (RS232_OpenComport(theComPort, BAUD_RATE, theMode) == RS232_SUCCESS)
    {
      /* Send the HELLO message */
      printf("SPP: Sending HELLO to %d\n", packet.destination);
      bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));
    }
  }

  if (bytesSent == sizeof(packet))
  {
    /* Wait for an ACK */
    if (receiveAck(&ackSrcAddr) == SPP_SUCCESS)
    {
      printf("SPP: Got HELLO ACK from %d\n", ackSrcAddr);

      /* If a device was specified, make sure the right device responded. */
      if ((destAddr == TO_ANY_DEVICE) || (destAddr == ackSrcAddr))
      {
        /* Send an ACK to complete the three-way handshake */
        printf("SPP: Sending final ACK to %d for device pairing\n", ackSrcAddr);
        if (sendAck(ackSrcAddr) == SPP_SUCCESS)
        {
          /* The devices are now effectively paired. Save the paired device address. */
          pairedDeviceAddress = ackSrcAddr;

          /* Reset the record of when the last BEACON was received */
          thisDeviceLastBeaconRx = time(NULL);

          /* Since this device initiated pairing, it should send the first Beacon */
          clearToSendBeacon = true;

          devicePaired = true;
          retval = SPP_SUCCESS;
          printf("SPP: Successfully paired with %d\n", ackSrcAddr);
        }
      }
    }
  }

  return (retval);
}

/*
 * Note: SPP_InitiatePairing opens the com port whether or not it succeeds.
 * Thus, this function should be called to close the com port even if no pairing
 * has taken place. Also, this device terminates the pairing on this end
 * whether or not the paired device is able to communicate an ACK back to it.
 * This prevents the device from being tied up by a failure in the other
 * device. The return code will indicate whether or not the pairing termination
 * was cleanly accomplished.
 */
int32_t SPP_TerminatePairing(void)
{
  SPP_DataPacket_t packet;
  int32_t          retval = SPP_FAILURE;
  int32_t          bytesSent;
  uint16_t         ackSrcAddr;

  /* Check if a pairing has been initiated */
  if (devicePaired)
  {
    /* Create the SPP packet */
    packet.source = thisDeviceAddress;
    packet.destination = pairedDeviceAddress;
    packet.action = GOODBYE;
    packet.sensorData = 0;
    packet.checksum = packetChecksum(&packet);

    /* Send the GOODBYE message */
    printf("SPP: Sending GOODBYE to %d\n", pairedDeviceAddress);
    bytesSent = RS232_SendBuf(theComPort, &packet, sizeof(packet));

    if (bytesSent == sizeof(packet))
    {
      /* Terminate the pairing whether or not an ACK is received. The other
       * device may not be communicating.
       */
      devicePaired = false;
      printf("SPP: Pairing with %d terminated\n", pairedDeviceAddress);

      /* Wait for an ACK */
      if (receiveAck(&ackSrcAddr) == SPP_SUCCESS)
      {
        printf("SPP: Got GOODBYE ACK from %d\n", ackSrcAddr);

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

bool SPP_IsPaired(void)
{
  return (devicePaired);
}

void SPP_UpdateSensorData(uint8_t sensorData)
{
  thisDeviceSensorData = sensorData;
}

int32_t SPP_ServiceProtocol(void)
{
  int32_t retval = SPP_SUCCESS;

  if (processIncomingMessages() == SPP_FAILURE)
  {
    retval = SPP_FAILURE;
    //printf("Failed to process incoming messages\n");
  }

  /* Terminate the pairing if the paired device is not sending a BEACON */
  checkForLostConnection();

  if (devicePaired && clearToSendBeacon)
  {
    /* Transmit a BEACON to the paired device */
    if (transmitBeacon() == SPP_FAILURE)
    {
      retval = SPP_FAILURE;
      //printf("Failed to transmit BEACON\n");
    }
  }

  return (retval);
}
