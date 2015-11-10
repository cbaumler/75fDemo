#75fDemo
A small demo project consisting of two parts:  
1. Implement a protocol that allows 'pairing' of two fictional devices.  
2. Create a skeleton for an embedded application to control the SPI on an Atmel ATxmega64B1 microprocessor.

##1. Simple Pairing Protocol

###Motivation
A simple protocol for pairing two fictional devices is desired. The pairing should be somewhat similar to how a headset might be paired with a phone. The Simple Pairing Protocol (SPP) is necessary to demonstrate comprehension of key concepts related to communication in an embedded environment.

###Scope
The SPP is intended to provide a simple process for pairing two devices which are using RS-232 for their underlying data transmission. The devices should be able to communicate sensor data to each other via periodic beacons.

###Interfaces
The SPP interfaces on one side to an embedded user application and on the other side to an RS-232 driver.

---

**`int32_t SPP_InitiatePairing(uint16_t srcAddr, uint16_t destAddr, uint32_t comPort)`**

> Open the RS-232 port and initiate a device pairing using a three-way handshake that starts with a HELLO packet. Assign the caller specified srcAddr as this device's address or generate a random device address if the caller specifies EPHEMERAL_SRC.

---

**`int32_t SPP_TerminatePairing(void)`**

> Terminate a device pairing by sending a GOODBYE and closing the RS-232 port. Block for an ACK from the paired device acknowledging the pairing termination.

---

**`bool SPP_IsPaired(void)`**
> Return the status of the current device pairing.

---

**`void SPP_UpdateSensorData(uint8_t sensorData)`**
> Update the internal record of the sensor data for this device. This data is sent to the paired device as part of the BEACON.

---

**`int32_t SPP_ServiceProtocol(void)`**
> Handle incoming messages from other devices. Check the connection to the paired device and terminate the pairing if the connection is lost. Lastly, send the BEACON to the paired device.

---

###Header Format
The SPP packet header includes the following fields:

**source:** This is the address of the source device.

**destination:** This is the address of the destination device.

**action:** This is the purpose of the message. It can have the following values:
- ACK: Acknowledge a message from another device
- HELLO: Initialize a pairing with another device
- BEACON: Transmit a sensor reading
- GOODBYE: Terminate a pairing

**sensorData:** This is arbitrary 8 bit binary sensor data.

**checksum:** This value is computed for error detection.

*Note on Address Association*  
When a device pairing is established, each device stores its own address and the address of the paired device. Once a pairing is accomplished, traffic from devices other than the paired device is ignored. Upon pairing termination, the address of the
paired device is forgotten. Also, when seeking to establish a new pairing, a device will assign itself a new address (see `SPP_InitiatePairing()`).

###Protocol Sequence
The following diagram depicts the sequence of messages necessary for the Simple Pairing Protocol.

![](https://github.com/cbaumler/75fDemo/blob/master/docs/protocol.png)

Note that beacons are sent every 10 seconds when devices are paired. The protocol was designed using an ACK message to confirm packet receipt. The ACK format was chosen over the NAK format because of the low data rate. Using ACKs, packet losses are detected almost immediately. If NAKs had been used, packet losses would not have been detected for as long as 10 seconds in some cases.

The three-way handshake during initial pairing is included for the case where a pairing is initiated with the destination address of TO_ANY_DEVICE. If multiple devices respond (ACK) to the initial HELLO, the initializing device will select the first response it receives and send an ACK. This way the devices that are ignored will not falsely assume that they have been paired.

In general, the protocol has been implemented such that ACKs are blocking. This ensures the correct protocol sequence is followed. A timeout ensures that a device does not wait forever for an ACK. 

Note that in this implementation with blocking ACKs, a deadlock of sorts will occur if both devices send a message and block waiting for an ACK until they time out. To prevent this in the case of the BEACON, the implementation enforces a particular order for sending BEACONs. The pairing initiator is allowed to transmit the first BEACON, and once it is received, the other device is allowed to begin sending BEACONs as well. This ensures that BEACON messages will be offset. In the case of pairing initialization via the HELLO message, the application using the protocol is expected to make additional attempts to pair. A random cool down period between attempts may be used to prevent repeated collisions. This issue is not as much of a concern for pairing termination via the GOODBYE message, as the pairing will be terminated by the requesting device even if the ACK is not received.

To prevent a device from becoming tied up where the other device has terminated the pairing (whether intentionally or through failure), the protocol dictates that a device will terminate the pairing automatically if a BEACON is not received every 10 seconds.

###Source
**simple_packet_protocol/main.c**  
This file contains a main function to test the SPP interface. It accepts an integer representing the RS-232 port to use as an input when the executable is launched from the command line. If none is specified, it defaults to the 2nd port on the host machine. This function services the SPP every SLEEP_PERIOD_SEC seconds. When unpaired, it attempts to pair periodically as defined by the cool down period. Once a pairing has been achieved, the sensor data is periodically updated.

**simple_packet_protocol/spp_interface.c**  
This file contains the Simple Pairing Protocol interface implementation as well as helper functions to perform underlying protocol tasks.

**simple_packet_protocol/rs232.c**  
This file contains a module for communicating via RS-232. It was authored by Teunis van Beelen and is [available](http://www.teuniz.net/RS-232/) via the GNU General Public License.

Note: This software was originally compiled on a Windows 7 machine using Cygwin. It was tested by running two instances connected to two virtual RS-232 ports running on the same machine.

###References
[RFC 793 Transmission Control Protocol](https://tools.ietf.org/html/rfc793)  
[RFC 1071 Computing The Internet Checksum](https://tools.ietf.org/html/rfc1071)  
[RS-232 for Linux, FreeBSD, and Windows](http://www.teuniz.net/RS-232/)

##2. ATxmega64B1 SPI Application

This application was developed using Atmel Studio 7 and was executed using the built in simulator.

**System Diagram**

![System Diagram](https://github.com/cbaumler/75fDemo/blob/master/docs/spi.png "System Diagram")

###Source

*Note: Source code has been annotated with detailed comments and references to the ATxmega64B1 manual and datasheet in an effort to "show work" and to explain rationales and thought processes.*

**75fDemo/main.c**  
This file contains a skeleton test application for communicating via Serial Peripheral Interface (SPI) on the Atmel Mega ATXMEGA64B1 processor. 

**75fDemo/dummy_driver.c**
This file contains a dummy driver that configures the ATXMEGA64B1 for SPI communication on port C as master. It also provides interrupt processing rules for the SPI peripheral, provides a set of functions for reading and writing data to a slave peripheral connected to the slave select on port C, and provides a function to dump an initialization message to the SPI bus. This driver uses a SPI driver to interface with the SPI module.

**75fDemo/spi_driver.c**  
This file contains functions implementing a SPI driver and was obtained from Atmel aplication note "AVR1309: Using the XMEGA SPI." I initially debated implementing my own source for this functionality, but the engineer in me would not allow me to reinvent the wheel. So I opted to reuse this code and attempted to demonstrate a thorough understanding of it. 

###References
AVR1309: Using the XMEGA SPI [[zip]](http://www.atmel.com/images/AVR1309.zip) [[pdf]](http://www.atmel.com/Images/doc8057.pdf)

[XMEGA B Manual](http://www.atmel.com/images/Atmel-8291-8-and-16-bit-AVR-Microcontrollers-XMEGA-B_Manual.pdf)

[ATxmega128B1 / ATxmega64B1 Datasheet](http://www.atmel.com/Images/Atmel-8330-8-and-16-bit-AVR-Microcontroller-XMEGA-B-ATxmega64B1-ATxmega128B1_Datasheet.pdf)
