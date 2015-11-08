#75fDemo
A small demo project to use SPI on the Atmel ATxmega64B1 microcontroller

##Source

*Note: Source code has been annotated with detailed comments and references to the ATxmega64B1 manual and datasheet in an effort to "show work" and to explain rationales and thought processes.*

**main.c**  
This file contains a skeleton application for communicating via Serial Peripheral Interface (SPI) on the Atmel Mega ATXMEGA64B1 processor. 

**spi_driver.c**  
This file contains functions implementing a SPI driver and was obtained from Atmel aplication note "AVR1309: Using the XMEGA SPI." I initially debated implementing my own source for this functionality, but the engineer in me would not allow me to reinvent the wheel. So I opted to reuse this code and attempted to demonstrate a thorough understanding of it. 

##References
AVR1309: Using the XMEGA SPI  
http://www.atmel.com/images/AVR1309.zip  
http://www.atmel.com/Images/doc8057.pdf

XMEGA B Manual  
http://www.atmel.com/images/Atmel-8291-8-and-16-bit-AVR-Microcontrollers-XMEGA-B_Manual.pdf

ATxmega128B1 / ATxmega64B1 Datasheet  
http://www.atmel.com/Images/Atmel-8330-8-and-16-bit-AVR-Microcontroller-XMEGA-B-ATxmega64B1-ATxmega128B1_Datasheet.pdf
