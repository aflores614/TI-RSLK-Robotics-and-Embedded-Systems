/**
 * @file EUSCI_A3_SPI.c
 * @brief Source code for the EUSCI_A3_SPI driver.
 *
 * This file contains the function definitions for the EUSCI_A3_SPI driver.
 *
 * @note This function assumes that the necessary pin configurations for SPI communication have been performed
 *       on the corresponding pins.
 *       - P9.4 (SCE, Slave Chip Enable)
 *       - P9.5 (SCLK, Slave Clock)
 *       - P9.6 (MISO, Master In Slave Out)
 *       - P9.7 (MOSI, Master Out Slave In)
 *
 * @note For more information regarding the eUSCI_SPI registers used, refer to the eUSCI_A SPI Registers section (25.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * 
 *
 */

#include "../inc/EUSCI_A3_SPI.h"

void EUSCI_A3_SPI_Init(uint8_t chip_select_mode, uint32_t clock_frequency)
{
    // If chip_select_mode is 1, configure the P9.4 pin (Chip Select) to use the
    // primary module function and be controlled by the STE pin to generate
    // the Chip Select signal automatically
    if (chip_select_mode == 0x01)
    {
        // Configure the following pins to use the primary module function
        // by setting Bits 7 to 4 in the SEL0 register and
        // clearing Bits 7 to 4 in the SEL1 register
        //   - P9.7 (UCA3SIMO) [MOSI]
        //   - P9.6 (UCA3SOMI) [MISO]
        //   - P9.5 (UCA3CLK)  [SCLK]
        //   - P9.4 (UCA3STE)  [CS]
        P9->SEL0 |= 0xF0;
        P9->SEL1 &= ~0xF0;
    }

    // Otherwise, if chip_select_mode is 0, the P9.4 pin (Chip Select) will not be configured to use the primary module function.
    // Instead, the P9.4 pin will be configured as an output GPIO pin so that the Chip Select signal can be manually toggled
    else if (chip_select_mode == 0x00)
    {
        // Configure the following pins to use the primary module function
        // by setting Bits 7 to 5 in the SEL0 register and
        // clearing Bits 7 to 5 in the SEL1 register
        //   - P9.7 (UCA3SIMO) [MOSI]
        //   - P9.6 (UCA3SOMI) [MISO]
        //   - P9.5 (UCA3CLK)  [SCLK]
        P9->SEL0 |= 0xE0;
        P9->SEL1 &= ~0xE0;

        // Configure the P9.4 pin (Chip Select) as an output GPIO pin by clearing Bit 4 in the
        // SEL0 and SEL1 registers and setting Bit 4 in the DIR register
        P9->SEL0 &= ~0x10;
        P9->SEL1 &= ~0x10;
        P9->DIR |= 0x10;

        // Initialize the output of the Chip Select signal (active low) to high
        // by setting Bit 4 in the OUT register
        P9->OUT |= 0x10;
    }

    // Hold the EUSCI_A3 module in the reset state by setting the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x01;

    // Configure the EUSCI_A3 module to operate in SPI Mode 0.
    // Set the UCCKPH bit (Bit 15) to allow data to be captured on the first SPI clock edge and
    // changed on the following edge. Then, clear the UCCKPL bit (Bit 14) to configure
    // the SPI clock to be low when it is inactive
    EUSCI_A3->CTLW0 |= 0x8000;
    EUSCI_A3->CTLW0 &= ~0x4000;

    // Set the bit order to Most Significant Bit (MSB) first by setting the UCMSB bit (Bit 13) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x2000;

    // Select 8-bit character length by clearing the UC7BIT bit (Bit 12) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x1000;

    // Select master mode by setting the UCMST bit (Bit 11) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x0800;

    // Configure the mode of the EUSCI_A3 module to be 4-pin SPI with active low UCSTE (chip select)
    // by writing 10b to the UCMODEx field (Bits 10-9). This can be done by setting Bit 10
    // and clearing Bit 9 in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x0400;
    EUSCI_A3->CTLW0 &= ~0x0200;

    // Enable synchronous mode to allow the EUSCI_A3 module to use SPI by
    // setting the UCSYNC bit (Bit 8) in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x0100;

    // Select the eUSCI clock source to SMCLK by writing 11b to the UCSSELx field (Bits 7-6).
    // This can be done by setting Bit 7 and Bit 6 in the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x00C0;

    // Configure the UCSTE pin to be used to generate the enable signal (chip select) for a 4-wire slave
    EUSCI_A3->CTLW0 |= 0x0002;

    // Set the baud rate value by writing to the UCBRx field (Bits 15 to 0) in the BRW register
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 1,000,000)
    // Use only the integer part, so N = 12
    EUSCI_A3->BRW = (12000000 / clock_frequency);

    // Disable the following interrupts by clearing the
    // corresponding bits in the IE register:
    // - Transmit Complete Interrupt (UCTXCPTIE, Bit 3)
    // - Start Bit Interrupt (UCSTTIE, Bit 2)
    EUSCI_A3->IE &= ~0x0C;

    // Enable the following interrupts by setting the
    // corresponding bits in the IE register
    // - Transmit Interrupt (UCTXIE, Bit 1)
    // - Receive Interrupt (UCRXIE, Bit 0)
    EUSCI_A3->IE |= 0x03;

    // Release the EUSCI_A3 module from the reset state by clearing the
    // UCSWRST bit (Bit 0) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x01;
}

uint8_t EUSCI_A3_SPI_Data_Read()
{
    // Check the Receive Interrupt flag (UCRXIFG, Bit 0)
    // in the IFG register and wait if the flag is not set
    // If the UCRXIFG is set, then the Receive Buffer (UCAxRXBUF) has
    // received a complete character
    while((EUSCI_A3->IFG & 0x01) == 0);

    // Return the data from the Receive Buffer (UCAxRXBUF)
    // Reading the UCAxRXBUF will reset the UCRXIFG flag
    return EUSCI_A3->RXBUF;
}

void EUSCI_A3_SPI_Data_Write(uint8_t data)
{
    // Check the Transmit Interrupt flag (UCTXIFG, Bit 1)
    // in the IFG register and wait if the flag is not set
    // If the UCTXIFG is set, then the Transmit Buffer (UCAxTXBUF) is empty
    while((EUSCI_A3->IFG & 0x02) == 0);

    // Write the data to the Transmit Buffer (UCAxTXBUF)
    // Writing to the UCAxTXBUF will clear the UCTXIFG flag
    EUSCI_A3->TXBUF = data;
}
