/*
 * Adapted from Texas Instruments supplied example code
 */
/*******************************************************************************
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "msp430.h"
#include "HAL_Board.h"

#define XT1_PORT_DIR             P7DIR
#define XT1_PORT_OUT             P7OUT
#define XT1_PORT_SEL             P7SEL
#define XT2_PORT_DIR             P5DIR
#define XT2_PORT_OUT             P5OUT
#define XT2_PORT_SEL             P5SEL
#define XT1_ENABLE               (BIT0 + BIT1)
#define XT2_ENABLE               (BIT2 + BIT3)


/***************************************************************************//**
 * @brief  Initialize the board - configure ports
 * @param  None
 * @return none
 ******************************************************************************/
void Board_init(void) {
   // Setup XT1 and XT2
   XT1_PORT_SEL |= XT1_ENABLE;
   XT2_PORT_SEL |= XT2_ENABLE;

   //FLASH_CS_N (same defaults as on Shimmer2r)
   P4OUT &= ~BIT0;            //set low
   P4DIR |= BIT0;             //set as output
   //FLASH_SIMO
   P3OUT &= ~BIT7;            //set low
   P3DIR |= BIT7;             //set as output
   //FLASH_SOMI
   P5DIR &= ~BIT4;            //set as input
   //FLASH_SCLK_R
   P5OUT &= ~BIT5;            //set low
   P5DIR |= BIT5;             //set as output
   //Flash power (SW_FLASH)
   P4OUT &= ~BIT2;            //set low
   P4DIR |= BIT2;             //set as output
   //SD card detect (SD_DETECT_N)
   P4DIR &= ~BIT1;            //set as input
   //DOCK
   //P2DIR &= ~BIT3;            //set as input
   //DETECT_N
   //P6OUT |= BIT0;             //set high
   //P6DIR |= BIT0;             //set as output

   //EXP_RESET_N
   P3DIR &= ~BIT3;            //set as input


   //SA_SIMO_TXD
   P3OUT &= ~BIT4;            //set low
   P3DIR |= BIT4;             //set as output
   //SA_SOMI_RXD
   P3DIR &= ~BIT5;            //set as input
   //SA_SCLK_R
   P3OUT &= ~BIT0;            //set low
   P3DIR |= BIT0;             //set as output


   //GPIO_INTERNAL
   //P1DIR &= ~BIT4;
   //GPIO_INTERNAL2
   P2DIR &= ~BIT1;
   //GPIO_EXTERNAL_RADIO_DD
   P1DIR &= ~BIT5;


   // ================================
   // ================================
   //uca0 - pc_uart: p3.4 rx, p3.5 rx
   //uca1 - bt_uart: p5.6 tx, p5.7 rx
   //ucb0 - i2c: p3.1 sda, p3.2 scl
   //I2C ports for LSM303DLHC, MPU9150 and BMP180 (USCI_B0)

   //VREFP (P5.0 connected to PV_VREF_MSP) and VREFN (P5.1 connected to GND)
   P5DIR &= ~(BIT0 + BIT1);  //set as input

   P3OUT |= BIT1+BIT2;
   P3DIR |= BIT1+BIT2;
   P3SEL |= BIT1+BIT2;
   //ucb1 - spi: p3.7 simo, p5.4 somi, p5.5 clk

   //Make the 2 CHG_STAT pins input
   P2DIR &= ~(BIT2 + BIT3);
   P2REN |= (BIT2 + BIT3);
   P2REN &= ~(BIT2 + BIT3);

   //SW_I2C todo: change pins
   P7OUT &= ~BIT5;           //set low
   P7DIR |= BIT5;            //set as output
   //SW_SG todo: change pins
   P6OUT &= ~BIT6;           //set low
   P6DIR |= BIT6;            //set as output

   //Not connected
   //P8OUT &= ~BIT5;           //set low
   //P8DIR |= BIT5;            //set as output

   // nc, as flag
   P8OUT &= ~BIT0;           //set low
   P8DIR |= BIT0;            //set as output

   P8OUT |= (BIT6 + BIT5 + BIT4 + BIT3 + BIT2 );
   P8DIR |= BIT6 + BIT5 + BIT4 + BIT3 + BIT2;

   // dock
   P8OUT &= ~BIT1;
   P8DIR &= ~BIT1;
   P8REN |= BIT1;             //enable pull up resistor



   P1DIR &= ~BIT4;            //sw1 in
   P1REN |= BIT4;
   P1OUT |= BIT4;             // default to high

   P2DIR &= ~BIT4;            //sw2
   P2DIR &= ~BIT5;            //vib
   //power_ctrl
   P6OUT |= BIT5;
   P6DIR |= BIT5;

   //todo: Battery voltage ADC pin as input
   //currently bt_status
   P6DIR &= ~BIT2;
   P6OUT &= ~BIT2;
   P6REN |= BIT2;

   //BT RN42 ports
   //BT power
   P7OUT &= ~BIT4;            //set low
   P7DIR |= BIT4;             //set as output
   //UART_RTS and connect indication as input
   P1DIR &= ~BIT0;            //todo: not used yet, wants to be bt_status_int
   P2DIR &= ~BIT7;            //bt_rts, input
   //UART_CTS
   P2OUT &= ~BIT6;            //set low
   P2DIR |= BIT6;             //output
   //BT_Reset
   P6OUT &= ~BIT1;            //set low
   P6DIR |= BIT1;             //output
   //BT_Factory Reset
   P6OUT &= ~BIT0;            //set low
   P6DIR |= BIT0;             //output
   //UART RX and TX
   P5SEL |= BIT6+BIT7;        //P5.6,P5.7 = USCI_A1 TXD/RXD
   P5DIR |= BIT6;
   P5DIR &= ~BIT7;

   //SG_DRDY as input
   P1DIR &= ~BIT7;
   P1IES |= BIT7;
   P1IE &= ~BIT7;
   //EMG_DRDY as input
   P2DIR &= ~BIT0;             //set as output
   P2IES |= BIT0;
   P2IE &= ~BIT0;



}


/***************************************************************************//**
 * @brief  Turn on LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn on
 * @return none
 ******************************************************************************/


void Board_ledOff(uint8_t ledMask) {
   if (ledMask & PSAD_LED_1_R)   P8OUT |= BIT6;
   if (ledMask & PSAD_LED_1_Y)   P8OUT |= BIT5;
   if (ledMask & PSAD_LED_2_B)   P8OUT |= BIT4;
   if (ledMask & PSAD_LED_3_Y)   P8OUT |= BIT3;
   if (ledMask & PSAD_LED_4_G)   P8OUT |= BIT2;
}


/***************************************************************************//**
 * @brief  Turn off LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn off
 * @return none
 ******************************************************************************/
void Board_ledOn(uint8_t ledMask) {
   if (ledMask & PSAD_LED_1_R)   P8OUT &= ~BIT6;
   if (ledMask & PSAD_LED_1_Y)   P8OUT &= ~BIT5;
   if (ledMask & PSAD_LED_2_B)   P8OUT &= ~BIT4;
   if (ledMask & PSAD_LED_3_Y)   P8OUT &= ~BIT3;
   if (ledMask & PSAD_LED_4_G)   P8OUT &= ~BIT2;
}


/***************************************************************************//**
 * @brief  Toggle LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to toggle
 * @return none
 ******************************************************************************/
void Board_ledToggle(uint8_t ledMask) {
   if (ledMask & PSAD_LED_1_R)   P8OUT ^= BIT6;
   if (ledMask & PSAD_LED_1_Y)   P8OUT ^= BIT5;
   if (ledMask & PSAD_LED_2_B)   P8OUT ^= BIT4;
   if (ledMask & PSAD_LED_3_Y)   P8OUT ^= BIT3;
   if (ledMask & PSAD_LED_4_G)   P8OUT ^= BIT2;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
