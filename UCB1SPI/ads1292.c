/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date December, 2013
 */

#include "ads1292.h"
#include "msp430.h"
#include "hal_UCA0.h"

uint8_t ads1292Uca0RxIsr();
uint8_t ads1292Uca0TxIsr();

uint8_t *activeBuffer;
uint8_t chip1Buffer1[9], chip1Buffer2[9], chip2Buffer1[9], chip2Buffer2[9];
uint8_t chip1CurrentFullBuffer, chip2CurrentFullBuffer;
uint8_t rxCount, chip1ReadPending, chip2ReadPending;

void ADS1292_init(void) {
   P7SEL &= ~BIT6;
   P7OUT &= ~BIT6;                  //EXP_RESET_N set low
   P7DIR |= BIT6;                   //EXP_RESET_N set as output

   P2DIR &= ~BIT0;                  //EXG_DRDY as input

   P7SEL &= ~BIT7;
   P7OUT |= BIT7;                   //SA_CS_ECG set high
   P7DIR |= BIT7;                   //SA_CS_ECG as output

   /*activeBuffer = chip1Buffer1;
   chip1ReadPending = 0;
   chip2ReadPending = 0;
   chip1CurrentFullBuffer = 0;
   chip2CurrentFullBuffer = 0;
   rxCount = 0;*/

}

void ADS1292_regRead(uint8_t startaddress, uint8_t size, uint8_t *rdata) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period
                                             //this value was determined experimentally

   //Clock the actual data transfer and send the bytes. Note that we
   //intentionally do not read out the receive buffer during frame transmission
   //in order to optimize transfer speed, however we need to take care of the
   //resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   UCB1TXBUF = startaddress | RREG;
   __delay_cycles(180);                      //Wait 7.5us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //this value was determined experimentally
   while (!(UCB1IFG & UCTXIFG));             //Wait while not ready for TX
   UCB1TXBUF = size-1;
   while ((UCB1STAT & UCBUSY));              //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions
   UCB1IFG &= ~UCRXIFG;                      //Ensure RXIFG is clear

   // Clock the actual data transfer and receive the bytes
   while (size--){
      __delay_cycles(180);                   //Wait 7.5us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
      while (!(UCB1IFG & UCTXIFG));          //Wait while not ready for TX
      UCB1TXBUF = 0xff;                      //Write dummy byte
      while (!(UCB1IFG & UCRXIFG));          //Wait for RX buffer (full)
      *rdata++ = UCB1RXBUF;
   }

   __bis_SR_register(gie);                   //Restore original GIE state
}

void ADS1292_regWrite(uint8_t startaddress, uint8_t size, uint8_t *wdata) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period

   // Clock the actual data transfer and send the bytes. Note that we
   // intentionally do not read out the receive buffer during frame transmission
   // in order to optimize transfer speed, however we need to take care of the
   // resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   UCB1TXBUF = startaddress | WREG;
   __delay_cycles(180);                      //Wait 7.5us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
   while (!(UCB1IFG & UCTXIFG));             //Wait while not ready for TX
   UCB1TXBUF = size-1;
   while(size--) {
      __delay_cycles(180);                   //Wait 7.5us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
      while (!(UCB1IFG & UCTXIFG));          //Wait while not ready for TX
      UCB1TXBUF = *wdata++;
   }

   while ((UCB1STAT & UCBUSY));              //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions
   __bis_SR_register(gie);                   //Restore original GIE state
}


void ADS1292_powerOn(void) {
   P7OUT |= BIT6;
   __delay_cycles(240000);                   //10ms (assuming 24MHz clock)
}

void ADS1292_powerOff(void) {
   P7OUT &= ~BIT6;
}


//Issues a reset pulse
//Remains powered on afterwards
//Delays match the sample code from TI
void ADS1292_resetPulse(void) {
   P7OUT |= BIT6;          //set high
   __delay_cycles(24000);  //1ms (assuming 24MHz clock)
   P7OUT &= ~BIT6;         //set low
   __delay_cycles(24000);  //1ms (assuming 24MHz clock)
   P7OUT |= BIT6;          //set high
   __delay_cycles(168000); //7ms (assuming 24MHz clock)
}

void ADS1292_csEnable(uint8_t enable) {
   if(enable) {
      P7OUT &= ~BIT7;
      //only need to wait 10ns here, so no need for a delay
      //(as less than 1 clock cycle)

   } else {
      __delay_cycles(141);    //wait 5.875us (assuming 24MHz clock)
                              //i.e. 3tCLKs
      P7OUT |= BIT7;
   }
}


void ADS1292_readDataContinuousMode(uint8_t enable) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period

   // Clock the actual data transfer and send the bytes. Note that we
   // intentionally not read out the receive buffer during frame transmission
   // in order to optimize transfer speed, however we need to take care of the
   // resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   if(enable)
      UCB1TXBUF = RDATAC;
   else
      UCB1TXBUF = SDATAC;
   while ( (UCB1STAT & UCBUSY) );            //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions

   __bis_SR_register(gie);                   //Restore original GIE state
}


void ADS1292_start(uint8_t start) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period

   // Clock the actual data transfer and send the bytes. Note that we
   // intentionally not read out the receive buffer during frame transmission
   // in order to optimize transfer speed, however we need to take care of the
   // resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   if(start)
      UCB1TXBUF = START;
   else
      UCB1TXBUF = STOP;
   while ( (UCB1STAT & UCBUSY) );            //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions

   __bis_SR_register(gie);                   //Restore original GIE state
}


void ADS1292_resetRegs(void) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period

   // Clock the actual data transfer and send the bytes. Note that we
   // intentionally not read out the receive buffer during frame transmission
   // in order to optimize transfer speed, however we need to take care of the
   // resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   UCB1TXBUF = RESET;
   while ( (UCB1STAT & UCBUSY) );            //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions

   __bis_SR_register(gie);                   //Restore original GIE state

   //takes 9f_MOD cycles (assume f_MOD is 128kHz as f_CLK is fixed to 512kHz)
   //so 70.3125us
   //must avoid sending other commands during this time
   //Every other function waits 6us before sending a command
   //so wait 65us here
   __delay_cycles(1560);                     //wait 65us (assuming 24MHz MCLK)
}

void ADS1292_offsetCal(void) {
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state

   __disable_interrupt();                    //Make this operation atomic

   __delay_cycles(144);                      //Wait 6us (assuming 24MHz MCLK), required to allow t_sdecode to be 4 T_clk
                                             //needed to ensure previous byte was not sent within this time period

   // Clock the actual data transfer and send the bytes. Note that we
   // intentionally not read out the receive buffer during frame transmission
   // in order to optimize transfer speed, however we need to take care of the
   // resulting overrun condition.
   while (!(UCB1IFG & UCTXIFG)) ;            //Wait while not ready for TX
   UCB1TXBUF = OFFSETCAL;
   while ( (UCB1STAT & UCBUSY) );            //Wait for all TX/RX to finish

   UCB1RXBUF;                                //Dummy read to empty RX buffer
                                             //and clear any overrun conditions

   __bis_SR_register(gie);                   //Restore original GIE state
}


void ADS1292_enableInternalReference(void) {
   uint8_t data[] = {0xA0};

   ADS1292_regWrite(ADS1x9x_REG_CONFIG2, 1, data);

   __delay_cycles(2400000);                  //100ms (assuming 24MHz clock)
                                             //for internal reference to settle
}

void ADS1292_enableDrdyInterrupts() {
   //enable falling edge interrupt on P2.0
   P2IES |= BIT0; //look for falling edge
   P2IFG &= ~BIT0;

   P2IE |= BIT0;  //enable interrupt
}

void ADS1292_disableDrdyInterrupts() {
   P2IE &= ~BIT0; //disable interrupt
}

uint8_t ADS1292_readDataChip1(uint8_t *data) {
   if(chip1CurrentFullBuffer == 1) {
      memcpy(data, chip1Buffer1, 9);
   } else if (chip1CurrentFullBuffer == 2) {
      memcpy(data, chip1Buffer2, 9);
   } else
      return 0;
   return 1;
}
uint8_t ADS1292_readDataChip2(uint8_t *data) {
   if(chip2CurrentFullBuffer == 1) {
      memcpy(data, chip2Buffer1, 9);
   } else if (chip2CurrentFullBuffer == 2) {
      memcpy(data, chip2Buffer2, 9);
   } else
      return 0;
   return 1;
}


unsigned char ADS1292_ReceiveByte(void)
{
   unsigned char Result = 0;

   while(!(UCB1IFG&UCTXIFG)); // Make sure nothing is currently transmitting
   UCB1TXBUF = 0xff;       // Send out NOP to initiate SCLK
   while(!(UCB1IFG&UCRXIFG)); // Wait until all data is transmitted (received)
   Result = UCB1RXBUF;  // Capture the receive buffer

   return Result;
}


//Tell the driver that the data is ready to be read from chipX
void ADS1292_dataReadyChip1() {
   if(chip2ReadPending)
      chip1ReadPending = 1;
   else {
      if(!chip1ReadPending) {
         chip1ReadPending = 1;
         if(chip1CurrentFullBuffer == 1) {
            activeBuffer = chip1Buffer2;
         } else {
            activeBuffer = chip1Buffer1;
         }
         rxCount = UCA0RXBUF;    //Dummy Read
         rxCount = 0;

         if(P6IN&BIT1) {
            //ADS1292_chip1CsEnable(1);
         }

         UCA0TXBUF = 0xFF;
         UCA0IE |= UCRXIE;       //Enable USCI_A0 RX interrupt
      }
   }
}

void ADS1292_dataReadyChip2() {
   if(chip1ReadPending)
      chip2ReadPending = 1;
   else {
      if(!chip2ReadPending) {
         chip2ReadPending = 1;
         if(chip2CurrentFullBuffer == 1) {
            activeBuffer = chip2Buffer2;
         } else {
            activeBuffer = chip2Buffer1;
         }
         rxCount = UCA0RXBUF;    //Dummy Read
         rxCount = 0;

         if(P7IN&BIT6) {
            //ADS1292_chip2CsEnable(1);
         }

         UCA0TXBUF = 0xFF;
         UCA0IE |= UCRXIE;       //Enable USCI_A0 RX interrupt
      }
   }
}


uint8_t ads1292Uca0RxIsr(){
   while (!(UCA0IFG&UCTXIFG));      //USCI_A0 TX buffer ready?

   activeBuffer[rxCount++] = UCA0RXBUF;
   if ( rxCount == ADS1292_DATA_PACKET_LENGTH) {
      UCA0IE &= ~UCRXIE;            //Disable USCI_A0 RX interrupt
      if((activeBuffer == chip1Buffer1) || (activeBuffer == chip1Buffer2)) {
         chip1ReadPending = 0;
         if(activeBuffer == chip1Buffer1)
            chip1CurrentFullBuffer = 1;
         else
            chip1CurrentFullBuffer = 2;
         if(chip2ReadPending) {
            chip2ReadPending = 0;
            ADS1292_dataReadyChip2();
         }
      } else {
         chip2ReadPending = 0;
         if(activeBuffer == chip2Buffer1)
            chip2CurrentFullBuffer = 1;
         else
            chip2CurrentFullBuffer = 2;
         if(chip1ReadPending) {
            chip1ReadPending = 0;
            ADS1292_dataReadyChip1();
         }
      }
   } else {
      UCA0TXBUF = 0xFF;             //To get Next byte.
   }
   return 0;
}

uint8_t ads1292Uca0TxIsr() {return 0;}  // to add if necessary

