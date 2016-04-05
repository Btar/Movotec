/*
 * UCB1SPI.c
 *
 *  Created on: Oct 21, 2015
 *      Author: WeiboP
 */


#include "UCB1SPI.h"
#include "ADS1220.h"
#include "ads1292.h"
#include "msp430.h"

#define UCB1SPI_SG   0x01
#define UCB1SPI_EMG  0x02

uint8_t ucb1spi_busy, ucb1spi_sgDataCnt;
uint8_t sgCurrentFullBuffer, sgBuffer1[12], sgBuffer2[12];
uint8_t emgCurrentFullBuffer, emgBuffer1[9], emgBuffer2[9];
uint8_t *ucb1ActiveBuffer, ucb1RxCount, ucb1RxExpectedLen, sgReadPending, emgReadPending;
uint8_t sgKeepSensing, currentSg0Emg1;


void UCB1SPI_Init(){
   ucb1spi_busy = 0;

   P3SEL |= BIT7;                   //Set SPI peripheral bits
   P3DIR &= ~BIT7;                  //Din as input
   P5SEL |= BIT4+BIT5;
   P5DIR |= BIT4+BIT5;              //Clock and DOUT as output

   UCB1CTL1 |= UCSWRST;                // Hold peripheral in reset
   UCB1CTL0 = UCMST + UCSYNC + UCMSB;        // SPI master, synchronous
   UCB1CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK for bit rate clock and keep in reset
   UCB1BR0 = 4;                       // SMCLK/2 = SCLK (12MHz)
   UCB1BR1 = 0;
   UCB1CTL1 &= ~UCSWRST;                  // Release peripheral for use


   ucb1ActiveBuffer = emgBuffer1;
   sgReadPending = 0;
   emgReadPending = 0;
   sgCurrentFullBuffer = 0;
   emgCurrentFullBuffer = 0;
   ucb1RxCount = 0;
   sgKeepSensing = 0;
}


//============================== below is strain gauge ==============================
void UCB1SPI_SG_init(uint8_t gain) {
   while(ucb1spi_busy);

   ucb1spi_busy = 1;
   ADS1220Init();
   ADS1220Config(gain);
   //ADS1220SendStartCommand();
   ucb1spi_busy = 0;
}

/*void ucb1spi_SG_SetChannel(int Mux){
   //while(ucb1spi_busy);
   //ucb1spi_busy = 2;
   ADS1220SetChannel(ADS1220_MUX_0_G+(Mux<<4));
   //ucb1spi_busy = 0;
}*/

void UCB1SPI_SG_SendStartCommand(void){
   while(ucb1spi_busy);
   ucb1spi_busy = 3;

   sgKeepSensing = 1;
   ucb1spi_sgDataCnt = 0;
   ADS1220SendStartCommand();

   ucb1spi_busy = 0;
}

void UCB1SPI_SG_SendShutdownCommand(void){
   sgKeepSensing = 0;
   while(ucb1spi_busy);
   ucb1spi_busy = 4;
   ADS1220SendShutdownCommand();
   ucb1spi_busy = 0;

}

void UCB1SPI_SG_SetGain(uint8_t gain){
   while(ucb1spi_busy);
   ucb1spi_busy = 4;
   ADS1220SetGain(gain);
   ucb1spi_busy = 0;
}



void UCB1SPI_SG_ReadData(uint8_t* buf){
   // data format:
   // sg1_xmsb, sg1_msb, sg1_lsb, sg2_xmsb, ..., sg4_lsb
   uint8_t *temp_buf;
   temp_buf = sgCurrentFullBuffer==1? sgBuffer1:sgBuffer2;

   memcpy(buf,temp_buf, 12);
}


void UCB1SPI_SG_dataReady() {//chip1
   if(emgReadPending)
      sgReadPending = 1;
   else {
      if(!sgReadPending) {
         sgReadPending = 1;
         if(sgCurrentFullBuffer == 1) {
            ucb1ActiveBuffer = sgBuffer2+(ucb1spi_sgDataCnt*3);
         } else {
            ucb1ActiveBuffer = sgBuffer1+(ucb1spi_sgDataCnt*3);
         }
         currentSg0Emg1 = 0;
         ucb1RxCount = UCB1RXBUF;    //Dummy Read
         ucb1RxCount = 0;
         ucb1RxExpectedLen = 3;

         if(P6IN&BIT3) {
            ADS1220AssertCS(1);
         }

         UCB1TXBUF = 0xff;//0xff
         UCB1IE |= UCRXIE;       //Enable USCI_A0 RX interrupt
      }
   }
}

void UCB1SPI_SG_dataRx_done(){
   uint8_t sg_next_channel;
   if(sgKeepSensing){
      if(++ucb1spi_sgDataCnt>=4){
         ucb1spi_sgDataCnt = 0;
      }
      if(ucb1spi_sgDataCnt==0)
         sg_next_channel = 8;
      else if(ucb1spi_sgDataCnt==1)
         sg_next_channel = 9;
      else if(ucb1spi_sgDataCnt==2)
         sg_next_channel = 10;
      else if(ucb1spi_sgDataCnt==3)
         sg_next_channel = 11;
      ADS1220SetChannel(sg_next_channel);
      ADS1220SendStartCommand();
   }
}





//============================== above is strain gauge ==============================

//============================== below is emg ==============================
void UCB1SPI_EMG_init(void) {
   //ADS1292_powerOn();             //not required as handled in init function

   while(ucb1spi_busy);
   ucb1spi_busy = 6;

   ADS1292_init();
   ADS1292_resetPulse();

   ADS1292_csEnable(1);
   ADS1292_readDataContinuousMode(0);
   ADS1292_csEnable(0);
   ucb1spi_busy = 0;
}

void UCB1SPI_EMG_start(void) {
   while(ucb1spi_busy);
   ucb1spi_busy = 7;

   ADS1292_csEnable(1);
   //ucb1spi_rdata = 0;
   //ADS1292_regRead(0, 1, &ucb1spi_rdata);


   ADS1292_readDataContinuousMode(1);

   //ADS1292_enableDrdyInterrupts();
   P2IES |= BIT0; //look for falling edge
   P2IFG &= ~BIT0;
   ADS1292_disableDrdyInterrupts();

   ADS1292_start(1);
   ADS1292_csEnable(0);
   ucb1spi_busy = 0;
}

void UCB1SPI_EMG_stop(void) {
   while(ucb1spi_busy);
   ucb1spi_busy = 8;

   ADS1292_disableDrdyInterrupts();

   ADS1292_csEnable(1);
   ADS1292_start(0);
   ADS1292_readDataContinuousMode(0);
   ADS1292_csEnable(0);

   ADS1292_powerOff();

   ucb1spi_busy = 0;
}

void UCB1SPI_EMG_writeRegs(uint8_t startaddress, uint8_t size, uint8_t *wdata) {
   while(ucb1spi_busy);
   ucb1spi_busy = 9;

   ADS1292_csEnable(1);
   ADS1292_regWrite(startaddress, size, wdata);
   ADS1292_csEnable(0);

   ucb1spi_busy = 0;
}

void UCB1SPI_EMG_offsetCal(void) {
   while(ucb1spi_busy);
   ucb1spi_busy = 10;
   ADS1292_csEnable(1);

   ADS1292_offsetCal();

   ADS1292_csEnable(0);
   ucb1spi_busy = 0;
}

void UCB1SPI_EMG_readData(uint8_t* buf){
   uint8_t* temp_buf;
   temp_buf = emgCurrentFullBuffer==1? emgBuffer1:emgBuffer2;

   buf[0] = ((*temp_buf & 0x4F)<<1) + ((*(temp_buf+1) & 0x80)>>7);
   buf[1] = temp_buf[5];
   buf[2] = temp_buf[4];
   buf[3] = temp_buf[3];
   buf[4] = temp_buf[8];
   buf[5] = temp_buf[7];
   buf[6] = temp_buf[6];


   uint8_t emgLoffBit1, emgLoffBit2;

   emgLoffBit1 = (buf[0] & 0b00010011)? 1:0;
   emgLoffBit2 = (buf[0] & 0b00001100)? 1:0;
   buf[1] = (buf[1] & 0xfe) + emgLoffBit1;
   buf[4] = (buf[4] & 0xfe) + emgLoffBit2;
}

void UCB1SPI_EMG_dataReady(){//chip1
   uint16_t gie = __get_SR_register() & GIE; //Store current GIE state
   __disable_interrupt();                    //Make this operation atomic
   if(sgReadPending)
      emgReadPending = 1;
   else {
      if(!emgReadPending) {
         emgReadPending = 1;
         if(emgCurrentFullBuffer == 1) {
            ucb1ActiveBuffer = emgBuffer2;
         } else {
            ucb1ActiveBuffer = emgBuffer1;
         }
         currentSg0Emg1 = 1;
         ucb1RxCount = UCB1RXBUF;    //Dummy Read
         ucb1RxCount = 0;
         ucb1RxExpectedLen = 9;

         if(P7IN&BIT7) {
            ADS1292_csEnable(1);
         }

         UCB1TXBUF = 0xFF;
         UCB1IE |= UCRXIE;       //Enable USCI_A0 RX interrupt
      }
   }
   __bis_SR_register(gie);                   //Restore original GIE state
}


//============================== above is emg ==============================




uint8_t ucb1RxIsr(){
   while (!(UCB1IFG&UCTXIFG));      //USCI_A0 TX buffer ready?

   ucb1ActiveBuffer[ucb1RxCount++] = UCB1RXBUF;
   if ( ucb1RxCount == ucb1RxExpectedLen) {
      UCB1IE &= ~UCRXIE;            //Disable USCI_A0 RX interrupt
      if(!currentSg0Emg1) {
         sgReadPending = 0;
         if(ucb1spi_sgDataCnt == 3){
            if (ucb1ActiveBuffer == (sgBuffer1+9))//sgBuffer2+(ucb1spi_sgDataCnt*3)
               sgCurrentFullBuffer = 1;
            else
               sgCurrentFullBuffer = 2;
         }
         UCB1SPI_SG_dataRx_done();
         if(emgReadPending) {
            emgReadPending = 0;
            UCB1SPI_EMG_dataReady();
         }
      } else {
         emgReadPending = 0;
         if(ucb1ActiveBuffer == emgBuffer1)
            emgCurrentFullBuffer = 1;
         else
            emgCurrentFullBuffer = 2;
         ADS1292_csEnable(0);
         if(sgReadPending) {
            sgReadPending = 0;
            UCB1SPI_SG_dataReady();
         }
      }
   } else {
      UCB1TXBUF = 0xFF;             //To get Next byte.
   }
   return 0;
}

uint8_t ucb1TxIsr() {return 0;}  // to add if necessary



