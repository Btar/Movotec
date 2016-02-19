/*
 * Copyright (c) 2015, Shimmer Research, Ltd.
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
 * @author Weibo Pan
 * @date January, 2016
 */

/***********************************************************************************

   Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... | chanX
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | chanX

   Inquiry Response Packet Format:
          Packet Type | ADC Sampling rate | Config Bytes | Num Chans | Buf size | Chan1 | Chan2 | ... | ChanX
   Byte:       0      |        1-2        |      3-6     |     7     |     8    |   9   |   10  | ... |   x

***********************************************************************************/

//TODO: figure out why first sample after starting streaming is sometimes incorrect when using DMA with the ADC

#include <stdint.h>
#include <stdlib.h>
#include "msp430.h"
#include "shimmer.h"
#include "hal_pmm.h"
#include "hal_UCS.h"
#include "hal_Board.h"
#include "hal_Button.h"
#include "hal_I2C.h"
#include "hal_ADC.h"
#include "hal_RTC.h"
#include "hal_DMA.h"
#include "hal_InfoMem.h"
#include "hal_TB0.h"
#include "RN42.h"
//#include "lsm303dtr.h"
#include "string.h"
#include "mpu9150.h"
//#include "exg.h"
//#include "ADS1220.h"
#include "UCB1SPI.h"
#include "cat24c16.h"
#include "hal_UCA0.h"
//#include "hal_UartA0.h"
#include "hal_CRC.h"

void Init(void);
void BlinkTimerStart(void);
inline void BlinkTimerStop(void);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
uint8_t BtDataAvailable(uint8_t data);
//uint8_t Dma0ConversionDone(void);
void ProcessCommand(void);
void SendResponse(void);
//void ConfigureChannels();
inline void StartStreaming(void);
inline void StopStreaming(void);
//void SetDefaultConfiguration(void);
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
inline void SetLed(void);
inline void ClearLed(void);
inline uint8_t IsLedSet(void);
inline void MonitorChargeStatus(void);
inline void MonitorChargeStatusStop(void);
inline void SetChargeStatusLeds(void);

//void SgReadData();
void ReadBatt();
void ConfigVBatt(void);


//data segment initialisation is disabled in system_pre_init.c
uint8_t currentBuffer, processCommand, sendAck, startSensing, stopSensing, sensing, btIsConnected,btIsPowerOn,
      readBatt,streamData, sendResponse, emgConfigRsp, configuring, syncRsp, statusRsp, sgShiftRsp,
      infomemResponse, fwVersionRsp, configAllRsp, dataEmgRsp, dataAccelRsp, dataGyroRsp, dataSgRsp,
      accelRangeRsp, gyroRangeRsp, autoOffEnRsp, standbyEnRsp, sgGainRsp, sgRegRsp, led3UserCtrl, a1CfgRsp;
uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, argsSize;
uint8_t resPacket[RESPONSE_PACKET_SIZE];
uint8_t psadConfig[NV_CONFIG_LEN];
uint8_t nbrAdcChans, nbrDigiChans, infomemLength;
uint16_t *adcStartPtr, infomemOffset;
uint8_t preSampleMpuMag, mpuMagFreq, mpuMagCount, mpu9150Initialised;
uint8_t preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp, sampleBmpTempFreq;
//+ 1 byte (at start) as can only read/write 16-bit values at even addresses
//+ 1 byte to allow ACK at end of last data packet when stop streaming command is received
uint8_t txBuff0[DATA_PACKET_SIZE+2], txBuff1[DATA_PACKET_SIZE+2];
uint8_t docked, selectedLed;
uint64_t sw2Press, sw2Release, sw2LastRelease, currentTs, discTs;

uint8_t btMac[12], fwInfo[7], battVal[3];
#define BATT_HIGH       0
//#define BATT_MID        1
#define BATT_LOW        2

char *dierecord;

uint8_t ledCounter, battStat;
uint8_t btMacHex[6];

void PSAD_setDefaultConfig(void);
void ParseConfig();
uint8_t dataLen, cfgEmgCnt, cfgAccelCnt, cfgGyroCnt, cfgStrainCnt, cfgAccelRange, cfgAccel303Range, cfgGyroRange,
        cfgAutoStandby, cfgAutoTurnoff, cfgAccelEn, cfgGyroEn, cfgEmgEn, cfgStrainEn, cfgSgToShift, cfgStrainGain;

//uint8_t uartWaitingForArgs, uartArgs[MAX_COMMAND_ARG_SIZE], uartArgsSize, uartSendAck, uartSendResponse;
uint8_t dataBuffEmg[7], dataBuffAccel[12], dataBuffAccelTemp[6], dataBuffGyro[6], dataBuffGyroZTemp[2],
        dataBuffStrain[8], sw1Status, sw2Status, vibStatus, sw1New, sw2New, vibNew, sw1Cnt, sw2Cnt, vibCnt;// vbatt: val_l + val_h + stat
//long dataBuffStrainRaw[4];
uint8_t dataBuffStrainRaw[12];
uint8_t ptrEMG, ptrAccel1X, ptrGyroX, ptrStrain, ptrStatus, ptrIdStamp;//ptrAccel2X,
uint8_t dataLen, statusReg, cpuIsSleeping, enterStandby, exitStandby;//, sgGoOn
uint16_t dataBuffTimer, dataBuffCurrent;
uint8_t sgReadDataCnt;

uint8_t* i2cBicOnExit;
uint8_t DMA0BicOnExit;

#define BATT_NO_POWER      0
#define BATT_CHARGING      1
#define BATT_CHARGE_DONE   2
uint8_t chargStatus;

void EnterStandby(void);
void ExitStandby(void);
uint8_t DataPtrCalc(void);
inline void GetEmgFromSensor(uint8_t * dbuff);
inline void GetAccel1FromSensor(uint8_t * dbuff);
inline void GetAccel2FromSensor(uint8_t * dbuff);
inline void GetGyroFromSensor(uint8_t * dbuff);
inline void GetStrainFromSensor(uint8_t * dbuff, uint8_t len);

void GetEmg(uint8_t * ptr);
void GetAccel(uint8_t * ptr);
void GetGyro(uint8_t * ptr);
void GetStrain(uint8_t * ptr);
void SaveSpiOff();
void LoadSpiOn();

uint8_t test_steps;

void main(void) {
   //uint8_t digi_offset;

   Init();

   dierecord = (char *)0x01A0A;

   uint8_t allff[NV_CONFIG_LEN];
   memset(allff,0xff,NV_CONFIG_LEN);
   InfoMem_read((uint8_t *)0, psadConfig, NV_CONFIG_LEN);
   //if(psadConfig[NV_CONFIG_0] == 0xFF){
   if(!memcmp(psadConfig, allff, NV_CONFIG_LEN)){
      PSAD_setDefaultConfig();
   }
   //PSAD_setDefaultConfig(); // todo: remove this line

   BT_setTempBaudRate("921K");

   //ConfigureChannels();

   txBuff0[1] = txBuff1[1] = BT_DATA_PACKET; //packet type

   BlinkTimerStart();   //blink the blue LED
                        //also starts the charge status timer if not docked
   //if(docked)
   MonitorChargeStatus();

   ConfigVBatt();
   //startSensing = 1;

   //configuring = 1;
   while(1) {
      test_steps = 0;
      cpuIsSleeping = 1;
      if(!(readBatt || processCommand || sendResponse || streamData || exitStandby || enterStandby))
         __bis_SR_register(LPM3_bits + GIE); //ACLK remains active
      cpuIsSleeping = 0;

      if(exitStandby) {
         test_steps = 2;
         exitStandby = 0;
         ExitStandby();
      }
      if(readBatt) {
         test_steps = 3;
         readBatt = 0;
         ReadBatt();
      }
      if(processCommand) {
         test_steps = 4;
         processCommand = 0;
         ProcessCommand();
      }
      if(sendResponse) {
         test_steps = 5;
         sendResponse = 0;
         SendResponse();
      }
      if(startSensing) {
         test_steps = 6;
         configuring = 1;
         if(!stopSensing) {
            startSensing = 0;
            ParseConfig();
            //dataLen = DataPtrCalc();
            StartStreaming();

            // get the 1st data sample so txing partial data is possible
            /*GetEmgFromSensor(dataBuffEmg);
            GetAccel1FromSensor(dataBuffAccel);
            GetAccel2FromSensor(dataBuffAccel+6);
            GetGyroFromSensor(dataBuffGyro);
            GetStrainFromSensor(dataBuffStrain);*/
         }
         configuring = 0;
      }
      if(streamData) {
         streamData = 0;
         dataBuffCurrent = dataBuffTimer;
         test_steps = 7;
         //P8OUT |= BIT0;

         uint8_t * data_ptr;
         if(currentBuffer){
            data_ptr = txBuff1;
            currentBuffer = 0;
         }else{
            data_ptr = txBuff0;
            currentBuffer = 1;
         }

         data_ptr[DATA_ID_STAMP] = dataBuffCurrent & 0xff;
         data_ptr[DATA_ID_STAMP+1] = ((dataBuffCurrent>>8) & 0x3f) + (sw1Status<<7) + (sw2Status<<6);

         test_steps = 9;
         if(cfgEmgEn)
            GetEmg(data_ptr+ptrEMG);
         test_steps = 10;
         if(cfgAccelEn)
            GetAccel(data_ptr+ptrAccel1X);
         test_steps = 11;
         if(cfgGyroEn)
            GetGyro(data_ptr+ptrGyroX);
         test_steps = 12;
         if(cfgStrainEn)
            GetStrain(data_ptr+ptrStrain);
         test_steps = 13;

         if(btIsConnected)
            //BT_append(data_ptr+DATA_PACKET, dataLen);
            BT_write(data_ptr+DATA_PACKET, dataLen);
         //P8OUT &= ~BIT0;
      }
      if(stopSensing) {
         test_steps = 8;
         configuring = 1;
         StopStreaming();
         stopSensing = 0;
         configuring = 0;
      }
      if(enterStandby) {
         test_steps = 1;
         enterStandby = 0;
         EnterStandby();
      }
   }
}
void Init(void) {
   // Stop WDT
   //WDTCTL = WDTPW + WDTHOLD; // already handled in system_pre_init.c

   Board_init();
   Board_ledOn(LED_ALL);

   // Set Vcore to accommodate for max. allowed system speed
   SetVCore(3);

   // Start 32.768kHz XTAL as ACLK
   LFXT_Start(XT1DRIVE_0);

   // Start 24MHz XTAL as MCLK and SMCLK
   XT2_Start(XT2DRIVE_2);        // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
   UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

   SFRIFG1 = 0;                  // clear interrupt flag register
   SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

   RTC_init(0);
   currentTs = discTs = 0;
   sw2Press = 0;
   sw2LastRelease = 0;

   //sgGoOn = 0;
   vibStatus = 0;
   sw1New = sw2New = vibNew = 0;
   sw1Cnt = sw2Cnt = vibCnt = 0;
   led3UserCtrl = 0;
   sgReadDataCnt = 0;
   sendResponse = 0;
   streamData = 0;
   configuring = 0;
   chargStatus = 0;
   enterStandby = 0;
   exitStandby = 1;
   cpuIsSleeping = 0;
   readBatt = 1;
   sensing = 0;
   currentBuffer = 0;
   processCommand = 0;
   sendAck = 0;
   startSensing = 0;
   stopSensing = 0;
   //configureChannels = 0;
   waitingForArgs = 0;
   argsSize = 0;
   nbrAdcChans = 0;
   nbrDigiChans = 0;
   preSampleMpuMag = 0;
   mpu9150Initialised = 0;
   preSampleBmpPress = 0;
   sampleBmpTemp = 0;
   docked = 0;
   selectedLed = 0;
   //uartProcessCmds = 0;
   //uartSendResponses = 0;
   fwVersionRsp = 0;
   //vbattRsp = 0;
   emgConfigRsp = 0;
   configAllRsp = 0;
   dataEmgRsp = 0;
   dataAccelRsp = 0;
   dataGyroRsp = 0;
   dataSgRsp = 0;
   accelRangeRsp = 0;
   gyroRangeRsp = 0;
   autoOffEnRsp = 0;
   standbyEnRsp = 0;
   infomemResponse = 0;
   sgShiftRsp = 0;
   sgGainRsp = 0;
   sgRegRsp = 0;
   a1CfgRsp = 0;
   cfgSgToShift = 0;
   syncRsp = 0;
   statusRsp = 0;

   battStat = BATT_HIGH;
   btIsConnected = 0;
   btIsPowerOn = 0;
   dataBuffTimer = 0;
   dataBuffCurrent = 0;


   //Globally enable interrupts
   _enable_interrupts();

   *i2cBicOnExit = 0;
   DMA0BicOnExit = 0;

   I2C_set_bic_on_exit(&i2cBicOnExit);
   BT_set_i2c_bic_on_exit(i2cBicOnExit);
   BT_set_DMA0_bic_on_exit(&DMA0BicOnExit);
   BT_setStreamDataStatus(&streamData);
   BT_setCPUSleepStatus(&cpuIsSleeping);

   //if((*BT_streamData && *BT_cpuIsSleeping) || *BT_i2c_bic_on_exit || *BT_DMA0_bic_on_exit)
//   uint8_t bt_started = 0;
   while(!btIsPowerOn){
	   BT_init();
	   BT_disableRemoteConfig(1);
	   BT_setRadioMode(SLAVE_MODE);
	   if(P1IN & BIT0){//if connected, turn off and back on again
		  btIsConnected = 0;               //set connect status to false
		  BT_connectionInterrupt(0);
		  BT_disable();                    //set bt disable, stop starting progress
		  __delay_cycles(360000);
		  continue;
	   }
	   BT_configure();
	   BT_receiveFunction(&BtDataAvailable);
	   BT_getMacAddress(btMac);
	   btIsPowerOn = 1;
   }

   // sw1
   if(P1IN & BIT4) {
      P1IES |= BIT4;    //look for falling edge
      sw1Status = 0;
   } else {
      P1IES &= ~BIT4;   //look for rising edge
      sw1Status = 1;
   }
   P1IFG &= ~BIT4;      //clear flag
   P1IE |= BIT4;        //enable interrupt

   //sw2
   if(P2IN & BIT4) {
      P2IES |= BIT4;    //look for falling edge
      sw2Status = 0;
   } else {
      P2IES &= ~BIT4;   //look for rising edge
      sw2Status = 1;
   }
   P2IFG &= ~BIT4;      //clear flag
   P2IE |= BIT4;        //enable interrupt

   // vibration
   if(P2IN & BIT5) {
      P2IES |= BIT5;    //look for falling edge
      vibStatus = 0;
   } else {
      P2IES &= ~BIT5;   //look for rising edge
      vibStatus = 1;
   }
   P2IFG &= ~BIT5;      //clear flag
   P2IE |= BIT5;        //enable interrupt

   // enable switch1 interrupt
   //Button_init();
   //Button_interruptEnable();

   //EXP_RESET_N
   //P3OUT &= ~BIT3;      //set low
   //P3DIR |= BIT3;       //set as output

   StartTB0();


   Board_ledOff(LED_ALL);
}


inline void StartStreaming(void) {
   //uint8_t offset;
   if(!sensing) {

      dataBuffTimer = 0;

      if(cfgGyroEn || cfgAccelEn){
         MPU9150_init(0);
         MPU9150_wake(1, 0);
         MPU9150_setSamplingRate(7, 0);
         if(cfgGyroEn)
            MPU9150_setGyroSensitivity(cfgGyroRange, 0);
         if(cfgAccelEn){
            MPU9150_setAccelRange(cfgAccelRange, 0);
            MPU9150_init(1);
            MPU9150_wake(1, 1);
            MPU9150_setSamplingRate(7, 1);
            MPU9150_setAccelRange(cfgAccelRange, 1);
         }
      }

//      if(cfgAccelEn)
//         LSM303DTR_accelInit(cfgAccel303Range, psadConfig+NV_A1_CFG);// range 10: data rate 1600Hz, selected range: default 0, zyx=7: all on.

      if(cfgEmgEn || cfgStrainEn)
         UCB1SPI_Init();
      //ExG
      if(cfgEmgEn) {
         UCB1SPI_EMG_init();
         //EXG_writeRegs(0, ADS1292R_CONFIG1, 10, (psadConfig+NV_EMG_1_CONFIG1));
         UCB1SPI_EMG_writeRegs(ADS1292R_CONFIG1, 10, (psadConfig+NV_EMG_1_CONFIG1));
         __delay_cycles(2400000);   //100ms (assuming 24MHz clock)
         if(psadConfig[NV_EMG_1_RESP2] & BIT7)
            UCB1SPI_EMG_offsetCal();
         UCB1SPI_EMG_start();
      }
      if(cfgStrainEn) {
         //sgGoOn = 1;
         UCB1SPI_SG_init(psadConfig+NV_SG_REG0);
         //cfgStrainGain
         //UCB1SPI_SG_SetGain(cfgStrainGain);
         UCB1SPI_SG_SendStartCommand();
      }

      if(cfgEmgEn){
         P2IE |= BIT0;  //enable interrupt
      }
      if(cfgStrainEn) {
         P1IE |= BIT7;
      }


      SampleTimerStart();
      BlinkTimerStart();      //Needs to go after SampleTimerStart(), as TBR is reset in SampleTimerStart()
      sensing = 1;
   }
}

uint8_t p17ie, p20ie, ucb1Rxie;
void SaveSpiOff(){
   //p20ie = P2IE & BIT0;
   //p17ie = P1IE & BIT7;
   //ucb1Rxie = UCB1IE & UCRXIE;

   TB0CCTL4 &= ~CCIE;
   //P2IE &= ~BIT0;
   //P1IE &= ~BIT7;
   //UCB1IE &= ~UCRXIE;
}
void LoadSpiOn(){
   TB0CCTL4 |= CCIE;
   //P2IE |= p20ie;  //enable interrupt
   //P1IE |= p17ie;
   //UCB1IE |= ucb1Rxie;
}

inline void StopStreaming(void) {
   if(sensing) {
      sensing = 0;
      SampleTimerStop();
      //ADC_disable();
      //DMA0_disable();

      if(cfgAccelEn || cfgGyroEn){
//         MPU9150_wake(0);
         MPU9150_wake(0, 0);
         MPU9150_wake(0, 1);
      }
//      if(cfgAccelEn)
//         LSM303DTR_sleep();

      if(cfgEmgEn) {
         //EXG_stop(0);         //probably not needed
         //EXG_powerOff();
         UCB1SPI_EMG_stop();
         P2IE &= ~BIT0;
      }

      if(cfgStrainEn) {
         UCB1SPI_SG_SendShutdownCommand();
         //sgGoOn = 0;
         P1IE &= ~BIT7;
         P6OUT &= ~BIT6;       // SG_SW : P6.6
      }

      __delay_cycles(240000); //10ms (assuming 24MHz clock)
                              //give plenty of time for I2C operations to finish before disabling I2C
      //I2C_Disable();
      //P7OUT &= ~BIT5;         //set SW_I2C low to power off I2C chips

      //if(btIsConnected)
      //   BlinkTimerStop();
      //preSampleMpuMag = 0;
      //preSampleBmpPress = 0;
   }
}


void GetEmgFromSensor(uint8_t * dbuff){
   UCB1SPI_EMG_readData(dbuff);
}
void GetAccel1FromSensor(uint8_t * dbuff){
   uint8_t temp[6];
   uint8_t data_accel_cnt;
   data_accel_cnt = dataBuffCurrent & 0x03;
   SaveSpiOff();
   if(cfgAccelCnt == 2)
      MPU9150_getAccelAxis(temp+4, MPU9150_Z, 1);
   else if(cfgAccelCnt == 1){
      MPU9150_getAccelAxis(temp, MPU9150_X, 1);
      if(data_accel_cnt == 0)
         MPU9150_getAccelAxis(temp+2, MPU9150_Y, 1);
      else if(data_accel_cnt == 1)
         MPU9150_getAccelAxis(temp+4, MPU9150_Z, 1);
   }
   else
      MPU9150_getAccel(temp, 1);
   dbuff[0] = temp[1];
   dbuff[1] = temp[0];
   dbuff[2] = temp[3];
   dbuff[3] = temp[2];
   dbuff[4] = temp[5];
   dbuff[5] = temp[4];
   LoadSpiOn();
}

void GetAccel2FromSensor(uint8_t * dbuff){
   uint8_t temp[6];
   uint8_t data_accel_cnt;
   data_accel_cnt = dataBuffCurrent & 0x03;
   SaveSpiOff();
   if(cfgAccelCnt == 2)
      MPU9150_getAccelAxis(temp+4, MPU9150_Z, 0);
   else if(cfgAccelCnt == 1){
      MPU9150_getAccelAxis(temp+4, MPU9150_Z, 0);
      if(data_accel_cnt == 2)
         MPU9150_getAccelAxis(temp, MPU9150_X, 0);
      else if(data_accel_cnt == 3)
         MPU9150_getAccelAxis(temp+2, MPU9150_Y, 0);
   }
   else
      MPU9150_getAccel(temp, 0);
   dbuff[0] = temp[1];
   dbuff[1] = temp[0];
   dbuff[2] = temp[3];
   dbuff[3] = temp[2];
   dbuff[4] = temp[5];
   dbuff[5] = temp[4];
   LoadSpiOn();

}
void GetGyroFromSensor(uint8_t * dbuff){
   uint8_t data_gyro_cnt;
   uint8_t temp[6];
   SaveSpiOff();
   if(cfgGyroCnt == 2)
      MPU9150_getGyroAxis(temp+2, MPU9150_Y, 0);
   else if(cfgGyroCnt == 1){
      data_gyro_cnt = dataBuffCurrent & 0x01;
      MPU9150_getGyroAxis(temp, MPU9150_X, 0);
      if(!data_gyro_cnt){
         MPU9150_getGyroAxis(temp+2, MPU9150_Y, 0);
      }else{
         MPU9150_getGyroAxis(temp+4, MPU9150_Z, 0);
      }
   }
   else
      MPU9150_getGyro(temp, 0);
   dbuff[0] = temp[1];
   dbuff[1] = temp[0];
   dbuff[2] = temp[3];
   dbuff[3] = temp[2];
   dbuff[4] = temp[5];
   dbuff[5] = temp[4];
   LoadSpiOn();
}

void GetStrainFromSensor(uint8_t * dbuff, uint8_t len){
   //uint8_t i;
   //memcpy(dbuff, dataBuffStrain, 8);
   UCB1SPI_SG_ReadData(dataBuffStrainRaw);

   uint8_t i;
   long temp;
   for(i=0;i<4;i++){
      if(len == 2){
         if(cfgSgToShift == 8){
            dbuff[i*2+0] = dataBuffStrainRaw[i*3+1];        //lsb
            dbuff[i*2+1] = dataBuffStrainRaw[i*3];          //msb
         }else{ // cfgSgToShift
            temp = dataBuffStrainRaw[i*3];
            temp = temp<<8 + dataBuffStrainRaw[i*3+1];
            temp = temp<<8 + dataBuffStrainRaw[i*3+2];

            dbuff[i*2+0] = (temp>>cfgSgToShift) & 0xff;        //lsb
            dbuff[i*2+1] = (dataBuffStrainRaw[i*3] & 0x80) + ((temp>>cfgSgToShift+8) & 0x7f);//sign+msb
         }
      }
      if(len == 3){
         dbuff[i*3+0] = dataBuffStrainRaw[i*3+2];        //lsb
         dbuff[i*3+1] = dataBuffStrainRaw[i*3+1];          //msb
         dbuff[i*3+2] = dataBuffStrainRaw[i*3];          //xmsb
      }
   }
}

void PSAD_setDefaultConfig(void){
   psadConfig[NV_CONFIG_0] = 0x10; // sg at half speed
   psadConfig[NV_CONFIG_1] = 0x27; // AUTO_TURNOFF_EN + AUTO_STANDBY_EN + ACCEL_4g + GYRO_1000dps
   psadConfig[NV_CONFIG_2] = 0x41; // 8bits shift, gain=2

   // default emg setting
   /*psadConfig[NV_EMG_1_CONFIG1] = 0x04;
   psadConfig[NV_EMG_1_CONFIG2] = 0xa0;
   psadConfig[NV_EMG_1_LOFF] = 0x10;
   psadConfig[NV_EMG_1_CH1SET] = 0x60;
   psadConfig[NV_EMG_1_CH2SET] = 0x60;
   psadConfig[NV_EMG_1_RLD_SENS] = 0x20;
   psadConfig[NV_EMG_1_LOFF_SENS] = 0x00;
   psadConfig[NV_EMG_1_LOFF_STAT] = 0x00;
   psadConfig[NV_EMG_1_RESP1] = 0x02;
   psadConfig[NV_EMG_1_RESP2] = 0x03;*/

   // testing array
   psadConfig[NV_EMG_1_CONFIG1] = 0x04;
   psadConfig[NV_EMG_1_CONFIG2] = 0xa3;
   psadConfig[NV_EMG_1_LOFF] = 0x10;
   psadConfig[NV_EMG_1_CH1SET] = 0x05;
   psadConfig[NV_EMG_1_CH2SET] = 0x05;
   psadConfig[NV_EMG_1_RLD_SENS] = 0x00;
   psadConfig[NV_EMG_1_LOFF_SENS] = 0x00;
   psadConfig[NV_EMG_1_LOFF_STAT] = 0x00;
   psadConfig[NV_EMG_1_RESP1] = 0x02;
   psadConfig[NV_EMG_1_RESP2] = 0x01;

   psadConfig[NV_SG_REG0] = 0x03;
   psadConfig[NV_SG_REG1] = 0xd0;
   psadConfig[NV_SG_REG2] = 0x40;
   psadConfig[NV_SG_REG3] = 0x00;

   //#      |7    |6       |5       |4    |3  2  1  0
   //content|BDU  |ABW[1]  |ABW[0]  |AFDS |
   psadConfig[NV_A1_CFG] = 0x00;

   InfoMem_write((void*)0, psadConfig, NV_CONFIG_LEN);

}

void ParseConfig(){
   cfgAccelCnt = psadConfig[NV_CONFIG_0] & CFG_ACCEL_DATA;
   cfgGyroCnt = (psadConfig[NV_CONFIG_0] & CFG_GYRO_DATA)>>2;
   cfgStrainCnt = (psadConfig[NV_CONFIG_0] & CFG_STRAIN_DATA)>>4;

   cfgEmgCnt = (psadConfig[NV_CONFIG_1]>>6) & 0x03;
   cfgGyroRange = (psadConfig[NV_CONFIG_1]>>4) & 0x03;
   cfgAccelRange = (psadConfig[NV_CONFIG_1]>>2) & 0x03;
   cfgAccel303Range = cfgAccelRange>2?cfgAccelRange+1:cfgAccelRange;
   cfgAutoTurnoff = (psadConfig[NV_CONFIG_1]>>1) & 0x01;
   cfgAutoStandby = psadConfig[NV_CONFIG_1] & 0x01;

   cfgSgToShift = (psadConfig[NV_CONFIG_2]>>3) & 0x0f;
   //cfgStrainGain = psadConfig[NV_CONFIG_2] & 0x07;
   cfgStrainGain = (psadConfig[NV_SG_REG0]>>1) & 0x07;

   cfgAccelEn = (cfgAccelCnt == 3)?0:1;
   cfgGyroEn = (cfgGyroCnt == 3)?0:1;
   cfgEmgEn = (cfgEmgCnt == 3)?0:1;
   cfgStrainEn = (cfgStrainCnt == 4)?0:1;

   dataLen = DataPtrCalc();
}

void GetEmg(uint8_t * ptr){
   uint8_t data_emg_cnt;

   if(!cfgEmgCnt){
      GetEmgFromSensor(dataBuffEmg);
      memcpy(ptr, dataBuffEmg+1, 6);
   }
   else if(cfgEmgCnt == 1){
      data_emg_cnt = dataBuffCurrent & 0x01;
      if(!data_emg_cnt){
         GetEmgFromSensor(dataBuffEmg);
         memcpy(ptr, dataBuffEmg+1, 3);
      }
      else{
         memcpy(ptr, dataBuffEmg+4, 3);
      }
   }
   else if(cfgEmgCnt == 2){
      data_emg_cnt = dataBuffCurrent & 0x03;
      if(!data_emg_cnt){
         GetEmgFromSensor(dataBuffEmg);
         memcpy(ptr, dataBuffEmg+1, 2);
      }
      else if (data_emg_cnt == 1){
         memcpy(ptr, dataBuffEmg+3, 2);
      }
      else{
         memcpy(ptr, dataBuffEmg+5, 2);
      }
   }
   else if(cfgEmgCnt == 3){
      // emg off;
   }

   statusReg = (statusReg & 0x07) + (dataBuffEmg[0]<<3);
}

void GetAccel(uint8_t * ptr){
   uint8_t data_accel_cnt;

   if(!cfgAccelCnt){
      test_steps = 14;
      GetAccel1FromSensor(ptr);
      test_steps = 15;
      GetAccel2FromSensor(ptr+6);
   }
   else if(cfgAccelCnt == 1){
      data_accel_cnt = dataBuffCurrent & 0x03;
      GetAccel1FromSensor(dataBuffAccel);
      GetAccel2FromSensor(dataBuffAccel+6);
      ptr[0] = dataBuffAccel[0];
      ptr[1] = dataBuffAccel[1];
      ptr[2] = dataBuffAccel[data_accel_cnt*2+2];
      ptr[3] = dataBuffAccel[data_accel_cnt*2+3];
      ptr[4] = dataBuffAccel[10];
      ptr[5] = dataBuffAccel[11];
//      if(!data_accel_cnt){
//         ptr[2] = dataBuffAccel[2];
//         ptr[3] = dataBuffAccel[3];
//         memcpy(dataBuffAccelTemp, dataBuffAccel+4, 6);
//      }else{
//         ptr[2] = dataBuffAccelTemp[data_accel_cnt*2-2];
//         ptr[3] = dataBuffAccelTemp[data_accel_cnt*2-1];
//      }
   }
   else if(cfgAccelCnt == 2){
      GetAccel1FromSensor(dataBuffAccel);
      GetAccel2FromSensor(dataBuffAccel+6);
      ptr[0] = dataBuffAccel[4];
      ptr[1] = dataBuffAccel[5];
      ptr[2] = dataBuffAccel[10];
      ptr[3] = dataBuffAccel[11];
   }else if(cfgAccelCnt == 3){
      // off
   }
}
void GetGyro(uint8_t * ptr){
   uint8_t data_gyro_cnt;

   if(!cfgGyroCnt){
      GetGyroFromSensor(ptr);
   }
   else if(cfgGyroCnt== 1){
      data_gyro_cnt = dataBuffCurrent & 0x01;
      GetGyroFromSensor(dataBuffGyro);
      ptr[0] = dataBuffGyro[0];
      ptr[1] = dataBuffGyro[1];
      if(!data_gyro_cnt){
         //memcpy(dataBuffGyroTemp, dataBuffGyro, 6);
         ptr[2] = dataBuffGyro[2];
         ptr[3] = dataBuffGyro[3];
         //dataBuffGyroZTemp[0] = dataBuffGyro[4];
         //dataBuffGyroZTemp[1] = dataBuffGyro[5];
      }else{
         ptr[2] = dataBuffGyro[4];//dataBuffGyroZTemp[0];
         ptr[3] = dataBuffGyro[5];//dataBuffGyroZTemp[1];
      }
   }
   else if(cfgGyroCnt == 2){
      GetGyroFromSensor(dataBuffGyro);
      ptr[0] = dataBuffGyro[2];
      ptr[1] = dataBuffGyro[3];
   }else if(cfgGyroCnt == 3){
      // off
   }
}

void GetStrain(uint8_t * ptr){
   uint8_t data_strain_cnt;

   if(!cfgStrainCnt){
      GetStrainFromSensor(ptr,2);
   }else if(cfgStrainCnt == 1){
      data_strain_cnt = dataBuffCurrent & 0x01;
      if(!data_strain_cnt){
         GetStrainFromSensor(dataBuffStrain,2);
      }
      memcpy(ptr, dataBuffStrain + (data_strain_cnt*4), 4);
   }
   else if (cfgStrainCnt == 2){
      data_strain_cnt = dataBuffCurrent & 0x03;
      if(!data_strain_cnt){
         GetStrainFromSensor(dataBuffStrain,2);
      }
      memcpy(ptr, dataBuffStrain + (data_strain_cnt*2), 2);
   }
   else if (cfgStrainCnt == 3){
      GetStrainFromSensor(ptr,3);
   }
   else if (cfgStrainCnt == 4){
      // off
   }
}

uint8_t BtDataAvailable(uint8_t data) {
   if(waitingForArgs) {
      args[argsSize++] = data;
      if (((gAction == BT_SET_INFOMEM) && (argsSize == 1)) ||
          ((gAction == BT_SET_INFOMEM_P) && (argsSize == 1))){
         waitingForArgs += data;
      }
      if(!--waitingForArgs) {
         processCommand = 1;
         argsSize = 0;
         return 1;
      }
      else
         return 0;
   } else {
      switch(data) {
      case BT_GET_FW_VERSION:
      //case BT_GET_VBATT:
      case BT_GET_CONFIG_ALL:
      case BT_GET_DATA_EMG:
      case BT_GET_DATA_ACCEL:
      case BT_GET_DATA_GYRO:
      case BT_GET_DATA_SG:
      case BT_GET_ACCEL_RANGE:
      case BT_GET_GYRO_RANGE:
      case BT_GET_AUTOOFF_EN:
      case BT_GET_STANDBY_EN:
      case BT_SET_CONFIG_DEFAULT:
      case BT_START_SENSING:
      case BT_STOP_SENSING:
      case BT_GET_EMG_CONFIG:
      case BT_LED4_TOGGLE:
      case BT_LED4_ON:
      case BT_LED4_OFF:
      case BT_LED3_TOGGLE:
      case BT_LED3_ON:
      case BT_LED3_OFF:
      case BT_SYNC_CMD:
      case BT_GET_STATUS:
      case BT_GET_SG_BITSHIFT:
      case BT_GET_SG_GAIN:
      case BT_GET_SG_REGS:
      case BT_GET_A1_CFG:
         gAction = data;
         processCommand = 1;
         return 1;
      case BT_SET_DATA_EMG:
      case BT_SET_DATA_ACCEL:
      case BT_SET_DATA_GYRO:
      case BT_SET_DATA_SG:
      case BT_SET_ACCEL_RANGE:
      case BT_SET_GYRO_RANGE:
      case BT_SET_AUTOOFF_EN:
      case BT_SET_STANDBY_EN:
      case BT_SET_SG_BITSHIFT:
      case BT_SET_SG_GAIN:
      case BT_LED3_SET_USERCTRL:
      case BT_SET_A1_CFG:
         waitingForArgs = 1;
         gAction = data;
         return 0;
      //case BT_SET_CONFIG_ALL:
      //   waitingForArgs = 2;
      //   gAction = data;
      //   return 0;
      case BT_SET_CONFIG_ALL:
      case BT_GET_INFOMEM:
      case BT_SET_INFOMEM:
      case BT_SET_INFOMEM_P:
         waitingForArgs = 3;
         gAction = data;
         return 0;
      case BT_SET_SG_REGS:
         waitingForArgs = 4;
         gAction = data;
         return 0;
      case BT_SET_EMG_CONFIG:
         waitingForArgs = 10;
         gAction = data;
         return 0;
      default:
         return 0;
      }
   }
}

void ProcessCommand(void) {
   switch(gAction) {
   case BT_START_SENSING:
      startSensing = 1;
      break;
   case BT_STOP_SENSING:
      if(sensing) {
         stopSensing = 1;
         //return;
      }
      break;
   case BT_SET_CONFIG_DEFAULT:
      PSAD_setDefaultConfig();
      break;
   case BT_LED4_TOGGLE:
      Board_ledToggle(PSAD_LED_4_G);// todo: change led_red to gpio
      break;
   case BT_LED4_ON:
      Board_ledOn(PSAD_LED_4_G);// todo: change led_red to gpio
      break;
   case BT_LED4_OFF:
      Board_ledOff(PSAD_LED_4_G);// todo: change led_red to gpio
      break;
   case BT_LED3_TOGGLE:
      if(led3UserCtrl)
         Board_ledToggle(PSAD_LED_3_Y);// todo: change led_red to gpio
      break;
   case BT_LED3_ON:
      if(led3UserCtrl)
         Board_ledOn(PSAD_LED_3_Y);// todo: change led_red to gpio
      break;
   case BT_LED3_OFF:
      if(led3UserCtrl)
         Board_ledOff(PSAD_LED_3_Y);// todo: change led_red to gpio
      break;
   case BT_GET_FW_VERSION:
      fwVersionRsp = 1;
      break;
   case BT_GET_STATUS:
      statusRsp = 1;
      break;
   //case BT_GET_VBATT:
   //   vbattRsp = 1;
   //   break;
   case BT_GET_CONFIG_ALL:
      configAllRsp = 1;
      break;
   case BT_SET_CONFIG_ALL:
      psadConfig[NV_CONFIG_0] = args[0];
      psadConfig[NV_CONFIG_1] = args[1];
      psadConfig[NV_CONFIG_2] = args[2];
      InfoMem_write((uint8_t*)(NV_CONFIG_0), (psadConfig+NV_CONFIG_0), 3);
      break;
   case BT_GET_DATA_EMG:
      dataEmgRsp = 1;
      break;
   case BT_SET_DATA_EMG:
      if(args[0]<4){
         psadConfig[NV_CONFIG_1] &= ~CFG_EMG_DATA;
         psadConfig[NV_CONFIG_1] |= args[0]<<6;
         InfoMem_write((uint8_t*)(NV_CONFIG_1), (psadConfig+NV_CONFIG_1), 1);
      }
      break;
   case BT_GET_DATA_ACCEL:
      dataAccelRsp = 1;
      break;
   case BT_SET_DATA_ACCEL:
      if(args[0]<4){
         psadConfig[NV_CONFIG_0] &= ~CFG_ACCEL_DATA;
         psadConfig[NV_CONFIG_0] |= args[0];
         InfoMem_write((uint8_t*)(NV_CONFIG_0), (psadConfig+NV_CONFIG_0), 1);
      }
      break;
   case BT_GET_DATA_GYRO:
      dataGyroRsp = 1;
      break;
   case BT_SET_DATA_GYRO:
      if(args[0]<4){
         psadConfig[NV_CONFIG_0] &= ~CFG_GYRO_DATA;
         psadConfig[NV_CONFIG_0] |= args[0]<<2;
         InfoMem_write((uint8_t*)(NV_CONFIG_0), (psadConfig+NV_CONFIG_0), 1);
      }
      break;
   case BT_GET_DATA_SG:
      dataSgRsp = 1;
      break;
   case BT_SET_DATA_SG:
      if(args[0]<5){
         psadConfig[NV_CONFIG_0] &= ~CFG_STRAIN_DATA;
         psadConfig[NV_CONFIG_0] |= args[0]<<4;
         InfoMem_write((uint8_t*)(NV_CONFIG_0), (psadConfig+NV_CONFIG_0), 1);
      }
      break;
   case BT_GET_ACCEL_RANGE:
      accelRangeRsp = 1;
      break;
   case BT_SET_ACCEL_RANGE:
      if(args[0]<4){
         psadConfig[NV_CONFIG_1] &= ~CFG_ACCEL_RANGE;
         psadConfig[NV_CONFIG_1] |= args[0]<<2;
         InfoMem_write((uint8_t*)(NV_CONFIG_1), (psadConfig+NV_CONFIG_1), 1);
      }
      break;
   case BT_GET_GYRO_RANGE:
      gyroRangeRsp = 1;
      break;
   case BT_SET_GYRO_RANGE:
      if(args[0]<4){
         psadConfig[NV_CONFIG_1] &= ~CFG_GYRO_RANGE;
         psadConfig[NV_CONFIG_1] |= args[0]<<4;
         InfoMem_write((uint8_t*)(NV_CONFIG_1), (psadConfig+NV_CONFIG_1), 1);
      }
      break;
   case BT_GET_AUTOOFF_EN:
      autoOffEnRsp = 1;
      break;
   case BT_SET_AUTOOFF_EN:
      if(args[0]<2){
         psadConfig[NV_CONFIG_1] &= ~CFG_AUTO_TURNOFF_EN;
         psadConfig[NV_CONFIG_1] |= args[0];
         InfoMem_write((uint8_t*)(NV_CONFIG_1), (psadConfig+NV_CONFIG_1), 1);
      }
      break;
   case BT_GET_STANDBY_EN:
      standbyEnRsp = 1;
      break;
   case BT_SET_STANDBY_EN:
      if(args[0]<2){
         psadConfig[NV_CONFIG_1] &= ~CFG_AUTO_STANDBY_EN;
         psadConfig[NV_CONFIG_1] |= args[0]<<1;
         InfoMem_write((uint8_t*)(NV_CONFIG_1), (psadConfig+NV_CONFIG_1), 1);
      }
      break;
   case BT_GET_EMG_CONFIG:
      emgConfigRsp = 1;
      break;
   case BT_SET_EMG_CONFIG:
      memcpy(psadConfig+NV_EMG_1_CONFIG1, args, 10);
      InfoMem_write((uint8_t*)(NV_EMG_1_CONFIG1), (psadConfig+NV_EMG_1_CONFIG1), 10);
      break;
   case BT_GET_INFOMEM:
      infomemLength = args[0];
      infomemOffset = args[1] + (args[2]<<8);
      if((infomemLength<=128) && (infomemOffset<=(NV_NUM_RWMEM_BYTES-1)) && (infomemLength+infomemOffset<=NV_NUM_RWMEM_BYTES))
         infomemResponse = 1;
      break;
   case BT_SET_INFOMEM:
      infomemLength = args[0];
      infomemOffset = args[1] + (args[2]<<8);
      //memcpy(psadConfig+infomemOffset, args+3, infomemLength);
      InfoMem_write((uint8_t*)(infomemOffset), args+3, infomemLength);
      InfoMem_read((uint8_t*)0, psadConfig, NV_CONFIG_LEN);
      break;
   case BT_SET_INFOMEM_P:
      infomemLength = args[0];
      infomemOffset = args[1] + (args[2]<<8);
      //memcpy(psadConfig+infomemOffset, args+3, infomemLength);
      InfoMem_read((uint8_t*)0, psadConfig, NV_CONFIG_LEN);
      InfoMem_write((uint8_t*)(infomemOffset), args+3, infomemLength);
      InfoMem_write((uint8_t*)0, psadConfig, NV_CONFIG_LEN);
      break;
   case BT_SYNC_CMD:
      syncRsp = 1;
      break;
   case BT_GET_SG_BITSHIFT:
      sgShiftRsp = 1;
      break;
   case BT_SET_SG_BITSHIFT:
      if(args[0]<9){
         //cfgSgToShift = args[0];
         psadConfig[NV_CONFIG_2] &= 0x87;
         psadConfig[NV_CONFIG_2] |= (args[0]<<3);
         InfoMem_write((uint8_t*)(NV_CONFIG_2), (psadConfig+NV_CONFIG_2), 1);
      }
      // todo: find a place (4 bits) in infomem for this value?
      break;
   case BT_GET_SG_GAIN:
      sgGainRsp = 1;
      break;
   case BT_SET_SG_GAIN:
      if(args[0]<8){// arg = 0-7, gain = 1-128
         psadConfig[NV_CONFIG_2] &= 0xf8;
         psadConfig[NV_CONFIG_2] |= args[0];
         InfoMem_write((uint8_t*)(NV_CONFIG_2), (psadConfig+NV_CONFIG_2), 1);
         psadConfig[NV_SG_REG0] &= 0xf1;
         psadConfig[NV_SG_REG0] |= args[0]<<1;
         InfoMem_write((uint8_t*)(NV_SG_REG0), (psadConfig+NV_SG_REG0), 1);
      }
      // todo: find a place (4 bits) in infomem for this value
      break;
   case BT_LED3_SET_USERCTRL:
      led3UserCtrl = args[0];
      break;
   case BT_GET_SG_REGS:
      sgRegRsp = 1;
      break;
   case BT_SET_SG_REGS:
	  psadConfig[NV_SG_REG0] = args[0];
	  psadConfig[NV_SG_REG1] = args[1];
	  psadConfig[NV_SG_REG2] = args[2];
	  psadConfig[NV_SG_REG3] = args[3];
	  InfoMem_write((uint8_t*)(NV_SG_REG0), (psadConfig+NV_SG_REG0), 4);
     psadConfig[NV_CONFIG_2] &= 0xf8;
     psadConfig[NV_CONFIG_2] |= (args[0]>>1)&0x07;
     InfoMem_write((uint8_t*)(NV_CONFIG_2), (psadConfig+NV_CONFIG_2), 1);
      break;
   case BT_GET_A1_CFG:
      a1CfgRsp = 1;
      break;
   case BT_SET_A1_CFG:
     psadConfig[NV_A1_CFG] = args[0];
     InfoMem_write((uint8_t*)(NV_A1_CFG), (psadConfig+NV_A1_CFG), 1);
      break;

   default:break;
   }
   sendAck = 1;
   sendResponse = 1;
}

void SendResponse(void) {
   uint16_t packet_length = 0;

   if(btIsConnected) {
      if(sendAck) {
         *(resPacket + packet_length++) = BT_RSP_ACK;
         sendAck = 0;
      }
      //fwVersionRsp, statusRsp, configAllRsp, dataEmgRsp, dataAccelRsp, dataGyroRsp, dataSgRsp;
      if(fwVersionRsp){
         *(resPacket + packet_length++) = BT_RSP_FW_VERSION;
         *(resPacket + packet_length++) = FW_VER_MAJOR;
         *(resPacket + packet_length++) = FW_VER_MINOR;
         *(resPacket + packet_length++) = FW_VER_REL;
         fwVersionRsp = 0;
      }
      //vbattRsp
      if(statusRsp){//todo: add value
         *(resPacket + packet_length++) = BT_RSP_STATUS;
         //memcpy((resPacket+packet_length), &psadConfig[NV_EMG_1_CONFIG1], 3);
         memcpy((resPacket+packet_length), battVal, 3);
         packet_length += 3;
//         *(resPacket + packet_length++) = sw1Cnt;
//         *(resPacket + packet_length++) = sw2Cnt;
         *(resPacket + packet_length++) = (sw1Cnt & 0x7f) + (sw1New << 7);
         *(resPacket + packet_length++) = (sw2Cnt & 0x7f) + (sw2New << 7);
         *(resPacket + packet_length++) = (vibCnt & 0x7f) + (vibNew << 7);
         *(resPacket + packet_length++) = statusReg;
         sw1New = 0;
         sw2New = 0;
         vibNew = 0;
         statusRsp = 0;
      }
      if(configAllRsp){
         *(resPacket + packet_length++) = BT_RSP_CONFIG_ALL;
         *(resPacket + packet_length++) = psadConfig[NV_CONFIG_0];
         *(resPacket + packet_length++) = psadConfig[NV_CONFIG_1];
         *(resPacket + packet_length++) = psadConfig[NV_CONFIG_2];
         configAllRsp = 0;
      }
      if(dataEmgRsp){
         *(resPacket + packet_length++) = BT_RSP_DATA_EMG;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_1] & CFG_EMG_DATA)>>6;
         dataEmgRsp = 0;
      }
      if(dataAccelRsp){
         *(resPacket + packet_length++) = BT_RSP_DATA_ACCEL;
         *(resPacket + packet_length++) = psadConfig[NV_CONFIG_0] & CFG_ACCEL_DATA;
         dataAccelRsp = 0;
      }
      if(dataGyroRsp){
         *(resPacket + packet_length++) = BT_RSP_DATA_GYRO;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_0] & CFG_GYRO_DATA)>>2;
         dataGyroRsp = 0;
      }
      if(dataSgRsp){
         *(resPacket + packet_length++) = BT_RSP_DATA_SG;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_0] & CFG_STRAIN_DATA)>>4;
         dataSgRsp = 0;
      }
      if(accelRangeRsp){
         *(resPacket + packet_length++) = BT_RSP_ACCEL_RANGE;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_1] & CFG_ACCEL_RANGE)>>2;
         accelRangeRsp = 0;
      }
      if(gyroRangeRsp){
         *(resPacket + packet_length++) = BT_RSP_GYRO_RANGE;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_1] & CFG_GYRO_RANGE)>>4;
         gyroRangeRsp = 0;
      }
      if(autoOffEnRsp){
         *(resPacket + packet_length++) = BT_RSP_AUTOOFF_EN;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_1] & CFG_AUTO_TURNOFF_EN);
         autoOffEnRsp = 0;
      }
      if(standbyEnRsp){
         *(resPacket + packet_length++) = BT_RSP_STANDBY_EN;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_1] & CFG_AUTO_STANDBY_EN)>>1;
         standbyEnRsp = 0;
      }
      if(emgConfigRsp){
         *(resPacket + packet_length++) = BT_RSP_EMG_CONFIG;
         memcpy((resPacket+packet_length), &psadConfig[NV_EMG_1_CONFIG1], 10);
         packet_length += 10;
         emgConfigRsp = 0;
      }
      if(infomemResponse) {
         *(resPacket + packet_length++) = BT_RSP_INFOMEM;
         *(resPacket + packet_length++) = infomemLength;
         InfoMem_read((uint8_t*)infomemOffset, resPacket + packet_length, infomemLength);
         packet_length += infomemLength;
         infomemResponse = 0;
      }
      if(syncRsp) {
         uint8_t i;
         for (i=1;i<SYNC_STRING_LEN;i++)
            *(resPacket + packet_length++) = i;// from 01 to 0a, following the 0xff, and followed by the 2nd 0xff
         *(resPacket + packet_length++) = 0xff;
         syncRsp = 0;
      }
      /*if(statusRsp) {
         uint8_t i;
         for (i=1;i<SYNC_STRING_LEN;i++)
            *(resPacket + packet_length++) = i;// from 01 to 0a
         *(resPacket + packet_length++) = 0xff;
         *(resPacket + packet_length++) = BT_RSP_STATUS;
         *(resPacket + packet_length++) = statusReg;
         statusRsp = 0;
      }*/
      if(sgShiftRsp) {
         *(resPacket + packet_length++) = BT_RSP_SG_BITSHIFT;
         //*(resPacket + packet_length++) = cfgSgToShift;
         *(resPacket + packet_length++) = (psadConfig[NV_CONFIG_2]>>3) & 0x0f;
         sgShiftRsp = 0;
      }
      if(sgGainRsp) {
         *(resPacket + packet_length++) = BT_RSP_SG_GAIN;
         //*(resPacket + packet_length++) = psadConfig[NV_CONFIG_2] & 0x07;
         *(resPacket + packet_length++) = (psadConfig[NV_SG_REG0] & 0x0E)>>1;
         sgGainRsp = 0;
      }
      if(sgRegRsp) {
         *(resPacket + packet_length++) = BT_RSP_SG_REGS;
         *(resPacket + packet_length++) = psadConfig[NV_SG_REG0];
         *(resPacket + packet_length++) = psadConfig[NV_SG_REG1];
         *(resPacket + packet_length++) = psadConfig[NV_SG_REG2];
         *(resPacket + packet_length++) = psadConfig[NV_SG_REG3];
         sgRegRsp = 0;
      }
      if(a1CfgRsp) {
         *(resPacket + packet_length++) = BT_RSP_A1_CFG;
         *(resPacket + packet_length++) = psadConfig[NV_A1_CFG];
         a1CfgRsp = 0;
      }


   }
   //BT_write(resPacket, packet_length);
   BT_append(resPacket, packet_length);
}


uint8_t Dma0BatteryRead(void) {
   ADC_disable();
   DMA0_disable();
   DMA0BicOnExit = 1;
   return 1;
}

void ReadBatt(void){
   DMA0_enable();
   SaveSpiOff();
   ADC_startConversion();
   __bis_SR_register(LPM3_bits + GIE);
   DMA0BicOnExit = 0;
   LoadSpiOn();
   battVal[2] = (P2IN & 0x0c)<<4;

   if(battStat == BATT_HIGH){
      if(*(uint16_t*)battVal<2400){
         battStat = BATT_LOW;
      }
      //else// battStat = BATT_HIGH;
      //   statusReg |= BIT0;
   }else {
      if(*(uint16_t*)battVal>2450){
         battStat = BATT_HIGH;
         //statusReg |= BIT0;
      }
   }
   statusReg = statusReg & 0xf8;
   statusReg |= battStat == BATT_HIGH?BIT0:0;
   statusReg |= docked?BIT1:0;
   statusReg |= ((P2IN&BIT2) && !(P2IN&BIT3))?BIT2:0;// completed?0x04:0
}

void ConfigVBatt(void){
   adcStartPtr = ADC_init(MASK_VBATT);
   DMA0_transferDoneFunction(&Dma0BatteryRead);
   if(adcStartPtr) {
      DMA0_init(adcStartPtr, (uint16_t *)(battVal), 1);
   }
}

void EnterStandby(void){
   I2C_Disable();
   P7OUT &= ~BIT5;         //set SW_I2C low to power off I2C chips
}

void ExitStandby(void){
//   LSM303DTR_init();
   I2C_PowerOn();
}


uint8_t DataPtrCalc(void){ // return the new total data length
   //uint8_t new_len = 35, shortened_len = 0;
   uint8_t new_len = 39, shortened_len = 0;

   ptrEMG = DATA_EMG1;
   ptrAccel1X = DATA_ACCEL1_X;
   ptrGyroX = DATA_GYRO_X;
   ptrStrain = DATA_STRAIN_GAUGE_YL;

   // dataXXXXXCntCfg = 0: @1024hz;
   // 1: proposed plan;
   // 2: @512hz;
   // 3: @256hz;
   switch(cfgEmgCnt){
   case 1: shortened_len += 3; break;
   case 2: shortened_len += 4; break;
   case 3: shortened_len += 6; break;
   default: break;
   }
   ptrAccel1X -= shortened_len;

   switch(cfgAccelCnt){
   case 1: shortened_len += 6; break;
   case 2: shortened_len += 8; break;
   case 3: shortened_len += 12; break;
   default: break;
   }

   ptrGyroX -= shortened_len;

   switch(cfgGyroCnt){
   case 1: shortened_len += 2; break;
   case 2: shortened_len += 4; break;
   case 3: shortened_len += 6; break;
   default: break;
   }

   ptrStrain -= shortened_len;
   switch(cfgStrainCnt){
   case 0: shortened_len += 4; break;
   case 1: shortened_len += 8; break;
   case 2: shortened_len += 10; break;
   case 3: shortened_len += 0; break;
   case 4: shortened_len += 12; break;
   default: break;
   }

   return (new_len - shortened_len);
}


/**
*** Switch SW1, BT_RTS and BT connect/disconnect
**/
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
   // Context save interrupt flag before calling interrupt vector.
   // Reading interrupt vector generator will automatically clear IFG flag

   switch (__even_in_range(P1IV, P1IV_P1IFG7)) {
   //BT Connect/Disconnect
   case  P1IV_P1IFG0:   //BT Connect/Disconnect
      if(P1IN & BIT0) {
         //BT is connected
         P1IES |= BIT0; //look for falling edge
         BT_connectionInterrupt(1);
         btIsConnected = 1;
//         BlinkTimerStop();
         waitingForArgs = 0;
      } else {
         //BT is not connected
         P1IES &= ~BIT0; //look for rising edge
         BT_connectionInterrupt(0);
         btIsConnected = 0;
         if(sensing) {
            stopSensing = 1;
         }
         discTs = RTC_get64();
//         BlinkTimerStart();
      }
      break;

   //BT RTS
   case  P1IV_P1IFG3:
      if(P1IN & BIT3) {
         P1IES |= BIT3;    //look for falling edge
         BT_rtsInterrupt(1);
      } else {
         P1IES &= ~BIT3;   //look for rising edge
         BT_rtsInterrupt(0);
      }
      break;

   case P1IV_P1IFG4://sw1
      if(P1IN & BIT4) {
         P1IES |= BIT4;    //look for falling edge
         sw1Status = 0;
         sw1New = 1;
         sw1Cnt ++;
      } else {
         P1IES &= ~BIT4;   //look for rising edge
         sw1Status = 1;
      }
      break;

   /*case  P1IV_P1IFG5:
      if(P1IN & BIT5) {
         P1IES |= BIT5;    //look for falling edge
      } else {
         P1IES &= ~BIT5;   //look for rising edge
      }
      break;*/

   //BUTTON_SW1
   /*case  P1IV_P1IFG6:
      //disable switch interrupts
      Button_debounce();
      break;*/

   case  P1IV_P1IFG7:
      //disable switch interrupts
//      if(P1IN & BIT7) {
//         P1IES |= BIT7;    //look for falling edge
//      } else {
//         P1IES &= ~BIT7;   //look for rising edge
//
//         UCB1SPI_SG_dataReady(sgGoOn);
//      }
      UCB1SPI_SG_dataReady();//sgGoOn
      break;


   // Default case
   default:
      break;
   }
   if((streamData && cpuIsSleeping) || *i2cBicOnExit || DMA0BicOnExit)
      __bic_SR_register_on_exit(LPM3_bits);
}
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void) {

   switch (__even_in_range(P2IV, P2IV_P2IFG7)) {
   //ExG chip1 data ready
   case  P2IV_P2IFG0:
      UCB1SPI_EMG_dataReady();
      break;

   //CHG_STAT1
   case P2IV_P2IFG2:
      SetChargeStatusLeds();
      if(P2IN & BIT2) {
         P2IES |= BIT2;    //look for falling edge
      } else {
         P2IES &= ~BIT2;   //look for rising edge
      }
      break;

   //CHG_STAT2
   case P2IV_P2IFG3:
      SetChargeStatusLeds();
      if(P2IN & BIT3) {
         P2IES |= BIT3;    //look for falling edge
      } else {
         P2IES &= ~BIT3;   //look for rising edge
      }
      break;

   //SW2_STATUS
   case P2IV_P2IFG4:
      if(P2IN & BIT4) {//release
         P2IES |= BIT4;    //look for falling edge
         sw2Release = RTC_get32();
         if(((sw2Release - sw2Press) > 327) && ((sw2Release-sw2LastRelease)>3277)){
            sw2Cnt ++;
            sw2LastRelease = sw2Release;
            sw2New = 1;
         }
         sw2Status = 0;
      } else {//press
         P2IES &= ~BIT4;   //look for rising edge
         sw2Status = 1;
         sw2Press = RTC_get32();
         //sw2Press, sw2Release, sw2LastRelease;
      }
      break;

   // vibration
   case P2IV_P2IFG5:
      if(P2IN & BIT5) {
         P2IES |= BIT5;    //look for falling edge
         vibStatus = 0;
      } else {
         P2IES &= ~BIT5;   //look for rising edge
         vibStatus = 1;
      }
      vibCnt++;
      vibNew = 1;
      break;

   case P2IV_P2IFG6:
      _NOP();
      break;

   case P2IV_P2IFG7:
      if(P2IN & BIT7) {
         P2IES |= BIT7;    //look for falling edge
         BT_rtsInterrupt(1);
      } else {
         P2IES &= ~BIT7;   //look for rising edge
         BT_rtsInterrupt(0);
      }
      break;
   // Default case
   default: break;
   }
   if((streamData && cpuIsSleeping) || *i2cBicOnExit || DMA0BicOnExit)
      __bic_SR_register_on_exit(LPM3_bits);
}

inline void SetLed(void) {
   switch(selectedLed) {
   case 0:
      //Board_ledOn(LED_GREEN0);
      break;
   case 1:
      //Board_ledOn(LED_YELLOW0);
      break;
   default:
      //Board_ledOn(LED_RED);
      break;
   }
}

inline void ClearLed(void) {
   switch(selectedLed) {
   case 0:
      //Board_ledOff(LED_GREEN0);
      break;
   case 1:
      //Board_ledOff(LED_YELLOW0);
      break;
   default:
      //Board_ledOff(LED_RED);
      break;
   }
}

inline uint8_t IsLedSet(void) {
   switch(selectedLed) {
   case 0:
      if(P7OUT & BIT3) return 1;
      else return 0;
   case 1:
      if(P8OUT & BIT0) return 1;
      else return 0;
   default:
      if(P7OUT & BIT2) return 1;
      else return 0;
   }
}

/**
*** TB0 is used for both the blink, charge status and sample timers
**/

/**
*** Charge Status Timer
**/
void ChargeStatusTimerStart(void) {
   TB0CCR3 = GetTB0() + ACLK_100ms;
   TB0CCTL3 = CCIE;
   SetLed();
}

void ChargeStatusTimerStop(void) {
   TB0CCTL3 = 0;
   ClearLed();
}

/**
*** Monitor charge status
**/
inline void MonitorChargeStatus(void) {
   if(P2IN & BIT2) {    //CHG_STAT1
      P2IES |= BIT2;    //look for falling edge
   } else {
      P2IES &= ~BIT2;   //look for rising edge
   }
   if(P2IN & BIT3) {    //CHG_STAT2
      P2IES |= BIT3;    //look for falling edge
   } else {
      P2IES &= ~BIT3;   //look for rising edge
   }
   P2IFG &= ~(BIT2 + BIT3);

   SetChargeStatusLeds();

   P2IE |= BIT2 + BIT3; //enable interrupts
}

inline void MonitorChargeStatusStop(void) {
   P2IE &= ~(BIT2 + BIT3);
   //Board_ledOff(LED_RED + LED_YELLOW0 + LED_GREEN0);
}

#define BATT_NO_POWER      0
#define BATT_CHARGING      1
#define BATT_CHARGE_DONE   2


inline void SetChargeStatusLeds(void) {
   //Board_ledOff(LED_RED + LED_YELLOW0 + LED_GREEN0);

   if((P2IN&BIT2) && !(P2IN&BIT3)) {
      //charge completed
      //Board_ledOn(PSAD_LED_1_R);
      chargStatus = BATT_CHARGE_DONE;
   } else if(!(P2IN&BIT2) && (P2IN&BIT3)) {
      //charging
      //Board_ledOn(PSAD_LED_1_R);
      chargStatus = BATT_CHARGING;
   } else if(!(P2IN&BIT2) &&!(P2IN&BIT3)) {
      chargStatus = BATT_NO_POWER;
   }
}
/**
*** Blink Timer
**/
// USING TB0 with CCR4
void BlinkTimerStart(void) {
   if(sensing) {
      TB0CCR4 = GetTB0() + ACLK_1S;   //1Hz
      //Board_ledOff(LED_BLUE);
   } else {
      TB0CCR4 = GetTB0() + ACLK_100ms;
      //Board_ledOn(LED_BLUE);
//      if(!docked) {
//         //to keep both LEDs in sync
//         ChargeStatusTimerStop();
//         ChargeStatusTimerStart();
//      }
   }
   TB0CCTL4 = CCIE;
}

//inline void BlinkTimerStop() {
//   TB0CCTL4 = 0;
//   //Board_ledOn(LED_BLUE);
//}

/**
*** Sample Timer
**/
void SampleTimerStart(void) {
   StopTB0();
   TB0CCTL1 = 0;
   TB0CCTL2 = 0;
   TB0CCR0 = PSAD_PERIOD;
   TB0CCTL0 = CCIE;
   StartTB0();
}

inline void SampleTimerStop(void) {
   TB0CCTL1 = 0;
   TB0CCTL2 = 0;
   TB0CCTL0 = 0;
}


#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
   //main sampling rate control
   TB0CCR0 += PSAD_PERIOD;
   dataBuffTimer++;
   streamData = 1;
   if(cpuIsSleeping){
      __bic_SR_register_on_exit(LPM3_bits);
      //Board_ledToggle(PSAD_LED_4_G);
   }
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void) {
   switch(__even_in_range(TB0IV,14)) {
   case  0: break;                        // No interrupt
   case  2: break;                        // TB0CCR1
   case  4: break;                        // TB0CCR2
   case  6:                               // TB0CCR3
      //Charge status LED
//      if(IsLedSet()) {                    //check current status of LED
//         TB0CCR3 += ACLK_2S;
//         ClearLed();
//      } else {
//         //LED is off
//         TB0CCR3 += ACLK_100ms;
//          SetLed();
//      }
      break;
   case  8:                               // TB0CCR4

      TB0CCR4 += ACLK_100ms;
//
//      if(P6IN & BIT2) {
//         if(!btIsConnected){
//            btIsConnected = 1;
//            BT_connectionInterrupt(1);
//            waitingForArgs = 0;
//         }
//      } else {
//         if(btIsConnected){
//            btIsConnected = 0;
//            BT_connectionInterrupt(0);
//            if(sensing) {
//               stopSensing = 1;
//            }
//            discTs = RTC_get64();
//         }
//      }
      ledCounter ++;
      if(ledCounter >= 20){
         ledCounter = 0;
         readBatt = 1;
         if(cpuIsSleeping)
            __bic_SR_register_on_exit(LPM3_bits);

         if(psadConfig[NV_CONFIG_1] & CFG_AUTO_TURNOFF_EN){
            currentTs = RTC_get64();
            if(!btIsConnected && ((currentTs - discTs)>9830400)){
               P6DIR |= BIT5;
               P6OUT &= ~BIT5;
            }
         }
      }

      //P1DIR &= ~BIT0;  //todo: this pin is going to be bt_status_int

      //==== led_1_red + led_1_yellow
      if(chargStatus == BATT_NO_POWER){// considered as undocked
         Board_ledOff(PSAD_LED_1_Y);
         if(battStat == BATT_HIGH){
            //normal: green on
            Board_ledOn(PSAD_LED_1_R);
         }else{
            if(!(ledCounter%5))
               Board_ledOn(PSAD_LED_1_R);
            else
               Board_ledOff(PSAD_LED_1_R);
         }
      }
      else if (chargStatus == BATT_CHARGING){
         if(!(ledCounter%5))
            Board_ledOn(PSAD_LED_1_Y);
         else
            Board_ledOff(PSAD_LED_1_Y);
      }
      else if (chargStatus == BATT_CHARGE_DONE){
         Board_ledOn(PSAD_LED_1_Y);
      }

      //==== led_2_blue
      //bt connected: blue on
      //bt on but not connected: blue flash
      //bt off: blue off
      if(btIsConnected){
         Board_ledOn(PSAD_LED_2_B);
      }else{
         if(btIsPowerOn){
            if(!(ledCounter%5))
               Board_ledOn(PSAD_LED_2_B);
            else
               Board_ledOff(PSAD_LED_2_B);
         }else{
            Board_ledOff(PSAD_LED_2_B);
         }
      }
      //==== led_3_yellow: controled by the user sw

      if(!led3UserCtrl){
         if(configuring)
            Board_ledToggle(PSAD_LED_3_Y);
         else
            if(sensing){
               if(ledCounter<10)
                  Board_ledOn(PSAD_LED_3_Y);
               else
                  Board_ledOff(PSAD_LED_3_Y);
            }
            else{
               if(!ledCounter)
                  Board_ledOn(PSAD_LED_3_Y);
               else
                  Board_ledOff(PSAD_LED_3_Y);
            }
      }

      if((streamData && cpuIsSleeping) || *i2cBicOnExit || DMA0BicOnExit)
         __bic_SR_register_on_exit(LPM3_bits);

      break;
   case 10: break;                       // reserved
   case 12: break;                       // reserved
   case 14: break;                       // TBIFG overflow handler
   }
}


/**
*** DMA interrupt vector
**/
//uint8_t Dma0ConversionDone(void) {
//   if(currentBuffer){
//      DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff0+4), nbrAdcChans); //Destination address for next transfer
//   } else {
//      DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff1+4), nbrAdcChans); //Destination address for next transfer
//   }
//   ADC_disable();    //can disable ADC until next time sampleTimer fires (to save power)?
//   DMA0_disable();
//   streamData = 1;
//   return 1;
//}




#pragma vector=USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
   switch(__even_in_range(UCB1IV,4)) {
   case 0:break;                       //Vector 0 - no interrupt

   case 2:                             //Vector 2 - RXIFG
      if(ucb1RxIsr())
         __bic_SR_register_on_exit(LPM3_bits);
      break;
   case 4:
      if(ucb1TxIsr())
         __bic_SR_register_on_exit(LPM3_bits);
      break;                           //Vector 4 - TXIFG

   default: break;
   }
   if((streamData && cpuIsSleeping) || *i2cBicOnExit || DMA0BicOnExit)
      __bic_SR_register_on_exit(LPM3_bits);
}


/**
***
 */
// trap isr assignation - put all unused ISR vector here
#pragma vector =TIMER0_A0_VECTOR, \
   TIMER0_A1_VECTOR, TIMER1_A1_VECTOR, \
   UNMI_VECTOR, WDT_VECTOR, SYSNMI_VECTOR
__interrupt void TrapIsr(void) {
  // this is a trap ISR - check for the interrupt cause here by
  // checking the interrupt flags, if necessary also clear the interrupt
  // flag
}
//USCI_B1_VECTOR for spi:SG+EMG
//RTC_VECTOR,
