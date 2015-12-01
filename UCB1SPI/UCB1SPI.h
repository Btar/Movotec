/*
 * UCB1SPI.h
 *
 *  Created on: Oct 21, 2015
 *      Author: WeiboP
 */

#ifndef UCB1SPI_H_
#define UCB1SPI_H_

#include <stdint.h>



void UCB1SPI_Init();

//============================== below is strain gauge ==============================

void UCB1SPI_SG_init(uint8_t *regs);

//void ucb1spi_SG_SetChannel(int Mux);

void UCB1SPI_SG_SendStartCommand(void);

void UCB1SPI_SG_SendShutdownCommand(void);

void UCB1SPI_SG_SetGain(uint8_t gain);

void UCB1SPI_SG_ReadData(uint8_t* buf);

//void UCB1SPI_SG_dataReady();
void UCB1SPI_SG_dataReady();

void UCB1SPI_SG_dataRx_done();

//============================== above is strain gauge ==============================

//============================== below is emg ==============================


#define ADS1292R_DEVID     0x00
#define ADS1292R_CONFIG1   0x01
#define ADS1292R_CONFIG2   0x02
#define ADS1292R_LOFF      0x03
#define ADS1292R_CH1SET    0x04
#define ADS1292R_CH2SET    0x05
#define ADS1292R_RLD_SENS  0x06
#define ADS1292R_LOFF_SENS 0x07
#define ADS1292R_LOFF_STAT 0x08
#define ADS1292R_RESP1     0x09
#define ADS1292R_RESP2     0x0A


void UCB1SPI_EMG_init(void);

void UCB1SPI_EMG_start(void);

void UCB1SPI_EMG_stop(void);

void UCB1SPI_EMG_writeRegs(uint8_t startaddress, uint8_t size, uint8_t *wdata);

void UCB1SPI_EMG_offsetCal(void);

void UCB1SPI_EMG_readData(uint8_t* buf);

void UCB1SPI_EMG_dataReady();
//=============================== above is emg =============================

void ucb1SetStreamData(uint8_t stream_Data);
uint8_t ucb1RxIsr();
uint8_t ucb1TxIsr();



#endif /* UCB1SPI_H_ */
