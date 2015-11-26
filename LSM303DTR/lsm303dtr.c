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

#include "msp430.h"
#include "hal_I2C.h"
#include "lsm303dtr.h"


uint8_t mag_spike_x, mag_spike_y, mag_spike_z;
uint8_t lastMagVal[6];


//configure I2C
void LSM303DTR_init(void) {
   P7OUT |= BIT5;                       //set SW_I2C high to power on all I2C chips
   __delay_cycles(24000000);                 //wait 1s (assuming 24MHz MCLK) to allow for power ramp up

   I2C_Master_Init(S_MCLK,24000000,400000);  //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
}


void LSM303DTR_accelInit(uint8_t samplingrate, uint8_t range, uint8_t zyx) {
   uint8_t i2c_buffer[2];

   if(samplingrate > 10) samplingrate = 5;
   if(range > 4) range = 0; // 0~4: 2,4,6,8,16
   uint8_t zyx_all = 7;// or use xyz

   //Configure Accel
   I2C_Set_Slave_Address(LSM303DTR_ADDR);
   //__delay_cycles(2400000);
   //write CTRL_REG1_A register
   i2c_buffer[0] = LSM303DTR_CTRL_REG1_A;
   i2c_buffer[1] = (samplingrate << 4) + zyx_all;
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
   //write CTRL_REG2_A register
   i2c_buffer[0] = LSM303DTR_CTRL_REG2_A;
   i2c_buffer[1] = (range << 3);
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
}

uint8_t LSM303DTR_getWhoAmI(void) {
   //I2C_Disable();
   //I2C_Enable();
   uint8_t buf;
   I2C_Set_Slave_Address(LSM303DTR_ADDR);
   //__delay_cycles(24000000);
   buf = LSM303DTR_WHO_AM_I;
   I2C_Read_Packet_From_Sensor(&buf, 1);
   return buf;
}

void LSM303DTR_getAccel(uint8_t *buf) {
   I2C_Set_Slave_Address(LSM303DTR_ADDR);
   // need to assert MSB of sub-address in order to read multiple bytes.
   // See section 5.1.2 of LSM303DLHC datasheet (April 2011, Doc ID 018771 Rev1) for details
   *buf = LSM303DTR_OUT_X_L_A | 0x80;
   I2C_Read_Packet_From_Sensor(buf, 6);
}
void LSM303DTR_getAccelAxis(uint8_t *buf, uint8_t xyz) {
   // this function only reads one axis, 2 bytes
   if(xyz & LSM303DTR_X)
      *buf = LSM303DTR_OUT_X_L_A | 0x80;
   else if(xyz & LSM303DTR_Y)
      *buf = LSM303DTR_OUT_Y_L_A | 0x80;
   else if(xyz & LSM303DTR_Z)
      *buf = LSM303DTR_OUT_Z_L_A | 0x80;
   else
      return;

   I2C_Set_Slave_Address(LSM303DTR_ADDR);
   // need to assert MSB of sub-address in order to read multiple bytes.
   // See section 5.1.2 of LSM303DLHC datasheet (April 2011, Doc ID 018771 Rev1) for details

   I2C_Read_Packet_From_Sensor(buf, 2);
}

void LSM303DTR_sleep(void) {
   uint8_t i2c_buffer[2];

   I2C_Set_Slave_Address(LSM303DTR_ADDR);
   //write CTRL_REG1_A register
   i2c_buffer[0] = LSM303DTR_CTRL_REG1_A;
   i2c_buffer[1] = 0;      //power down mode, 3 axes disabled
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
}
