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

#ifndef LSM303DTR_H
#define LSM303DTR_H

#include <stdint.h>

#define LSM303DTR_ADDR                 0x1E // 7 bit address of the LSM303DTR

//registers
#define LSM303DTR_WHO_AM_I             0x0F
#define LSM303DTR_CTRL_REG1_A          0x20
#define LSM303DTR_CTRL_REG2_A          0x21
#define LSM303DTR_CTRL_REG3_A          0x22
#define LSM303DTR_CTRL_REG4_A          0x23
#define LSM303DTR_CTRL_REG5_A          0x24
#define LSM303DTR_CTRL_REG6_A          0x25
#define LSM303DTR_CTRL_REG7_A          0x26
#define LSM303DTR_STATUS_REG_A         0x27
#define LSM303DTR_OUT_X_L_A            0x28
#define LSM303DTR_OUT_X_H_A            0x29
#define LSM303DTR_OUT_Y_L_A            0x2A
#define LSM303DTR_OUT_Y_H_A            0x2B
#define LSM303DTR_OUT_Z_L_A            0x2C
#define LSM303DTR_OUT_Z_H_A            0x2D
/*#define FIFO_CTRL_REG_A       0x2E
#define FIFO_SRC_REG_A        0x2F
#define INT1_CFG_A            0x30
#define INT1_SOURCE_A         0x31
#define INT1_THS_A            0x32
#define INT1_DURATION_A       0x33
#define INT2_CFG_A            0x34
#define INT2_SOURCE_A         0x35
#define INT2_THS_A            0x36
#define INT2_DURATION_A       0x37
#define CLICK_CFG_A           0x38
#define CLICK_SRC_A           0x39
#define CLICK_THS_A           0x3A
#define TIME_LIMIT_A          0x3B
#define TIME_LATENCY_A        0x3C
#define TIME_WINDOW_A         0x3D
#define CRA_REG_M             0x00
#define CRB_REG_M             0x01
#define MR_REG_M              0x02
#define OUT_X_H_M             0x03
#define OUT_X_L_M             0x04
#define OUT_Z_H_M             0x05
#define OUT_Z_L_M             0x06
#define OUT_Y_H_M             0x07
#define OUT_Y_L_M             0x08
#define SR_REG_M              0x09
#define IRA_REG_M             0x0A
#define IRB_REG_M             0x0B
#define IRC_REG_M             0x0C
#define TEMP_OUT_H_M          0x31
#define TEMP_OUT_L_M          0x32*/

//initialise the I2C for use with the LSM303DLHC
void LSM303DTR_init(void);

//initialise the accelerometer
void LSM303DTR_accelInit(uint8_t samplingrate, uint8_t range, uint8_t zyx);

//put x, y and z accel values into buf
void LSM303DTR_getAccel(uint8_t *buf);

#define LSM303DTR_X            0x04
#define LSM303DTR_Y            0x02
#define LSM303DTR_Z            0x01
//put x or y or z accel values into buf
void LSM303DTR_getAccelAxis(uint8_t *buf, uint8_t xyz);

//read who_am_i value into buf, expecting a 0x49
uint8_t LSM303DTR_getWhoAmI(void);

//powers down the LSM303DLHC
void LSM303DTR_sleep(void);

#endif //LSM303DLCH_H
