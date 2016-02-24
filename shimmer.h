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
 * @author Weibo Pan
 * @date January, 2016
 */


#ifndef SHIMMER_H
#define SHIMMER_H

//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER      3     //Represents shimmer3
#define FW_IDENTIFIER   8     //Two byte firmware identifier number
#define FW_VER_MAJOR    0     //Major version number: 0-65535
#define FW_VER_MINOR    0     //Minor version number: 0-255
#define FW_VER_REL      28     //Release candidate version number: 0-255


// Packet Types
#define  BT_DATA_PACKET          0X00
#define  BT_RSP_ACK              0XFF
#define  BT_GET_FW_VERSION       0X01
#define  BT_RSP_FW_VERSION       0X02
#define  BT_GET_STATUS           0X03
#define  BT_RSP_STATUS           0X04
#define  BT_SET_CONFIG_ALL       0X05
#define  BT_GET_CONFIG_ALL       0X06
#define  BT_RSP_CONFIG_ALL       0X07
#define  BT_SET_CONFIG_DEFAULT   0X08
#define  BT_SET_DATA_EMG         0X09
#define  BT_GET_DATA_EMG         0X0A
#define  BT_RSP_DATA_EMG         0X0B
#define  BT_SET_DATA_ACCEL       0X0C
#define  BT_GET_DATA_ACCEL       0X0D
#define  BT_RSP_DATA_ACCEL       0X0E
#define  BT_SET_DATA_GYRO        0X0F
#define  BT_GET_DATA_GYRO        0X10
#define  BT_RSP_DATA_GYRO        0X11
#define  BT_SET_DATA_SG          0X12
#define  BT_GET_DATA_SG          0X13
#define  BT_RSP_DATA_SG          0X14
#define  BT_SET_ACCEL_RANGE      0X15
#define  BT_GET_ACCEL_RANGE      0X16
#define  BT_RSP_ACCEL_RANGE      0X17
#define  BT_SET_GYRO_RANGE       0X18
#define  BT_GET_GYRO_RANGE       0X19
#define  BT_RSP_GYRO_RANGE       0X1A
#define  BT_SET_AUTOOFF_EN       0X1B
#define  BT_GET_AUTOOFF_EN       0X1C
#define  BT_RSP_AUTOOFF_EN       0X1D
#define  BT_SET_STANDBY_EN       0X1E
#define  BT_GET_STANDBY_EN       0X1F
#define  BT_RSP_STANDBY_EN       0X20
#define  BT_SET_EMG_CONFIG       0X21
#define  BT_GET_EMG_CONFIG       0X22
#define  BT_RSP_EMG_CONFIG       0X23
#define  BT_SET_INFOMEM          0X24
#define  BT_GET_INFOMEM          0X25
#define  BT_RSP_INFOMEM          0X26
#define  BT_START_SENSING        0X27
#define  BT_STOP_SENSING         0X28
#define  BT_SET_INFOMEM_P        0X29
#define  BT_SYNC_CMD             0x2a
//#define  BT_GET_STATUS           0x2b
//#define  BT_RSP_STATUS           0x2c
#define  BT_SET_SG_BITSHIFT      0x2d
#define  BT_GET_SG_BITSHIFT      0x2e
#define  BT_RSP_SG_BITSHIFT      0x2f
#define  BT_LED4_TOGGLE          0x30
#define  BT_LED4_ON              0x31
#define  BT_LED4_OFF             0x32
#define  BT_SET_SG_GAIN          0x33
#define  BT_GET_SG_GAIN          0x34
#define  BT_RSP_SG_GAIN          0x35
#define  BT_LED3_TOGGLE          0x36
#define  BT_LED3_ON              0x37
#define  BT_LED3_OFF             0x38
#define  BT_LED3_SET_USERCTRL    0x39
#define  BT_SET_SG_REGS          0x3A
#define  BT_GET_SG_REGS          0x3B
#define  BT_RSP_SG_REGS          0x3C
#define  BT_SET_A1_CFG           0x3D
#define  BT_GET_A1_CFG           0x3E
#define  BT_RSP_A1_CFG           0x3F

#define NV_NUM_RWMEM_BYTES                512

//define data structure
#define DATA_PACKET              1
#define DATA_ID_STAMP            2
#define DATA_EMG1                4
#define DATA_EMG2                7
#define DATA_ACCEL1_X            10
#define DATA_ACCEL1_Y            12
#define DATA_ACCEL1_Z            14
#define DATA_ACCEL2_X            16
#define DATA_ACCEL2_Y            18
#define DATA_ACCEL2_Z            20
#define DATA_GYRO_X              22
#define DATA_GYRO_Y              24
#define DATA_GYRO_Z              26
#define DATA_STRAIN_GAUGE_YL     28
#define DATA_STRAIN_GAUGE_ZL     30
#define DATA_STRAIN_GAUGE_YR     32
#define DATA_STRAIN_GAUGE_ZR     34

//==============================================
//==============================================
// new infomem for movotec
#define NV_CONFIG_0              0
#define NV_CONFIG_1              1
#define NV_CONFIG_2              2
#define NV_EMG_1_CONFIG1         3
#define NV_EMG_1_CONFIG2         4
#define NV_EMG_1_LOFF            5
#define NV_EMG_1_CH1SET          6
#define NV_EMG_1_CH2SET          7
#define NV_EMG_1_RLD_SENS        8
#define NV_EMG_1_LOFF_SENS       9
#define NV_EMG_1_LOFF_STAT       10
#define NV_EMG_1_RESP1           11
#define NV_EMG_1_RESP2           12
#define NV_SG_REG0               13//page 39, ads1220 datasheet
#define NV_SG_REG1               14
#define NV_SG_REG2               15
#define NV_SG_REG3               16
#define NV_A1_CFG                17

#define NV_CONFIG_LEN            (NV_A1_CFG+1)



//#define NV_CONFIG_0
#define CFG_ACCEL_DATA           0x03
#define CFG_GYRO_DATA            0x0c
#define CFG_STRAIN_DATA          0x70
#define CFG_AVAILABLE            0x80  // once configured, this bit must be 0
//#define NV_CONFIG_1
#define CFG_AUTO_TURNOFF_EN      0x01
#define CFG_AUTO_STANDBY_EN      0x02
#define CFG_ACCEL_RANGE          0x0c
#define CFG_GYRO_RANGE           0x30
#define CFG_EMG_DATA             0xC0
//#define NV_CONFIG_2
#define CFG_SG_BITTOSHIFT        0x78
#define CFG_SG_GAIN              0x07

#define PSAD_FREQ    1024
#define PSAD_PERIOD  32768/PSAD_FREQ

//==============================================
//==============================================

//SENSORS0
#define SENSOR_A_ACCEL           0x80
#define SENSOR_MPU9150_GYRO      0x40
#define SENSOR_LSM303DLHC_MAG    0x20
#define SENSOR_EXG1_24BIT        0x10
#define SENSOR_EXG2_24BIT        0x08
#define SENSOR_GSR               0x04
#define SENSOR_EXT_A7            0x02
#define SENSOR_EXT_A6            0x01
//SENSORS1
#define SENSOR_BRIDGE_AMP        0x80     //higher priority than SENSOR_INT_A13 and SENSOR_INT_A14
#define SENSOR_VBATT             0x20
#define SENSOR_LSM303DLHC_ACCEL  0x10
#define SENSOR_EXT_A15           0x08
#define SENSOR_INT_A1            0x04
#define SENSOR_INT_A12           0x02
#define SENSOR_INT_A13           0x01
//SENORS2
#define SENSOR_INT_A14           0x80
#define SENSOR_MPU9150_ACCEL     0x40
#define SENSOR_MPU9150_MAG       0x20
#define SENSOR_EXG1_16BIT        0x10
#define SENSOR_EXG2_16BIT        0x08
#define SENSOR_BMP180_PRESSURE   0x04


#define MAX_COMMAND_ARG_SIZE     133   //maximum number of arguments for any command sent
                                       //(daughter card mem write)
#define RESPONSE_PACKET_SIZE     133   //biggest possibly required  (daughter card mem read + 1 byte for ack)
#define MAX_NUM_CHANNELS         28    //3xanalogAccel + 3xdigiGyro + 3xdigiMag + 
                                       //3xLSM303DLHCAccel + 3xMPU9150Accel + 3xMPU9150MAG +
                                       //BMP180TEMP + BMP180PRESS + batteryVoltage + 
                                       //3xexternalADC + 4xinternalADC
#define DATA_PACKET_SIZE         40    //36 necessary. use 40 in developing
#define DATA_PACKET_BUF_SIZE     50




//Config byte masks
//Config Byte0
#define LSM303DLHC_ACCEL_SAMPLING_RATE          0xF0
#define LSM303DLHC_ACCEL_RANGE                  0x0C
#define LSM303DLHC_ACCEL_LOW_POWER_MODE         0x02
#define LSM303DLHC_ACCEL_HIGH_RESOLUTION_MODE   0x01
//Config Byte1
//MPU9150_SAMPLING_RATE                         0xFF
//Config Byte2
#define LSM303DLHC_MAG_GAIN                     0xE0
#define LSM303DLHC_MAG_SAMPLING_RATE            0x1C
#define MPU9150_GYRO_RANGE                      0x03
//Config Byte3
#define MPU9150_ACCEL_RANGE                     0xC0
#define BMP180_PRESSURE_RESOLUTION              0x30
#define GSR_RANGE                               0x0E
#define INT_EXP_POWER_ENABLE                    0x01


//ADC initialisation mask
#define MASK_A_ACCEL 	0x0001
#define MASK_VBATT   	0x0002
#define MASK_EXT_A7  	0x0004
#define MASK_EXT_A6  	0x0008
#define MASK_EXT_A15 	0x0010
#define MASK_INT_A1  	0x0020
#define MASK_INT_A12 	0x0040
#define MASK_INT_A13 	0x0080
#define MASK_INT_A14 	0x0100
#define MASK_GSR     	0x0020   //uses ADC1
#define MASK_BRIDGE_AMP	0x0180   //uses ADC13 and ADC14


//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G     0x00
#define ACCEL_4G     0x01
#define ACCEL_8G     0x02
#define ACCEL_16G    0x03

//MPU9150 Gyro range
#define MPU9150_GYRO_250DPS         0x00 //+/-250 dps
#define MPU9150_GYRO_500DPS         0x01 //+/-500 dps
#define MPU9150_GYRO_1000DPS        0x02 //+/-1000 dps
#define MPU9150_GYRO_2000DPS        0x03 //+/-2000 dps


#define SYNC_STRING_LEN 11

#define ACLK_1S      32768    //1s for ACLK (running at 32768Hz)
#define ACLK_2S      65536    //2s for ACLK (running at 32768Hz)
#define ACLK_100ms   3278     //approx 100ms for ACLK (running at 32768Hz)

#endif
