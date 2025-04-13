/*
 * MkBMP180.h
 *
 * 	sample code for CPU's: m16/32 and m644(P)/m1284(P)
 * 	F_CPU: 1 - 20 MHz
 *
 *  Created on: 22 kwiecieñ 2021
 *      Author: Miros³aw Kardaœ
 *
 *		www.atnel.pl           www.sklep.atnel.pl
 *      Pressure, Temperature & humidity LIB
 *      ver: 1.0
 *
 *      for:
 *      BMP085/BMP180	- ciœnienie, temperatura,     -
 *      BMP280			- ciœnienie, temperatura,     -
 *      BME280			- ciœnienie, temperatura, wilgotnoœæ
 *      HIH6131 		-     -      temperatura, wilgotnoœæ 				(najlepszy czujnik wilgotnoœci !)
 */

#ifndef BMP180_MKBMP180_H_
#define BMP180_MKBMP180_H_

#include "mk_pressure_cfg.h"

#if USE_BMP_085_180 == 1

#define BMP180_ADDR 	0xEE        // Adres I2C


#define AC1_HIGH        0xAA
#define AC1_LOW         0xAB
#define AC2_HIGH        0xAC
#define AC2_LOW         0xAD
#define AC3_HIGH        0xAE
#define AC3_LOW         0xAF
#define AC4_HIGH        0xB0
#define AC4_LOW         0xB1
#define AC5_HIGH        0xB2
#define AC5_LOW         0xB3
#define AC6_HIGH        0xB4
#define AC6_LOW         0xB5
#define B1_HIGH         0xB6
#define B1_LOW          0xB7
#define B2_HIGH         0xB8
#define B2_LOW          0xB9
#define MB_HIGH         0xBA
#define MB_LOW          0xBB
#define MC_HIGH         0xBC
#define MC_LOW          0xBD
#define MD_HIGH         0xBE
#define MD_LOW          0xBF


#define OSS_REG         0xF4


#define OSS             0x01

#define OSS_TEMP_VALUE  0x2E
#define TEMP_CONV_TIME  4500


#define OSS_PRES_VALUE  0x34
#if OSS == 0x00
    #define PRES_CONV_TIME  4500
#elif OSS == 0x01
    #define PRES_CONV_TIME  7500
#elif OSS == 0x02
    #define PRES_CONV_TIME  13500
#elif OSS == 0x03
    #define PRES_CONV_TIME  25550
#endif


#define DATA_HIGH       	0xF6
#define DATA_LOW        	0xF7
#define DATA_XLOW       	0xF8



#define SEA_LEVEL_PRESSURE 	101325




typedef struct {
    // Variables from EEPROM BMP085 / BMP180
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    int16_t MB;
    int16_t MC;
    int16_t MD;

    // Coefficients that calculates
    int32_t B3;
    uint32_t B4;
    int32_t B5;
    int32_t B6;
    uint32_t B7;
} TBMPCALIB;

















extern uint8_t bmp180_init(void);

extern uint8_t bmp180_read_temp( int8_t * at_int, uint8_t * at_fract );
extern uint8_t bmp180_read_pressure( uint16_t * hp_int, uint8_t * hp_fract );


extern float bmp180_get_height( void );


#endif		// #if USE_BMP_085_180 == 1


#endif /* BMP180_MKBMP180_H_ */
