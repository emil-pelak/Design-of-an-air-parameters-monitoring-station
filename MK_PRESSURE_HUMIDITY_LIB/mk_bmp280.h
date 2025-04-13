/*
 * mk_bmp280.h
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

#ifndef MK_BMP280_H_
#define MK_BMP280_H_

#include "mk_pressure_cfg.h"

#if USE_BME_BMP_280 == 1

#define BMPE280_ADDR		0xEC


#define OVER_0x		0x00
#define OVER_1x		0x01
#define OVER_2x		0x02
#define OVER_4x		0x03
#define OVER_8x		0x04
#define OVER_16x	0x05

#define BME280_FORCED_MODE 0x01
#define BME280_NORMAL_MODE 0x03
#define BME280_SLEEP_MODE 0x00

#define BME280_STANDBY_500us	0x00
#define BME280_STANDBY_62500us	0x01
#define BME280_STANDBY_125ms	0x02
#define BME280_STANDBY_250ms	0x03
#define BME280_STANDBY_500ms	0x04
#define BME280_STANDBY_1000ms	0x05
#define BME280_STANDBY_10ms		0x06
#define BME280_STANDBY_20ms		0x07

#define BME280_IIR_OFF	0x00
#define BME280_IIR_2x	0x01
#define BME280_IIR_4x	0x02
#define BME280_IIR_8x	0x03
#define BME280_IIR_16x	0x04

#define BME280_SPI_OFF	0x00
#define BME280_SPI_ON	0x01



// Standby-Time, IIR-Filter, SPI Disable
#define BME280_CONFIG		(BME280_STANDBY_250ms << 5)|(BME280_IIR_8x << 2)|(BME280_SPI_OFF)
// Temperature sensor
#define BME280_TEMP_CONFIG	OVER_16x
// Pressure sensor
#define BME280_PRESS_CONFIG	OVER_16x
// Humitity sensor
#define BME280_HUM_CONFIG	OVER_16x
// Mode
#define BME280_MODE_CONFIG	BME280_NORMAL_MODE




typedef struct {

    uint16_t T1;
    int16_t  T2;
    int16_t  T3;

    uint16_t P1;
    int16_t  P2;
    int16_t  P3;
    int16_t  P4;
    int16_t  P5;
    int16_t  P6;
    int16_t  P7;
    int16_t  P8;
    int16_t  P9;

    uint8_t  H1;
    int16_t  H2;
    uint8_t  H3;
    int16_t  H4;
    int16_t  H5;
    int8_t   H6;

} TCALIBDATA;

enum {

    BME280_REGISTER_T1              = 0x88,
    BME280_REGISTER_T2              = 0x8A,
    BME280_REGISTER_T3              = 0x8C,

    BME280_REGISTER_P1              = 0x8E,
    BME280_REGISTER_P2              = 0x90,
    BME280_REGISTER_P3              = 0x92,
    BME280_REGISTER_P4              = 0x94,
    BME280_REGISTER_P5              = 0x96,
    BME280_REGISTER_P6              = 0x98,
    BME280_REGISTER_P7              = 0x9A,
    BME280_REGISTER_P8              = 0x9C,
    BME280_REGISTER_P9              = 0x9E,

    BME280_REGISTER_H1              = 0xA1,
    BME280_REGISTER_H2              = 0xE1,
    BME280_REGISTER_H3              = 0xE3,
    BME280_REGISTER_H4              = 0xE4,
    BME280_REGISTER_H5              = 0xE5,
    BME280_REGISTER_H6              = 0xE7,

    BME280_REGISTER_CHIPID             = 0xD0,
    BME280_REGISTER_VERSION            = 0xD1,
    BME280_REGISTER_SOFTRESET          = 0xE0,

    BME280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

    BME280_REGISTER_CONTROL            = 0xF4,
    BME280_REGISTER_CONFIG             = 0xF5,
    BME280_REGISTER_PRESSUREDATA       = 0xF7,
    BME280_REGISTER_TEMPDATA           = 0xFA,

    BME280_REGISTER_CONTROLHUMID       = 0xF2,
    BME280_REGISTER_HUMIDDATA          = 0xFD,

};










extern uint8_t bmpe280_init( void );
extern uint8_t bmpe280_read_temp( int8_t * at_int, int8_t * at_fract );
extern uint32_t bmpe280_read_pressure( uint16_t * hp_int, uint8_t * hp_fract );
extern uint8_t bmpe280_read_humidity( uint8_t * hm_int, uint8_t * hm_fract );


#endif


#endif /* MK_BMP280_H_ */
