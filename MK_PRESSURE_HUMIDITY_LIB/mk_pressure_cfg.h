/*
 * mk_pressure.h
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>




#ifndef MK_PRESSURE_LIB_MK_PRESSURE_H_
#define MK_PRESSURE_LIB_MK_PRESSURE_H_

/* ............ KONFIGURACJA BIBLIOTEKI - WYBÓR RODZAJU CZUJNIKA ........................... */

#define	USE_BMP_085_180			0	// ciœnienie, temperatura
#define USE_BME_BMP_280			1	// BME280-ciœnienie, temperatura, wilgotnoœæ, BMP280-ciœnienie, temperatura
#define USE_HIH6131				0	// temperatura, wilgotnoœæ





/*................................ koniec konfiguracji biblioteki ........................... */




enum { st_none=0, st_bmp180=1, st_bmp280=2, st_bme280=4, st_hih6131=8 };




extern uint8_t mk_press_hum_init( void );

extern uint8_t mkp_read_temp( uint8_t sensor_type, int8_t * at_int, int8_t * at_fract );
extern uint8_t mkp_read_pressure( uint8_t sensor_type, uint16_t * hp_int, uint8_t * hp_fract );
extern uint8_t mkp_read_humidity( uint8_t sensor_type, uint8_t * hm_int, uint8_t * hm_fract );




#endif /* MK_PRESSURE_LIB_MK_PRESSURE_H_ */
