/*
 * mk_pressure.c
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
 *
 */


#include "mk_pressure_cfg.h"

#include "mk_i2c.h"

#if USE_BME_BMP_280 == 1
	#include "mk_bmp280.h"
#endif

#if USE_BMP_085_180 == 1
	#include "mk_bmp180.h"
#endif

#if USE_HIH6131 == 1
	#include "mk_hih6131.h"
#endif



uint8_t mk_press_hum_init( void ) {

	uint8_t res = st_none;

#if USE_BME_BMP_280 == 1
	res |= bmpe280_init();
#endif

#if USE_BMP_085_180 == 1
	res |= bmp180_init();
#endif

#if USE_HIH6131 == 1
	res |= hih6131_init();
#endif

	return res;

}


uint8_t mkp_read_temp( uint8_t sensor_type, int8_t * at_int, int8_t * at_fract ) {

#if USE_BME_BMP_280 == 1
	if( (sensor_type & st_bmp280) || (sensor_type & st_bme280) ) {
		return bmpe280_read_temp( at_int, at_fract );
	}
#endif

#if USE_BMP_085_180 == 1
	if( (sensor_type & st_bmp180) ) {
		return bmp180_read_temp( at_int, at_fract );
	}
#endif

#if USE_HIH6131 == 1
	if( (sensor_type & st_hih6131) ) {
		return hih6131_read_temp( at_int, at_fract );
	}
#endif

	return 0xFF;	// error
}



uint8_t mkp_read_pressure( uint8_t sensor_type, uint16_t * hp_int, uint8_t * hp_fract ) {

#if USE_BME_BMP_280 == 1
	if( (sensor_type & st_bmp280) || (sensor_type & st_bme280) ) {
		return bmpe280_read_pressure( hp_int, hp_fract );
	}
#endif

#if USE_BMP_085_180 == 1
	if( (sensor_type & st_bmp180) ) {
		return bmp180_read_pressure( hp_int, hp_fract );
	}
#endif

#if USE_HIH6131 == 1
	if( (sensor_type & st_hih6131) ) {
		return 0xFF;	// error
	}
#endif

	return 0xFF;	// error

}



uint8_t mkp_read_humidity( uint8_t sensor_type, uint8_t * hm_int, uint8_t * hm_fract ) {


#if USE_BME_BMP_280 == 1
	if( sensor_type & st_bme280 ) {
		return bmpe280_read_humidity( hm_int, hm_fract );
	}
#endif

#if USE_BMP_085_180 == 1
	if( (sensor_type & st_bmp180) ) {
		return 0xFF;	// error
	}
#endif

#if USE_HIH6131 == 1
	if( (sensor_type & st_hih6131) ) {
		return hih6131_read_humidity( hm_int, hm_fract );
	}
#endif

	return 0xFF;	// error
}











