/*
 * mk_hih6131.c
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


#include "mk_pressure_cfg.h"


#if USE_HIH6131 == 1

#include "mk_i2c.h"

#include "mk_hih6131.h"




static int8_t hih_temp1;
static uint8_t hih_temp2;

static uint8_t hih_hm1;
static uint8_t hih_hm2;



void hih_write_cmd( uint8_t cmd, uint16_t data ) {

	i2c__start();
	i2c_write( HIH6131_ADDR );
	i2c_write( cmd );
	i2c_write( data >> 8 );
	i2c_write( cmd );
	i2c__stop();


}

uint8_t hih_measure_request( void ) {

	i2c__start();
	uint8_t res = !i2c_write( HIH6131_ADDR );
	i2c__stop();

	if( !res ) return st_hih6131;
	else return st_none;
}







uint8_t hih_read_humidity_temp( void ) {


	uint32_t tmp, raw_hm, raw_temperature;

	static uint8_t ft;

	if( !ft ) {
		hih_measure_request();
		_delay_ms(30);
		ft=1;
	}


	i2c__start();
	i2c_write( HIH6131_ADDR + 1 );

	tmp = i2c_read( ACK );
	tmp = (tmp & 0x3F) << 8;

	tmp |= i2c_read( ACK );
	raw_hm = tmp;

	tmp = i2c_read( ACK );
	tmp = tmp << 8;

	tmp |= i2c_read( NACK );
	tmp = tmp >> 2;

	raw_temperature = tmp;

	i2c__stop();


	int32_t t = ( ( (((uint64_t)raw_temperature*10000l)+8191l )/(16384l) ) * 165l ) - 400000ul;
	hih_temp1 = (t)/10000ul;
	hih_temp2 = (t % 10000ul)/100;


	int32_t h = ( ( (((uint64_t)raw_hm*10000l)+8191l )/(16384l) ) * 100l );
	hih_hm1 = (h)/10000ul;
	hih_hm2 = (h % 10000ul)/100;

	if( 99 == hih_hm1 && 99 == hih_hm2 ) {
		hih_hm1 = 100;
		hih_hm2 = 0;
	}

	hih_measure_request();

	return 0;
}


uint8_t hih6131_init( void ) {


	return hih_measure_request();

}



uint8_t hih6131_read_temp( int8_t * at_int, uint8_t * at_fract ) {

	hih_read_humidity_temp();

    if( at_int && at_fract ) {
    	*at_int = hih_temp1;
    	*at_fract = hih_temp2;
    	return 0;
    }

    return 1;
}



uint8_t hih6131_read_humidity( uint8_t * hm_int, uint8_t * hm_fract ) {

	hih_read_humidity_temp();

    if( hm_int && hm_fract ) {
    	*hm_int = hih_hm1;
    	*hm_fract = hih_hm2;
    	return 0;
    }

	return 1;
}





#endif	// #if USE_HIH6131 == 1
