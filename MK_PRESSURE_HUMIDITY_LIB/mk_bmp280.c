/*
 * mk_bmp280.c
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

#if USE_BME_BMP_280 == 1

#include "mk_i2c.h"
#include "mk_bmp280.h"


static int32_t tmp_temperature;
static TCALIBDATA bmpe_calib;



static uint8_t bmpe280_read_byte( uint8_t addr ) {

    uint8_t value;

    i2c__start();
    i2c_write(BMPE280_ADDR);
    i2c_write(addr);
    i2c__start();
    i2c_write(BMPE280_ADDR+1);

    value = i2c_read( NACK );

    i2c__stop();

    return value;
}

static uint16_t bmpe280_read_word( uint8_t addr ) {

    uint16_t value;

    i2c__start();
    i2c_write(BMPE280_ADDR);
    i2c_write(addr);
    i2c__start();
    i2c_write(BMPE280_ADDR+1);

    value = i2c_read( ACK );
    value <<= 8;
    value |= i2c_read( NACK );

    i2c__stop();

    return value;
}

static uint32_t bmpe280_read_3bytes( uint8_t addr ) {

    uint32_t value;
    i2c__start();
    i2c_write(BMPE280_ADDR);
    i2c_write(addr);
    i2c__start();
    i2c_write(BMPE280_ADDR+1);
    value = i2c_read( ACK );

    value <<= 8;
    value |= i2c_read( ACK );
    value <<= 8;
    value |= i2c_read( NACK );

    i2c__stop();

    return value;
}




static uint16_t bmpe280_read_uint16( uint8_t reg ) {

    uint16_t temp = bmpe280_read_word( reg );
    return (temp >> 8) | (temp << 8);
}

static int16_t bmpe280_read_int16( uint8_t reg ) {

    return (int16_t)bmpe280_read_uint16( reg );
}


static void bme280_readCoefficients( void ) {

    bmpe_calib.T1 = bmpe280_read_uint16( BME280_REGISTER_T1 );
    bmpe_calib.T2 = bmpe280_read_int16( BME280_REGISTER_T2 );
    bmpe_calib.T3 = bmpe280_read_int16( BME280_REGISTER_T3 );

    bmpe_calib.P1 = bmpe280_read_uint16( BME280_REGISTER_P1 );
    bmpe_calib.P2 = bmpe280_read_int16( BME280_REGISTER_P2 );
    bmpe_calib.P3 = bmpe280_read_int16( BME280_REGISTER_P3 );
    bmpe_calib.P4 = bmpe280_read_int16( BME280_REGISTER_P4 );
    bmpe_calib.P5 = bmpe280_read_int16( BME280_REGISTER_P5 );
    bmpe_calib.P6 = bmpe280_read_int16( BME280_REGISTER_P6 );
    bmpe_calib.P7 = bmpe280_read_int16( BME280_REGISTER_P7 );
    bmpe_calib.P8 = bmpe280_read_int16( BME280_REGISTER_P8 );
    bmpe_calib.P9 = bmpe280_read_int16( BME280_REGISTER_P9 );

    if( bmpe280_read_byte( BME280_REGISTER_CHIPID ) == 0x60 ) {
        // it's a BME280
        bmpe_calib.H1 = bmpe280_read_byte( BME280_REGISTER_H1 );
        bmpe_calib.H2 = bmpe280_read_int16( BME280_REGISTER_H2 );
        bmpe_calib.H3 = bmpe280_read_byte( BME280_REGISTER_H3 );
        bmpe_calib.H4 = (bmpe280_read_byte( BME280_REGISTER_H4 ) << 4) | (bmpe280_read_byte( BME280_REGISTER_H4+1 ) & 0xF);
        bmpe_calib.H5 = (bmpe280_read_byte( BME280_REGISTER_H5+1 ) << 4) | (bmpe280_read_byte( BME280_REGISTER_H5 ) >> 4);
        bmpe_calib.H6 = (int8_t)bmpe280_read_byte( BME280_REGISTER_H6 );
    }
}


uint8_t bmpe280_init( void ) {

    uint8_t res = st_none;

	i2c__start();
	res = !i2c_write( BMPE280_ADDR );
	i2c__stop();

	if( res ) return 0xFF;

    switch( bmpe280_read_byte( BME280_REGISTER_CHIPID ) ) {

        case 0x60:
			// BME280 connected
			res = st_bme280;// 0x00;

			i2c__start();
			i2c_write( BMPE280_ADDR );
			i2c_write( BME280_REGISTER_SOFTRESET );
			i2c__stop();

			_delay_ms(10);

			i2c__start();
			i2c_write( BMPE280_ADDR );

			i2c_write( BME280_REGISTER_CONTROLHUMID );
			i2c_write( BME280_HUM_CONFIG );
        break;

        case 0x58:
			// BMP280 connected
			res = st_bmp280;// 0x01;

			i2c__start();
			i2c_write( BMPE280_ADDR );


			i2c_write( BME280_REGISTER_SOFTRESET );
			i2c_write( 0xB6 );
			i2c__stop();

			_delay_ms(10);

			i2c__start();
			i2c_write( BMPE280_ADDR );
        break;

        default:
        // ERROR - bad chip-id
        return st_none; //0xff;
    }

    i2c_write( BME280_REGISTER_CONFIG );
    i2c_write( BME280_CONFIG );

    i2c_write( BME280_REGISTER_CONTROL );
    i2c_write( (BME280_TEMP_CONFIG << 5)|(BME280_PRESS_CONFIG << 2)|(BME280_MODE_CONFIG) );

    i2c__stop();

    _delay_ms(10);

    bme280_readCoefficients();

    return res;
}


uint8_t bmpe280_read_temp( int8_t * at_int, int8_t * at_fract  ) {



    int32_t adc_T = bmpe280_read_3bytes( BME280_REGISTER_TEMPDATA );

    if (adc_T == 0x800000) // ERROR if temperature measurement was disabled
        return 0xFF;

    int32_t var1, var2;

    adc_T >>= 4;

    var1  = ((((adc_T>>3) - ((int32_t)bmpe_calib.T1 <<1))) *
             ((int32_t)bmpe_calib.T2)) >> 11;

    var2  = (((((adc_T>>4) - ((int32_t)bmpe_calib.T1)) *
               ((adc_T>>4) - ((int32_t)bmpe_calib.T1))) >> 12) *
             ((int32_t)bmpe_calib.T3)) >> 14;

    tmp_temperature = var1 + var2;

    adc_T = (tmp_temperature * 5 + 128) >> 8;
	
    if( at_int && at_fract ) {
    	*at_int = adc_T / 100;
    	*at_fract = adc_T - (*at_int*100);	
    }
	return 0;
}



uint32_t bmpe280_read_pressure( uint16_t * hp_int, uint8_t * hp_fract ) {


    int64_t var1, var2, p;

    bmpe280_read_temp( 0, 0 );

    int32_t adc_P = bmpe280_read_3bytes( BME280_REGISTER_PRESSUREDATA );
    if (adc_P == 0x800000) // ERROR if pressure measurement was disabled
        return 0xFF;
    adc_P >>= 4;

    var1 = ((int64_t)tmp_temperature) - 128000ul;
    var2 = var1 * var1 * (int64_t)bmpe_calib.P6;
    var2 = var2 + ((var1*(int64_t)bmpe_calib.P5)<<17);
    var2 = var2 + (((int64_t)bmpe_calib.P4)<<35);
    var1 = ((var1 * var1 * (int64_t)bmpe_calib.P3)>>8) +
    ((var1 * (int64_t)bmpe_calib.P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)bmpe_calib.P1)>>33;

    if (var1 == 0) {
        return 0xff; // ERROR avoid exception caused by division by zero
    }
    p = 1048576ul - adc_P;
    p = (((p<<31) - var2)*3125ul) / var1;
    var1 = (((int64_t)bmpe_calib.P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)bmpe_calib.P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmpe_calib.P7)<<4);

    p /= 256ul;

    if( hp_int && hp_fract ) {
    	*hp_int = p/100ul;
    	*hp_fract = p - (*hp_int*100);
    }

    return (p<<8);	// OK

}


uint8_t bmpe280_read_humidity( uint8_t * hm_int, uint8_t * hm_fract ) {


    if( bmpe280_read_byte( BME280_REGISTER_CHIPID ) != 0x60 ) // ERROR if it is BMP280 - not BME280
        return 0xFF;

    bmpe280_read_temp( 0, 0 );

    int32_t adc_H = bmpe280_read_word( BME280_REGISTER_HUMIDDATA );
    if( adc_H == 0x8000 ) // ERROR if humidity measurement was disabled
        return 0xFF;

    int32_t v_x1_u32r;

    v_x1_u32r = (tmp_temperature - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)bmpe_calib.H4) << 20) -
                    (((int32_t)bmpe_calib.H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)bmpe_calib.H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)bmpe_calib.H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)bmpe_calib.H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)bmpe_calib.H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;


    uint32_t h = (v_x1_u32r >> 12);
    h *= 100;
    h /= 1024;

    if( hm_int && hm_fract ) {
    	*hm_int = h/100ul;
    	*hm_fract = h - (*hm_int*100);
    }

    return 0; // OK
}





#endif






