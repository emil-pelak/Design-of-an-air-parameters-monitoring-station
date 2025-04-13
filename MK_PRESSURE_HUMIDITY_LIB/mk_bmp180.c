/*
 * MkBMP180.c
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


#if USE_BMP_085_180 == 1

#include "mk_i2c.h"

#include "mk_bmp180.h"





static TBMPCALIB bmp180;





void bmp180_set_reg_pointer(uint8_t reg) {
    i2c__start();
    i2c_write(BMP180_ADDR);
    i2c_write(reg);
    i2c__stop();
}


void bmp180_set_reg(uint8_t reg, uint8_t value) {
    i2c__start();
    i2c_write(BMP180_ADDR);
    i2c_write(reg);
    i2c_write(value);
    i2c__stop();
}


uint8_t bmp180_read_reg(uint8_t reg) {
    bmp180_set_reg_pointer(reg);

    i2c__start();
    i2c_write(BMP180_ADDR+1);
    uint8_t byte = i2c_read(NACK);
    i2c__stop();

    return byte;
}


int32_t bmp180_read_bytes(uint8_t reg, uint8_t bytes) {
    // Move to register
    bmp180_set_reg_pointer(reg);

    uint32_t value = 0;
    for (uint8_t i = 0; i < bytes; i++) {
        uint8_t temp = bmp180_read_reg(reg + i);
        value += temp * pow(2, 8 * (bytes - i - 1));
    }
    return value;
}


TBMPCALIB BMP180_getCalibrationData( void ) {

    bmp180.AC1 = bmp180_read_bytes(AC1_HIGH, 2);
    bmp180.AC2 = bmp180_read_bytes(AC2_HIGH, 2);
    bmp180.AC3 = bmp180_read_bytes(AC3_HIGH, 2);
    bmp180.AC4 = bmp180_read_bytes(AC4_HIGH, 2);
    bmp180.AC5 = bmp180_read_bytes(AC5_HIGH, 2);
    bmp180.AC6 = bmp180_read_bytes(AC6_HIGH, 2);
    bmp180.B1 = bmp180_read_bytes(B1_HIGH, 2);
    bmp180.B2 = bmp180_read_bytes(B2_HIGH, 2);
    bmp180.MB = bmp180_read_bytes(MB_HIGH, 2);
    bmp180.MC = bmp180_read_bytes(MC_HIGH, 2);
    bmp180.MD = bmp180_read_bytes(MD_HIGH, 2);

    return bmp180;
}


int32_t bmp180_read_uncomp_temp( void ) {
    bmp180_set_reg(OSS_REG, OSS_TEMP_VALUE);
    _delay_us(TEMP_CONV_TIME);
    return bmp180_read_bytes(DATA_HIGH, 2);
}


int32_t bmp180_read_uncomp_pressure( void ) {
    bmp180_set_reg(OSS_REG, OSS_PRES_VALUE + (OSS << 6));
    _delay_us(PRES_CONV_TIME);
    return bmp180_read_bytes(DATA_HIGH, 3) >> (8 - OSS);
}


int32_t bmp180_get_temp( void ) {

    uint32_t UT = bmp180_read_uncomp_temp();

    int32_t x1,x2;
    x1 = ( (int32_t)UT -  bmp180.AC6 ) *  bmp180.AC5 >> 15;
    x2 = ( (int32_t)bmp180.MC << 11) / (x1 +  bmp180.MD);
    bmp180.B5 = x1 + x2;

    return ( (bmp180.B5 + 8) >> 4 );
}


uint8_t bmp180_read_temp( int8_t * at_int, uint8_t * at_fract ) {

	int t = bmp180_get_temp();

    if( at_int && at_fract ) {
    	*at_int = t / 10;
    	*at_fract = (t % 10) * 10;
    }

    return 0;
}



int32_t bmp180_get_pressure( void ) {

    bmp180_get_temp();
    int32_t UP = bmp180_read_uncomp_pressure();

    bmp180.B6 = bmp180.B5 - 4000;
    int32_t X1 = (bmp180.B2 * (bmp180.B6 * bmp180.B6 / pow(2, 12))) / pow(2, 11);
    int32_t X2 = bmp180.AC2 * bmp180.B6 / pow(2, 11);
    int32_t X3 = X1 + X2;
    bmp180.B3 = (((bmp180.AC1 * 4 + X3) << OSS) + 2) / 4;
    X1 = bmp180.AC3 * bmp180.B6 / pow(2, 13);
    X2 = (bmp180.B1 * (bmp180.B6 * bmp180.B6 / pow(2, 12))) / pow(2, 16);
    X3 = ((X1 + X2) + 2) / pow(2, 2);
    bmp180.B4 = bmp180.AC4 * (uint32_t)(X3 + 32768) / pow(2, 15);
    bmp180.B7 = ((uint32_t)UP - bmp180.B3) * (50000 >> OSS);
    int32_t pressure;
    if (bmp180.B7 < 0x80000000) {
        pressure = (bmp180.B7 * 2) / bmp180.B4;
    } else {
        pressure = (bmp180.B7 / bmp180.B4) * 2;
    }
    X1 = (pressure / pow(2, 8)) * (pressure / pow(2, 8));
    X1 = (X1 * 3038) / pow(2, 16);
    X2 = (-7357 * pressure) / pow(2, 16);
    pressure = pressure + (X1 + X2 + 3791) / pow(2, 4);
    return pressure;
}


uint8_t bmp180_read_pressure( uint16_t * hp_int, uint8_t * hp_fract ) {

	int32_t p = bmp180_get_pressure();

    if( hp_int && hp_fract ) {
    	*hp_int = p / 100;
    	*hp_fract = (p % 10) * 10;
    }

    return 0;

}



/* **************  do testów  ************************** */
//float bmp180_get_height( void ) {
//    uint32_t pressure = bmp180_get_pressure();
//    return 44330 * (1 - pow((float)pressure / SEA_LEVEL_PRESSURE, (float)1 / 5.255));
//}







uint8_t bmp180_init( void ) {

	i2c__start();
	uint8_t res = !i2c_write( BMP180_ADDR );
	i2c__stop();

	if( !res ) {
		BMP180_getCalibrationData();
		return st_bmp180;
	}

	return st_none;
}







#endif		// #if USE_BMP_085_180 == 1







