/*
main.c
Created on: 2023-10-01
Author: Emil Pelak
*/


/***********************INCLUSION OF STANDARD LIBRARIES***********************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/***********************INCLUSION OF CUSTOM LIBRARIES***********************/
#include "1Wire/ds18x20.h"	// library for DS18B20 (1-Wire)
#include "7_LED/7_led.h"	// library for LED display
#include "I2C_TWI/i2c_twi.h"	// library for RTC (I2C)
#include "IR_DECODE/ir_decode.h"	// library for IR pilot (RC5)
//#include "LCD/lcd44780.h"		// switched off from compilation - LCD display is not used in this project
#include "MK_PRESSURE_HUMIDITY_LIB/mk_i2c.h"	// library for BME280 (I2C)
#include "MK_PRESSURE_HUMIDITY_LIB/mk_pressure_cfg.h"	// library for BME280 (reading parameters via I2C)
#include "MKUART/mkuart.h"	// library for UART communication
#include "OLED/lcd.h"	// library for OLED display
#include "OLED/font.h"	// library for OLED display


/***********************PREPROCESSORS DEFINITIONS***********************/
#define DS1337_ADDR 0xD0	// address of the RTC chip on I2C

#define LED1 (1<<PC6)	// standard LED definition

#define LED1_DDR DDRC

#define LED1_ON PORTC &= ~LED1
#define LED1_OFF PORTC |= LED1
#define LED1_TOG PORTC ^= LED1


#define KL_DDRA DDRA		// buttons definition
#define KL_DDRC DDRC

#define KL_PORTA PORTA
#define KL_PORTC PORTC

#define KEY1 (1<<PA4)
#define KEY2 (1<<PA5)
#define KEY3 (1<<PA6)
#define KEY4 (1<<PA7)
#define KEY5 (1<<PC7)


/***********************DEFINITIONS OF GLOBAL VARIABLES AND TYPES***********************/
uint8_t key1, key2, key3, key4, key5;		// variables for SuperDebounce function

volatile static uint8_t led_display = 1;	// variable responsible for switching on/off LED display
volatile static uint8_t led_blink;	// variable responsible for switching on/off LED blinking

volatile static uint8_t command_cnt;	// variable responsible for counting cycles of IR receiver events

volatile static char uart_tx;	// data from UART (RX data)
volatile static uint8_t uart_help = 1;	// variable responsible for UART help
volatile static uint8_t uart_logs_data;	// variable responsible for UART logging data

volatile static uint16_t Timer1, Timer2, Timer3, Timer4;	// software timers 100Hz 

uint8_t RTC_modify = 0;	// RTC_modify == 0 -> don't modify time and date with every flashing / reset of device,  RTC_modify == 1 -> - modify time and date with every flashing / reset of device
enum {SS, MIN, HH, DAY, DD, MM, YY};	// enumerated type for seconds, minutes, hours, day of the week, and date which is used to point indexes in the table/buffer (saving data to RTC chip)
uint8_t buffer[7];	// table/buffer (saving data to RTC chip)
uint8_t ss, min, hh, day, dd, mm, yy;	// variables to read data from RTC chip
static volatile uint8_t ss_memory; // variable to save difference for time

volatile static uint8_t int0_flag=1;	// flag changed in the interrupt and checked in the main loop (RTC event)

uint8_t subzero, cel, cel_fract_bits;	// variables for DS18B20
uint8_t sensors_cnt;	// quantity of sensors on the 1-Wire bus
volatile static uint8_t cycle;	// variable to start measuring and read the temperature from DS18B20

int8_t t_int, t_fract;	// variables to read and display the value of temperature
volatile static uint8_t t_fract_flag = 0; // information about minus temperature after the decimal point for UART

uint16_t hp_int; // variables to read and display the value of pressure
uint8_t hp_fract;

uint8_t hm_int, hm_fract;	// variables to read and display the value of humidity


/***********************FUNCTIONS DECLARATION***********************/
void SuperDebounce(uint8_t * key_state, volatile uint8_t *KPIN, uint8_t key_mask, uint16_t rep_time,	// SuperDebounce function for a single button operation
uint16_t rep_wait, void (*push_proc)(void), void (*rep_proc)(void) );

void oled_display_temp_DS18B20(uint8_t y);	// displaying a temperature from DS18B20 sensor on OLED display

void oled_day_of_the_week(uint8_t day);	// convert a number of day to day of the week for OLED display
void uart_day_of_the_week(uint8_t day);	// convert a number of day to day of the week for UART

void uart_control(void); // function for receiving data from UART and device control

void ir_control(void); // function for receiving data from RC5 and device control

void uart_data_logging(void); // function for sending measurements from sensors via UART

void key1_press(void);	// function assigned to a button 1 (short press)
void key1_repeat(void);	// function assigned to a button 1 (long press)

void key2_press(void);	// function assigned to a button 2 (short press)
void key2_repeat(void);	// function assigned to a button 2 (long press)

void key3_press(void);	// function assigned to a button 3 (short press)
void key3_repeat(void);	// function assigned to a button 2 (long press)

void key4_press(void);	// function assigned to a button 4 (short press)
void key4_repeat(void);	// function assigned to a button 4 (long press)

void key5_press(void);	// function assigned to a button 5 (short press)
void key5_repeat(void);	// function assigned to a button 5 (long press)

uint8_t dec2bcd(uint8_t dec);	// convert decimal to BCD
uint8_t bcd2dec(uint8_t bcd);	// convert BCD to decimal


/***********************MAIN FUNCTION OF PROGRAM***********************/
int main(void) 
{

/*------------------INITIALIZATION OF PINS DIRECTION------------------*/
	KL_DDRA &= ~(KEY1|KEY2|KEY3|KEY4);	// the direction register of buttons as an input
	KL_DDRC &= ~(KEY5);
	
	KL_PORTA |= KEY1|KEY2|KEY3|KEY4;	// switching on a pull-up to VCC
	KL_PORTC |= KEY5;	// switching on a pull-up to VCC

	
	LED1_DDR |= LED1;	// the direction register of LED as an output
	LED1_OFF;	// switch off LED


/*------------------INTERRUPT INITIALIZATION------------------*/

	/* INT0 interrupt */
	MCUCR |= (1<<ISC01);	// falling edge triggering
	GICR |= (1<<INT0);		// interrupt unlock
	PORTD |= (1<<PD2);		// pulling the INT0 pin to VCC

	/* Timer2 – interrupt initialization - 10 ms (100Hz) */
	TCCR2 	|= (1<<WGM21);			// CTC work mode
	TCCR2 	|= (1<<CS22)|(1<<CS21)|(1<<CS20);	// prescaler = 1024
	OCR2 	= 108;					// interrupt comparison every 10ms (100Hz)
	TIMSK 	= (1<<OCIE2);			// interrupt unlock CompareMatch


/*------------------INITIALIZATION OF PROGRAM MODULES------------------*/
	sensors_cnt = search_sensors();		// check how many DS18xxx is present on 1-Wire bus
	DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); // start measuring of temperature for DS18B20
	
	i2cSetBitrate(100);	// set the speed for I2C (kHz)
	
	mk_press_hum_init();	// BME280 sensor initialization
	
	lcd_oled_init(LCD_DISP_ON);	// initialization of OLED and turn on
	lcd_set_contrast(100);	// set a contrast for OLED display
	lcd_charMode(1);	// set a font size
	
	d_led_init();	// LED multiplexing initialization
	
	ir_init();	// initialization of the RC5 infrared receiver 

	USART_Init(__UBRR);	// UART initialization
	
	sei();	// global interrupt unlock

/*------------------SEND AND SET TIME, DATA FOR RTC CHIP------------------*/
	if (RTC_modify == 1)	// RTC_modify == 0 -> don't modify time and date with every flashing / reset of device,  
		{					// RTC_modify == 1 -> - modify time and date with every flashing / reset of device
			buffer[HH] = dec2bcd(22);	// hours
			buffer[MIN] = dec2bcd(44);	// minutes
			buffer[SS] = dec2bcd(00);	// seconds
			buffer[DAY] = dec2bcd(1);	// day of the week (1-7)
			buffer[DD] = dec2bcd(19);	// day
			buffer[MM] = dec2bcd(2);	// month
			buffer[YY] = dec2bcd(24);	// year
	
			TWI_write_buf(DS1337_ADDR, 0x00, 7, buffer); // writing data to RAM of RTC chip (7 bytes from buffer 
		}												 // to RAM from address 0x00)
	

 
	
/*--------------------------------------------------------------MAIN LOOP---------------------------------------------------------------*/
	while(1) 
		{
			
			/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF BUTTONS^^^^^^^^^^^^^^^^*/
			SuperDebounce(&key1, &PINA, KEY1, 2, 15, key1_press, key1_repeat );	//function assigned for KEY1 - OLED display ON / OFF
			SuperDebounce(&key2, &PINA, KEY2, 2, 15, key2_press, key2_repeat );	//function assigned for KEY2 - LED display ON / OFF
 			SuperDebounce(&key3, &PINA, KEY3, 2, 15, key3_press, key3_repeat );	//function assigned for KEY3 - LED blinks / doesn't blink
 			SuperDebounce(&key4, &PINA, KEY4, 2, 15, key4_press, key4_repeat );	//function assigned for KEY4 - DATA send / DATA don't send
 			SuperDebounce(&key5, &PINC, KEY5, 2, 15, key5_press, key5_repeat );	//function assigned for KEY5 - OLED, LED display, LED blinks, DATA send / DATA don't send



			/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF UART^^^^^^^^^^^^^^^^*/
			uart_control();	// receiving data from UART and device control
		
		
		
			/**^^^^^^^^^^^^^^^^INFRARED RECEIVER EVENT^^^^^^^^^^^^^^^^*/
			ir_control();	// receiving data from RC5 and device control



			/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF SOFTWARE TIMER2^^^^^^^^^^^^^^^^*/
			if(!Timer2)		// task assigned for software Timer2
				{
					Timer2=25;
					if(led_blink) LED1_TOG;
				}


			
			/*----------------------RTC SYSTEM EVENT----------------------*/
			if (int0_flag) 
				{
					
					/*^^^^^^^^^^^^^^^^READ AND SET TIME, DATA FOR RTC CHIP^^^^^^^^^^^^^^^^*/
					TWI_read_buf( DS1337_ADDR, 0x00, 7, buffer);	// reading data from RAM of RTC chip 
					hh = bcd2dec( buffer[HH] );						//(7 bytes from RAM from address 0x00 to buffer)
					min = bcd2dec( buffer[MIN] );
					ss = bcd2dec( buffer[SS] );
					day = bcd2dec( buffer[DAY] );
					dd = bcd2dec( buffer[DD] );
					mm = bcd2dec( buffer[MM] );
					yy = bcd2dec( buffer[YY] );
			
			
			
					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF CLOCK ON LED DISPLAY^^^^^^^^^^^^^^^^*/
					if (led_display == 1) // show a clock on LED display
					{
						if  (ss %2 != 0)	// if seconds %2 != 0, seconds on LED display are off
							{
								cy1 = buffer[HH]>>4;	// displaying a time on OLED display (only hours)
								cy2 = buffer[HH]&0x0f;
								cy3=cy4=10;		// display nothing
							}
									
						else if  (ss %2 == 0) // if seconds %2 == 0, seconds on LED display are on
							{
								cy1 = buffer[HH]>>4;	// displaying a time on OLED display (hours and minutes)
								cy2 = buffer[HH]&0x0f;
								cy3 = buffer[MIN]>>4;
								cy4 = buffer[MIN]&0x0f;
							}
					}

					if (led_display == 0)	// doesn't show a clock on LED display
						{
							cy1=cy2=cy3=cy4=10;		// display nothing	
						}
				
				
					
					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF OLED DISPLAY - TIME^^^^^^^^^^^^^^^^*/
					lcd_gotoxy(0,0);	// displaying a time on OLED display
					lcd_puts_p(PSTR("Time: "));
					if( hh < 10 ) lcd_puts_p(PSTR("0"));
					oled_int(hh);
					lcd_puts_p(PSTR(":"));
					if( min < 10 ) lcd_puts_p(PSTR("0"));
					oled_int(min);
					lcd_puts_p(PSTR(":"));
					if( ss < 10 ) lcd_puts_p(PSTR("0"));
					oled_int(ss);
					
					
					
					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF OLED DISPLAY - DATE^^^^^^^^^^^^^^^^*/	
					lcd_gotoxy(0,1);	// displaying a date on OLED display
					lcd_puts_p(PSTR("Date: "));
					oled_day_of_the_week(day);
					lcd_puts_p(PSTR("/"));
					oled_int(dd);
					lcd_puts_p(PSTR("/"));
					oled_int(mm);
					lcd_puts_p(PSTR("/"));
					if (yy != 0) lcd_puts_p(PSTR("20"));
					oled_int(yy);
					if(dd < 10 || mm < 10) lcd_puts_p(PSTR("  "));
				
		

					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF DS18B20 SENSOR^^^^^^^^^^^^^^^^*/
					
					// read the temperature from the sensor every second, if detected. Display the temperature when the sensor is detected
					if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[0], &subzero, &cel, &cel_fract_bits) ) oled_display_temp_DS18B20(2);
							
					DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL ); // start measuring of temperature for DS18B20
									
			
			
					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF BME280 SENSOR^^^^^^^^^^^^^^^^*/
					lcd_gotoxy(0,3);	// display data from BME280 sensor
					lcd_puts_p(PSTR("----BME280 sensor----"));						
						
						
					
					/*^^^^^^^^^^^^^^^^DISPLAY TEMPERATURE FROM BME280 SENSOR^^^^^^^^^^^^^^^^*/
					if( 0 == mkp_read_temp(st_bme280, &t_int, &t_fract) )	// Reading and displaying a temperature in °C 
						{													// (-40°C to 85°C) from BME280 sensor on OLED display
										
							if( t_int == 0 && t_fract < 0 )
								{
									t_fract = -t_fract;
									lcd_gotoxy(0,4);
									lcd_puts_p(PSTR("Temperature:  -"));
									t_fract_flag = 1;
								}
							else if( t_int < 0 && t_fract < 0 )
								{
							 		t_fract = -t_fract;
									lcd_gotoxy(0,4);
									lcd_puts_p(PSTR("Temperature:  "));
								}
							else
							{
								lcd_gotoxy(0,4);
								lcd_puts_p(PSTR("Temperature:    "));
								t_fract_flag = 0;
							}
	
							
							if (t_int <= -10 ) lcd_gotoxy(13,4);
							else if (t_int > -10 && t_int < 0) lcd_gotoxy(14,4);
							else if (t_int >= 0 && t_int < 10) lcd_gotoxy(15,4);
							else if (t_int >= 10 ) lcd_gotoxy(14,4);
							
							oled_int( t_int );
							lcd_puts_p(PSTR("."));
							if( t_fract < 10 ) lcd_puts_p(PSTR("0"));
							oled_int( t_fract );
							lcd_puts_p(PSTR("°C"));
						}
						

						/*^^^^^^^^^^^^^^^^DISPLAY PRESSURE FROM BME280 SENSOR^^^^^^^^^^^^^^^^*/
						if( !mkp_read_pressure( st_bmp280, &hp_int, &hp_fract ) )	// Reading and displaying a pressure in hPa 
						{															// (300 hPa to 1100 hPa) from BME280 sensor on OLED display
							lcd_gotoxy(0,5);
							lcd_puts_p(PSTR("Pressure:   "));
							
							if(hp_int < 1000) lcd_gotoxy(11,5);
							else lcd_gotoxy(10,5);
							
							oled_long_int(hp_int);
							lcd_puts_p(PSTR("."));
							if( hp_fract < 10 ) lcd_puts_p(PSTR("0") );
							oled_int( hp_fract );
							lcd_puts_p(PSTR(" hPa"));
						}


						/*^^^^^^^^^^^^^^^^DISPLAY HUMIDITY FROM BME280 SENSOR^^^^^^^^^^^^^^^^*/
						if( !mkp_read_humidity( st_bme280, &hm_int, &hm_fract ) )	// Reading and displaying a humidity in % 
						{															// (10% to 100% RH) from BME280 sensor on OLED display
							lcd_gotoxy(0,6);
							lcd_puts_p(PSTR("Humidity: "));
							
							if(hm_int < 10) lcd_gotoxy(15,6);
							else lcd_gotoxy(14,6);
							
							oled_int(hm_int);
							lcd_puts_p(PSTR("."));
							if( hm_int < 100 && hm_fract < 10) lcd_puts_p(PSTR("0") );
							oled_int(hm_fract);
							lcd_puts_p(PSTR(" %") );
						}
						
					
					
					if (uart_logs_data == 0) lcd_gotoxy(0,7), lcd_puts_p(PSTR("DATA don't send"));	// information for user about logging data with measurements
										
					lcd_display();	// display data on OLED display
				
						
						
					/*^^^^^^^^^^^^^^^^FUNCTIONALITY OF UART DATA LOGGING^^^^^^^^^^^^^^^^*/
					 if (ss_memory != ss && uart_logs_data == 1) uart_data_logging();	// send measurements from sensors via UART every second when logging has been activated								
					

					int0_flag = 0; // reset a flag for RTC event
		
		
					ss_memory = ss; // save actual value of seconds
				}

		}
}




/***********************INT0 interrupt service routine***********************/
ISR( INT0_vect )
{
	int0_flag = 1;
}


/***********************Timer2 interrupt service routine***********************/
ISR(TIMER2_COMP_vect)
{
	uint16_t x;

	x = Timer1;		/* 100Hz Timer1 */
	if (x) Timer1 = --x;
	
	x = Timer2;		/* 100Hz Timer2 */
	if (x) Timer2 = --x;
	
	x = Timer3;		/* 100Hz Timer3 */
	if (x) Timer3 = --x;
	
	x = Timer4;		/* 100Hz Timer4 */
	if (x) Timer4 = --x;
}


/************* SuperDebounce function for single key operation ********************
AUTHOR: Miros³aw Kardaœ

  ADVANTAGES:
- does not cause any slowdown
- has a REPEAT function (repeating the action for a longer pressed button)
- you can assign different actions for REPEAT mode and single click
- only one action can be assigned, then we pass 0 (NULL) instead of the second one
 
REQUIREMENTS:
- Software timer created based on Hardware timer (100Hz interrupt)
 
-INPUT PARAMETERS:
- *key_state - pointer to a variable in RAM (1 byte)
- *KPIN - PINx name of the port on which the key is located, e.g. PINB
- key_mask - key mask, e.g.: (1<<PB3)
- rep_time - repetition time of the rep_proc function in REPEAT mode
- rep_wait - waiting time before switching to REPEAT mode
- *push_proc - pointer to its own function called once when the button is released
- *rep_proc - pointer to its own function executed in REPEAT mode
**********************************************************************************/
void SuperDebounce(uint8_t * key_state, volatile uint8_t *KPIN,
		uint8_t key_mask, uint16_t rep_time, uint16_t rep_wait,
		void (*push_proc)(void), void (*rep_proc)(void) ) {

	enum {idle, debounce, go_rep, wait_rep, rep};

	if(!rep_time) rep_time=20;
	if(!rep_wait) rep_wait=150;

	uint8_t key_press = !(*KPIN & key_mask);

	if( key_press && !*key_state ) {
		*key_state = debounce;
		Timer1 = 15;
	} else
	if( *key_state  ) {

		if( key_press && debounce==*key_state && !Timer1 ) {
			*key_state = 2;
			Timer1=5;
		} else
		if( !key_press && *key_state>1 && *key_state<4 ) {
			if(push_proc) push_proc();						/* KEY_UP */
			*key_state=idle;
		} else
		if( key_press && go_rep==*key_state && !Timer1 ) {
			*key_state = wait_rep;
			Timer1=rep_wait;
		} else
		if( key_press && wait_rep==*key_state && !Timer1 ) {
			*key_state = rep;
		} else
		if( key_press && rep==*key_state && !Timer1 ) {
			Timer1 = rep_time;
			if(rep_proc) rep_proc();						/* KEY_REP */
		}
	}
	if( *key_state>=3 && !key_press ) *key_state = idle;
}


/**********************Displaying a temperature from DS18B20 sensor on OLED display***********************/
void oled_display_temp_DS18B20(uint8_t y)
{
	lcd_gotoxy(0, y);	// set the line for displaying
	lcd_puts_p(PSTR("DS18B20 sen.T: "));	// display a name of sensor
	
	if(subzero && cel < 10) lcd_gotoxy(14, y), lcd_puts_p(PSTR(" -"));	// if subzero = 1 and cel < 10, display correctly sign "-"
	else if (subzero && cel >= 10) lcd_gotoxy(14, y), lcd_puts_p(PSTR("-"));	// if subzero = 1 and cel >= 10, display correctly sign "-"
	
	if(!subzero && cel < 10) lcd_gotoxy(14, y), lcd_puts_p(PSTR("  "));	// set the cursor correctly when the temperature is smaller or greater than 10 or equal to 10
	else if (!subzero && cel >= 10) lcd_gotoxy(14, y), lcd_puts_p(PSTR(" "));
	
	oled_int(cel);	// display decimals of temperature
	lcd_puts_p(PSTR("."));	// display a dot
	oled_int(cel_fract_bits);	// display the number after the decimal point
	lcd_puts_p(PSTR("°C"));		// display unit sign (C - degrees Celsius)
}


/**********************Convert a number of day to day of the week for OLED display***********************/
void oled_day_of_the_week(uint8_t day) 
{
	switch (day) 
		{
			case 1:
			lcd_puts_p(PSTR("Mo"));
			break;
			case 2:
			lcd_puts_p(PSTR("Tu"));
			break;
			case 3:
			lcd_puts_p(PSTR("We"));
			break;
			case 4:
			lcd_puts_p(PSTR("Th"));
			break;
			case 5:
			lcd_puts_p(PSTR("Fr"));
			break;
			case 6:
			lcd_puts_p(PSTR("Sa"));
			break;
			case 7:
			lcd_puts_p(PSTR("Su"));
			break;
		}
}


/**********************Convert a number of day to day of the week for UART***********************/
void uart_day_of_the_week(uint8_t day)
{
	switch (day)
		{
			case 1:
			uart_puts("Mo");
			break;
			case 2:
			uart_puts("Tu");
			break;
			case 3:
			uart_puts("We");
			break;
			case 4:
			uart_puts("Th");
			break;
			case 5:
			uart_puts("Fr");
			break;
			case 6:
			uart_puts("Sa");
			break;
			case 7:
			uart_puts("Su");
			break;
		}
}


/**********************Function for receiving data from UART and device control***********************/
void uart_control(void)
{
	if (uart_help) uart_puts("Type 'h' or 'H' and press ENTER for help :)\r\n");	// Help information for user
	uart_help = 0;	// reset a variable responsible for Help information for user	
	
	uart_tx = uart_getc(); // receiving data from UART
	
	if(uart_tx == 'h' || uart_tx == 'H')	// Device commands for user
	{
		uart_puts("\r\nWelcome to User Help. Enter the numbers below to activate functions and press Enter\r\n");
		
		uart_puts("'0' - OLED display ON\r\n");
		uart_puts("'1' - OLED display OFF\r\n");
		
		uart_puts("'2' - LED display ON\r\n");
		uart_puts("'3' - LED display OFF\r\n");
		
		uart_puts("'4' - LED blinks\r\n");
		uart_puts("'5' - LED doesn't blink\r\n");
		
		uart_puts("'6' - DATA send\r\n");
		uart_puts("'7' - DATA don't send\r\n");
		
		uart_puts("'8' - OLED display ON, LED display ON, LED blinks, DATA send\r\n");
		uart_puts("'9' - OLED display OFF, LED display OFF, LED doesn't blink, DATA don't send\r\n");
	}
	
	
	/*^^^^^^^^^^^^^^^^THIS PART OF CODE REALIZE THE SAME FUNCTIONALITY LIKE BUTTONS FUNCTIONS^^^^^^^^^^^^^^^^*/
	if(uart_tx == '0') lcd_sleep(0), uart_puts("UART cmd = 0 -> "), uart_puts("OLED display ON\r\n");
	else if(uart_tx == '1') lcd_sleep(1), uart_puts("UART cmd = 1 -> "), uart_puts("OLED display OFF\r\n");
	else if(uart_tx == '2') led_display = 1, uart_puts("UART cmd = 2 -> "), uart_puts("LED display ON\r\n");
	else if(uart_tx == '3') led_display = 0, uart_puts("UART cmd = 3 -> "), uart_puts("LED display OFF\r\n");
	else if(uart_tx == '4') led_blink = 1, uart_puts("UART cmd = 4 -> "), uart_puts("LED blinks\r\n");
	else if(uart_tx == '5') led_blink = 0, uart_puts("UART cmd = 5 -> "), LED1_OFF, uart_puts("LED doesn't blink\r\n");
	else if(uart_tx == '6') 
		{
			uart_logs_data = 1;
			lcd_gotoxy(0,7);
			uart_puts("UART cmd = 6 -> ");
			lcd_puts_p(PSTR("DATA send      "));
			uart_puts("DATA send\r\n");
		}
	else if(uart_tx == '7')
		{
			uart_logs_data = 0;
			lcd_gotoxy(0,7);
			uart_puts("UART cmd = 7 -> ");
			lcd_puts_p(PSTR("DATA don't send"));
			uart_puts("DATA don't send\r\n");
		} 
	else if(uart_tx == '8') 
		{
			lcd_sleep(0);
			led_display = 1;
			led_blink = 1;
			uart_logs_data = 1;
			lcd_gotoxy(0,7); 
			uart_puts("UART cmd = 8 -> ");
			lcd_puts_p(PSTR("DATA send      "));
			uart_puts("OLED display ON, LED display ON, LED blinks, DATA send\r\n");
		}
	else if(uart_tx == '9') 
		{
			lcd_sleep(1);
			led_display = 0;
			led_blink = 0;
			LED1_OFF;
			uart_logs_data = 0;
			lcd_gotoxy(0,7);
			uart_puts("UART cmd = 9 -> ");
			lcd_puts_p(PSTR("DATA don't send"));
			uart_puts("OLED display OFF, LED display OFF, LED doesn't blink, DATA don't send\r\n");
		}
}


/**********************Function for receiving data from RC5 and device control***********************/
void ir_control(void)
{
	if(!Ir_key_press_flag) lcd_gotoxy(16,7), lcd_puts_p(PSTR("     "));		// empty place on OLED display when IR control is not used
	
	if(Ir_key_press_flag)
	{
		
		if( !address && 0 <= command && command <= 9 )	// inform a user via terminal and OLED display about IR control
			{
				lcd_gotoxy(16,7);
				lcd_puts_p(PSTR("IRcon"));
				if (command_cnt == 0) uart_puts("RC5 cmd = "), uart_putint(command, 10), uart_puts(" -> "); // send an info only one time
			}
		
		
		/*^^^^^^^^^^^^^^^^THIS PART OF CODE REALIZE THE SAME FUNCTIONALITY LIKE BUTTONS FUNCTIONS^^^^^^^^^^^^^^^^*/
		if( !address && command == 0 ) 
			{
				lcd_sleep(0);
				if (command_cnt == 0) uart_puts("OLED display ON\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 1 ) 
			{
				lcd_sleep(1); 
				if (command_cnt == 0) uart_puts("OLED display OFF\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 2 ) 
			{
				led_display = 1; 
				if (command_cnt == 0) uart_puts("LED display ON\r\n"); // send info only once time in the cycle
			
			}
		else if( !address && command == 3 ) 
			{
				led_display = 0;
				if (command_cnt == 0) uart_puts("LED display OFF\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 4 ) 
			{
				led_blink = 1; 
				if (command_cnt == 0) uart_puts("LED blinks\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 5 ) 
			{
				led_blink = 0;
				LED1_OFF;
				if (command_cnt == 0) uart_puts("LED doesn't blink\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 6 )
			{
				uart_logs_data = 1;
				lcd_gotoxy(0,7);
				lcd_puts_p(PSTR("DATA send      "));
				if (command_cnt == 0)  uart_puts("DATA send\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 7 )
			{
				uart_logs_data = 0;
				lcd_gotoxy(0,7);
				lcd_puts_p(PSTR("DATA don't send"));
				if (command_cnt == 0) uart_puts("DATA don't send\r\n"); // send info only once time in the cycle
			}
		else if( !address && command == 8 ) 
			{
				lcd_sleep(0);
				led_display = 1;
				led_blink = 1;
				uart_logs_data = 1;
				lcd_gotoxy(0,7);
				lcd_puts_p(PSTR("DATA send      "));
				if (command_cnt == 0) uart_puts("OLED display ON, LED display ON, LED blinks, DATA send\r\n"); // send info only once time in the cycle
			}	
		else if( !address && command == 9 ) 
			{
				lcd_sleep(1);
				led_display = 0;
				led_blink = 0;
				LED1_OFF;
				uart_logs_data = 0;
				lcd_gotoxy(0,7);
				lcd_puts_p(PSTR("DATA don't send"));
				if (command_cnt == 0) uart_puts("OLED display OFF, LED display OFF, LED doesn't blink, DATA don't send\r\n"); // send info only once time in the cycle
			}
		
		
		Ir_key_press_flag = 0; // reset a variable when the value == 0 
		if ( ++command_cnt == 2 ) command_cnt = 0; // reset a variable when the value == 2 
		
		command=0xff;
		address=0xff;
	}	
}		


/**********************Function for sending measurements from sensors via UART***********************/
void uart_data_logging(void)
{
	if (hh < 10) uart_puts("0");	// send a time
	uart_putint(hh, 10);
	uart_puts(":");
	if (min < 10) uart_puts("0");
	uart_putint(min, 10);
	uart_puts(":");
	if (ss < 10) uart_puts("0");
	uart_putint(ss, 10);
	uart_puts(" / ");

	uart_day_of_the_week(day);	// send a date
	uart_puts("/");
	uart_putint(dd, 10);
	uart_puts("/");
	uart_putint(mm, 10);
	uart_puts("/");
	if (yy != 0) uart_puts("20");
	uart_putint(yy, 10);

	uart_puts(" / DS18B20: "); // send a data from DS18B20 sensor
	if(subzero) uart_puts("-");
	uart_putint(cel, 10);
	uart_puts(".");
	uart_putint(cel_fract_bits, 10);
	uart_puts("°C");

	uart_puts(" / BME280: ");	// send a data from BME280 sensor
	if(t_fract_flag) uart_puts("-");
	uart_putint(t_int, 10);
	uart_puts(".");
	if(t_fract < 10) uart_puts("0");
	uart_putint(t_fract, 10);
	uart_puts("°C / ");
	uart_putint(hp_int, 10);
	uart_puts(".");
	if(hp_fract < 10) uart_puts("0");
	uart_putint(hp_fract, 10);
	uart_puts("hPa / ");
	uart_putint(hm_int, 10);
	uart_puts(".");
	if (hm_int < 100 && hm_fract < 10) uart_puts("0");
	uart_putint(hm_fract, 10);
	uart_puts("%\r\n");
}

		
/***********************Function for button 1 (short press) - OLED display ON***********************/
void key1_press(void) 
{
	lcd_sleep(0);
	uart_puts("OLED display ON\r\n");
}


/***********************Function for button 1 (long press) - OLED display OFF***********************/
void key1_repeat(void) 
{
	lcd_sleep(1);
	uart_puts("OLED display OFF\r\n");
}


/***********************Function for button 2 (short press) - LED display ON***********************/
void key2_press(void) 
{
	led_display = 1;
	uart_puts("LED display ON\r\n");
}


/***********************Function for button 2 (long press) - LED display OFF***********************/
void key2_repeat(void) 
{
	led_display = 0;
	uart_puts("LED display OFF\r\n");
}


/***********************Function for button 3 (short press) - LED blinks***********************/
void key3_press(void)
{
	led_blink = 1;
	uart_puts("LED blinks\r\n");
}


/***********************Function for button 3 (long press) - LED doesn't blink***********************/
void key3_repeat(void)
{
	led_blink = 0;
	LED1_OFF;
	uart_puts("LED doesn't blink\r\n");
}


/***********************Function for button 4 (short press) - DATA send***********************/
void key4_press(void)
{
	uart_logs_data = 1;
	lcd_gotoxy(0,7);
 	lcd_puts_p(PSTR("DATA send      "));
	uart_puts("DATA send\r\n");
}


/***********************Function for button 4 (long press) - DATA don't send***********************/
void key4_repeat(void)
{
	uart_logs_data = 0;
 	lcd_gotoxy(0,7);
 	lcd_puts_p(PSTR("DATA don't send"));
	uart_puts("DATA don't send\r\n");
}


/***********************Function for button 5 (short press) - OLED&LED display ON, LED blinks, DATA send***********************/
void key5_press(void)
{
	lcd_sleep(0);
	led_display = 1;
	led_blink = 1;
	uart_logs_data = 1;
 	lcd_gotoxy(0,7);
 	lcd_puts_p(PSTR("DATA send      "));
	uart_puts("OLED display ON, LED display ON, LED blinks, DATA send\r\n");
}


/***********************Function for button 5 (long press) - OLED&LED display OFF, LED doesn't blink, DATA don't send***********************/
void key5_repeat(void)
{
	lcd_sleep(1);
	led_display = 0;
	led_blink = 0;
	LED1_OFF;
	uart_logs_data = 0;
 	lcd_gotoxy(0,7);
	lcd_puts_p(PSTR("DATA don't send"));
	uart_puts("OLED display OFF, LED display OFF, LED doesn't blink, DATA don't send\r\n");
}


/***********************Convert decimal to BCD***********************/
uint8_t dec2bcd(uint8_t dec)
{
	return ((dec / 10)<<4) | (dec % 10);
}


/***********************Convert BCD to decimal***********************/
uint8_t bcd2dec(uint8_t bcd)
{
	return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}