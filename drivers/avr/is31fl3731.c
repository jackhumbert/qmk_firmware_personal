/* Copyright 2017 Jason Williams
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "is31fl3731.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "TWIlib.h"
#include "progmem.h"

// This is a 7-bit address, that gets left-shifted and bit 0
// set to 0 for write, 1 for read (as per I2C protocol)
// The address will vary depending on your wiring:
// 0b1110100 AD <-> GND
// 0b1110111 AD <-> VCC
// 0b1110101 AD <-> SCL
// 0b1110110 AD <-> SDA
#define ISSI_ADDR_DEFAULT 0x74

#define ISSI_REG_CONFIG  0x00
#define ISSI_REG_CONFIG_PICTUREMODE 0x00
#define ISSI_REG_CONFIG_AUTOPLAYMODE 0x08
#define ISSI_REG_CONFIG_AUDIOPLAYMODE 0x18

#define ISSI_CONF_PICTUREMODE 0x00
#define ISSI_CONF_AUTOFRAMEMODE 0x04
#define ISSI_CONF_AUDIOMODE 0x08

#define ISSI_REG_PICTUREFRAME  0x01

#define ISSI_REG_SHUTDOWN 0x0A
#define ISSI_REG_AUDIOSYNC 0x06

#define ISSI_COMMANDREGISTER 0xFD
#define ISSI_BANK_FUNCTIONREG 0x0B    // helpfully called 'page nine'

// Transfer buffer for TWITransmitData()
uint8_t g_twi_transfer_buffer[TXMAXBUFLEN];

// These buffers match the IS31FL3731 PWM registers 0x24-0xB3.
// Storing them like this is optimal for I2C transfers to the registers.
// We could optimize this and take out the unused registers from these
// buffers and the transfers in IS31FL3731_write_pwm_buffer() but it's
// probably not worth the extra complexity.
uint8_t g_pwm_buffer[DRIVER_COUNT][144];
bool g_pwm_buffer_update_required = false;

uint8_t g_led_control_registers[DRIVER_COUNT][18] = { { 0 }, { 0 } };
bool g_led_control_registers_update_required = false;


typedef struct
{
	uint8_t red_register;
	uint8_t red_bit;
	uint8_t green_register;
	uint8_t green_bit;
	uint8_t blue_register;
	uint8_t blue_bit;
} led_control_bitmask;

// This is the bit pattern in the LED control registers
// (for matrix A, add one to register for matrix B)
//
//  reg -  b7  b6  b5  b4  b3  b2  b1  b0
// 0x00 - R08,R07,R06,R05,R04,R03,R02,R01
// 0x02 - G08,G07,G06,G05,G04,G03,G02,R00
// 0x04 - B08,B07,B06,B05,B04,B03,G01,G00
// 0x06 -  - , - , - , - , - ,B02,B01,B00
// 0x08 -  - , - , - , - , - , - , - , -
// 0x0A - B17,B16,B15, - , - , - , - , -
// 0x0C - G17,G16,B14,B13,B12,B11,B10,B09
// 0x0E - R17,G15,G14,G13,G12,G11,G10,G09
// 0x10 - R16,R15,R14,R13,R12,R11,R10,R09

const led_control_bitmask g_led_control_bitmask[18] =
{
	{ 0x02, 0, 0x04, 0, 0x06, 0 }, // R00,G00,B00
	{ 0x00, 0, 0x04, 1, 0x06, 1 }, // R01,G01,B01
	{ 0x00, 1, 0x02, 1, 0x06, 2 }, // R02,G02,B02
	{ 0x00, 2, 0x02, 2, 0x04, 2 }, // R03,G03,B03
	{ 0x00, 3, 0x02, 3, 0x04, 3 }, // R04,G04,B04
	{ 0x00, 4, 0x02, 4, 0x04, 4 }, // R05,G05,B05
	{ 0x00, 5, 0x02, 5, 0x04, 5 }, // R06,G06,B06
	{ 0x00, 6, 0x02, 6, 0x04, 6 }, // R07,G07,B07
	{ 0x00, 7, 0x02, 7, 0x04, 7 }, // R08,G08,B08

	{ 0x10, 0, 0x0E, 0, 0x0C, 0 }, // R09,G09,B09
	{ 0x10, 1, 0x0E, 1, 0x0C, 1 }, // R10,G10,B10
	{ 0x10, 2, 0x0E, 2, 0x0C, 2 }, // R11,G11,B11
	{ 0x10, 3, 0x0E, 3, 0x0C, 3 }, // R12,G12,B12
	{ 0x10, 4, 0x0E, 4, 0x0C, 4 }, // R13,G13,B13
	{ 0x10, 5, 0x0E, 5, 0x0C, 5 }, // R14,G14,B14
	{ 0x10, 6, 0x0E, 6, 0x0A, 5 }, // R15,G15,B15
	{ 0x10, 7, 0x0C, 6, 0x0A, 6 }, // R16,G16,B16
	{ 0x0E, 7, 0x0C, 7, 0x0A, 7 }, // R17,G17,B17
};

const uint8_t g_map_control_index_to_register[2][18][3] PROGMEM = {
 {
 	{0x34, 0x44, 0x54}, // 00
 	{0x24, 0x45, 0x55}, // 01
 	{0x25, 0x35, 0x56}, // 02
 	{0x26, 0x36, 0x46}, // 03
 	{0x27, 0x37, 0x47}, // 04
 	{0x28, 0x38, 0x48}, // 05
 	{0x29, 0x39, 0x49}, // 06
 	{0x2a, 0x3a, 0x4a}, // 07
 	{0x2b, 0x3b, 0x4b}, // 08

 	{0xa4, 0x94, 0x84}, // 09
 	{0xa5, 0x95, 0x85}, // 10
 	{0xa6, 0x96, 0x86}, // 11
 	{0xa7, 0x97, 0x87}, // 12
 	{0xa8, 0x98, 0x88}, // 13
 	{0xa9, 0x99, 0x89}, // 14
 	{0xaa, 0x9a, 0x79}, // 15
 	{0xab, 0x8a, 0x7a}, // 16
 	{0x9b, 0x8b, 0x7b}  // 17
 }, {
 	{0x34 + 8, 0x44 + 8, 0x54 + 8}, // 00
 	{0x24 + 8, 0x45 + 8, 0x55 + 8}, // 01
 	{0x25 + 8, 0x35 + 8, 0x56 + 8}, // 02
 	{0x26 + 8, 0x36 + 8, 0x46 + 8}, // 03
 	{0x27 + 8, 0x37 + 8, 0x47 + 8}, // 04
 	{0x28 + 8, 0x38 + 8, 0x48 + 8}, // 05
 	{0x29 + 8, 0x39 + 8, 0x49 + 8}, // 06
 	{0x2a + 8, 0x3a + 8, 0x4a + 8}, // 07
 	{0x2b + 8, 0x3b + 8, 0x4b + 8}, // 08

 	{0xa4 + 8, 0x94 + 8, 0x84 + 8}, // 09
 	{0xa5 + 8, 0x95 + 8, 0x85 + 8}, // 10
 	{0xa6 + 8, 0x96 + 8, 0x86 + 8}, // 11
 	{0xa7 + 8, 0x97 + 8, 0x87 + 8}, // 12
 	{0xa8 + 8, 0x98 + 8, 0x88 + 8}, // 13
 	{0xa9 + 8, 0x99 + 8, 0x89 + 8}, // 14
 	{0xaa + 8, 0x9a + 8, 0x79 + 8}, // 15
 	{0xab + 8, 0x8a + 8, 0x7a + 8}, // 16
 	{0x9b + 8, 0x8b + 8, 0x7b + 8}  // 17
}};

void IS31FL3731_write_register( uint8_t addr, uint8_t reg, uint8_t data )
{
	g_twi_transfer_buffer[0] = (addr << 1) | 0x00;
	g_twi_transfer_buffer[1] = reg;
	g_twi_transfer_buffer[2] = data;

	// Set the error code to have no relevant information
	TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
	// Continuously attempt to transmit data until a successful transmission occurs
	//while ( TWIInfo.errorCode != 0xFF )
	//{
		TWITransmitData( g_twi_transfer_buffer, 3, 0 );
	//}
}

void IS31FL3731_write_pwm_buffer( uint8_t addr, uint8_t *pwm_buffer )
{
	// assumes bank is already selected

	// transmit PWM registers in 9 transfers of 16 bytes
	// g_twi_transfer_buffer[] is 20 bytes

	// set the I2C address
	g_twi_transfer_buffer[0] = (addr << 1) | 0x00;

	// iterate over the pwm_buffer contents at 16 byte intervals
	for ( int i = 0; i < 144; i += 16 )
	{
		// set the first register, e.g. 0x24, 0x34, 0x44, etc.
		g_twi_transfer_buffer[1] = 0x24 + i;
		// copy the data from i to i+15
		// device will auto-increment register for data after the first byte
		// thus this sets registers 0x24-0x33, 0x34-0x43, etc. in one transfer
		for ( int j = 0; j < 16; j++ )
		{
			g_twi_transfer_buffer[2 + j] = pwm_buffer[i + j];
		}

		// Set the error code to have no relevant information
		TWIInfo.errorCode = TWI_NO_RELEVANT_INFO;
		// Continuously attempt to transmit data until a successful transmission occurs
		while ( TWIInfo.errorCode != 0xFF )
		{
			TWITransmitData( g_twi_transfer_buffer, 16 + 2, 0 );
		}
	}
}

void IS31FL3731_init( uint8_t addr )
{
	// In order to avoid the LEDs being driven with garbage data
	// in the LED driver's PWM registers, first enable software shutdown,
	// then set up the mode and other settings, clear the PWM registers,
	// then disable software shutdown.

	// select "function register" bank
	IS31FL3731_write_register( addr, ISSI_COMMANDREGISTER, ISSI_BANK_FUNCTIONREG );

	// enable software shutdown
	IS31FL3731_write_register( addr, ISSI_REG_SHUTDOWN, 0x00 );
	// this delay was copied from other drivers, might not be needed
	_delay_ms( 10 );

	// picture mode
	IS31FL3731_write_register( addr, ISSI_REG_CONFIG, ISSI_REG_CONFIG_PICTUREMODE );
	// display frame 0
	IS31FL3731_write_register( addr, ISSI_REG_PICTUREFRAME, 0x00 );
	// audio sync off
	IS31FL3731_write_register( addr, ISSI_REG_AUDIOSYNC, 0x00 );

	// select bank 0
	IS31FL3731_write_register( addr, ISSI_COMMANDREGISTER, 0 );

	// turn off all LEDs in the LED control register
	for ( int i = 0x00; i <= 0x11; i++ )
	{
		IS31FL3731_write_register( addr, i, 0x00 );
	}

	// turn off all LEDs in the blink control register (not really needed)
	for ( int i = 0x12; i <= 0x23; i++ )
	{
		IS31FL3731_write_register( addr, i, 0x00 );
	}

	// set PWM on all LEDs to 0
	for ( int i = 0x24; i <= 0xB3; i++ )
	{
		IS31FL3731_write_register( addr, i, 0x00 );
	}

	// select "function register" bank
	IS31FL3731_write_register( addr, ISSI_COMMANDREGISTER, ISSI_BANK_FUNCTIONREG );

	// disable software shutdown
	IS31FL3731_write_register( addr, ISSI_REG_SHUTDOWN, 0x01 );

	// select bank 0 and leave it selected.
	// most usage after initialization is just writing PWM buffers in bank 0
	// as there's not much point in double-buffering
	IS31FL3731_write_register( addr, ISSI_COMMANDREGISTER, 0 );
}


void map_index_to_led( uint8_t index, is31_led *led ) {
	//led = , sizeof(struct is31_led));
	// led->driver = addr->driver;
	// led->matrix = addr->matrix;
	// led->modifier = addr->modifier;
	// led->control_index = addr->control_index;
	// led->matrix_co.raw = addr->matrix_co.raw;
	// led->driver = (pgm_read_byte(addr) >> 6) && 0b11;
	// led->matrix = (pgm_read_byte(addr) >> 4) && 0b1;
	// led->modifier = (pgm_read_byte(addr) >> 3) && 0b1;
	// led->control_index = pgm_read_byte(addr+1);
	// led->matrix_co.raw = pgm_read_byte(addr+2);
}

void IS31FL3731_set_color( int index, uint8_t red, uint8_t green, uint8_t blue )
{
	if ( index >= 0 && index < DRIVER_LED_TOTAL )
	{
		is31_led led = g_is31_leds[index];
		//map_index_to_led(index, &led);

		// Subtract 0x24 to get the second index of g_pwm_buffer
		g_pwm_buffer[led.driver][ pgm_read_byte(&g_map_control_index_to_register[led.matrix][led.control_index][0]) - 0x24] = red;
		g_pwm_buffer[led.driver][ pgm_read_byte(&g_map_control_index_to_register[led.matrix][led.control_index][1]) - 0x24] = green;
		g_pwm_buffer[led.driver][ pgm_read_byte(&g_map_control_index_to_register[led.matrix][led.control_index][2]) - 0x24] = blue;
		g_pwm_buffer_update_required = true;
	}
}

void IS31FL3731_set_color_all( uint8_t red, uint8_t green, uint8_t blue )
{
	for ( int i = 0; i < DRIVER_LED_TOTAL; i++ )
	{
		IS31FL3731_set_color( i, red, green, blue );
	}
}

void IS31FL3731_set_led_control_register( uint8_t index, bool red, bool green, bool blue )
{
	is31_led led = g_is31_leds[index];
	// map_index_to_led(index, &led);

	led_control_bitmask bitmask = g_led_control_bitmask[led.control_index];

	// Matrix A and B registers are interleaved.
	// Add 1 to Matrix A register to get Matrix B register
	if ( red )
	{
		g_led_control_registers[led.driver][bitmask.red_register+led.matrix] |= (1<<bitmask.red_bit);
	}
	else
	{
		g_led_control_registers[led.driver][bitmask.red_register+led.matrix] &= ~(1<<bitmask.red_bit);
	}
	if ( green )
	{
		g_led_control_registers[led.driver][bitmask.green_register+led.matrix] |= (1<<bitmask.green_bit);
	}
	else
	{
		g_led_control_registers[led.driver][bitmask.green_register+led.matrix] &= ~(1<<bitmask.green_bit);
	}
	if ( blue )
	{
		g_led_control_registers[led.driver][bitmask.blue_register+led.matrix] |= (1<<bitmask.blue_bit);
	}
	else
	{
		g_led_control_registers[led.driver][bitmask.blue_register+led.matrix] &= ~(1<<bitmask.blue_bit);
	}

	g_led_control_registers_update_required = true;


}

void IS31FL3731_update_pwm_buffers( uint8_t addr1, uint8_t addr2 )
{
	if ( g_pwm_buffer_update_required )
	{
		IS31FL3731_write_pwm_buffer( addr1, g_pwm_buffer[0] );
		IS31FL3731_write_pwm_buffer( addr2, g_pwm_buffer[1] );
	}
	g_pwm_buffer_update_required = false;
}

void IS31FL3731_update_led_control_registers( uint8_t addr1, uint8_t addr2 )
{
	if ( g_led_control_registers_update_required )
	{
		for ( int i=0; i<18; i++ )
		{
			IS31FL3731_write_register(addr1, i, g_led_control_registers[0][i] );
			IS31FL3731_write_register(addr2, i, g_led_control_registers[1][i] );
		}
	}
}

