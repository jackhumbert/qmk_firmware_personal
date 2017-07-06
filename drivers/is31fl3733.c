/* Copyright 2017 Jack Humbert
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

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include "TWIlib.h"
#include "progmem.h"
#include "is31fl3733.h"

#define ISSI_COMMAND_REGISTER 0xFD
#define ISSI_COMMAND_REGISTER_LOCK 0xFE
	#define ISSI_COMMAND_REGISTER_WRITE_DISABLE 0x00
	#define ISSI_COMMAND_REGISTER_WRITE_ONCE 0xC5

#define ISSI_CONTROL_REGISTER 0x00 // LED Control Register
// 0x00 - 0x17 set on/off state (w)
// 0x18 - 0x2F store open state (r)
// 0x30 - 0x47 store short state (r)

#define ISSI_PWM_REGISTER 0x01 // PWM Register
// 0x00 - 0xBF set PWM duty for LED (w)

//  CSx ...
// G 0x
// R 1x
// B 2x

// G 3x
// R 4x
// B 5x

// G 6x
// R 7x
// B 8x

// G 9x
// R Ax
// B Bx


#define ISSI_ABM_REGISTER 0x02 // Auto Breath Mode Register 
// 0x00 - 0xBF set operating mode of each dot (w)

#define ISSI_FUNCTION_REGISTER 0x03 // Function Register
#define ISSI_CONFIGURATION_REGISTER 0x00 // Configuration Register Configure the operation mode 13 W
	#define ISSI_SYNC_MASTER 0b01000000
	#define ISSI_SYNC_SLAVE  0b10000000
	#define ISSI_OSD_TRIGGER 0b00000100
	#define ISSI_ABM_MODE    0b00000010
	#define ISSI_PWM_MODE    0b00000000
	#define ISSI_NORMAL_OP   0b00000010
	#define ISSI_SSD_MODE    0b00000000
#define ISSI_GLOBAL_CURRENT  0x01 // Global Current Control Register Set the global current 14 W
// 02h Auto Breath Control Register 1 of ABM-1 Set fade in and hold time for breath function of ABM-1 15 W
// 03h Auto Breath Control Register 2 of ABM-1 Set the fade out and off time for breath function of ABM-1 16 W
// 04h Auto Breath Control Register 3 of ABM-1 Set loop characters of ABM-1 17 W
// 05h Auto Breath Control Register 4 of ABM-1 Set loop characters of ABM-1 18 W
// 06h Auto Breath Control Register 1 of ABM-2 Set fade in and hold time for breath function of ABM-1 15 W
// 07h Auto Breath Control Register 2 of ABM-2 Set the fade out and off time for breath function of ABM-1 16 W
// 08h Auto Breath Control Register 3 of ABM-2 Set loop characters of ABM-2 17 W
// 09h Auto Breath Control Register 4 of ABM-2 Set loop characters of ABM-2 18 W
// 0Ah Auto Breath Control Register 1 of ABM-3 Set fade in and hold time for breath function of ABM-1 15 W
// 0Bh Auto Breath Control Register 2 of ABM-3 Set the fade out and off time for breath function of ABM-1 16 W
// 0Ch Auto Breath Control Register 3 of ABM-3 Set loop characters of ABM-3 17 W
// 0Dh Auto Breath Control Register 4 of ABM-3 Set loop characters of ABM-3 18 W
// 0Eh Time Update Register Update the setting of 02h ~ 0Dh registers - W
// 0Fh SWy Pull-Up Resistor Selection Register Set the pull-up resistor for SWy 19 W
// 10h CSx Pull-Down Resistor Selection Register Set the pull-down resistor for CSx 20 W
// 11h Reset Register Reset all register to POR state 


// Transfer buffer for TWITransmitData()
uint8_t g_twi_transfer_buffer[TXMAXBUFLEN];

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

void IS31FL3733_write_register( uint8_t addr, uint8_t reg, uint8_t data )
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

void IS31FL3733_write_pwm_buffer( uint8_t addr, uint8_t *pwm_buffer )
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

void IS31FL3733_init( uint8_t addr )
{
	// In order to avoid the LEDs being driven with garbage data
	// in the LED driver's PWM registers, first enable software shutdown,
	// then set up the mode and other settings, clear the PWM registers,
	// then disable software shutdown.

	// unlock command register
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER_LOCK, ISSI_COMMAND_REGISTER_WRITE_ONCE );

	// select "configure register"
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER, ISSI_FUNCTION_REGISTER );

	// enable software shutdown (it should already be shutdown)
	IS31FL3733_write_register( addr, ISSI_CONFIGURATION_REGISTER, ISSI_SSD_MODE );

	// this delay was copied from other drivers, might not be needed
	_delay_ms( 10 );

	// select bank 0
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER, ISSI_CONTROL_REGISTER );

	// turn off all LEDs in the LED control register
	for ( int i = 0x00; i <= 0x17; i++ )
	{
		IS31FL3733_write_register( addr, i, 0x00 );
	}

	// select "function register" bank
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER, ISSI_PWM_REGISTER );

	for ( int i = 0x00; i <= 0xBF; i++ )
	{
		IS31FL3733_write_register( addr, i, 0x00 );
	}

	// select "configure register"
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER, ISSI_FUNCTION_REGISTER );

	// disable software shutdown, turn on pwm mode
	IS31FL3733_write_register( addr, ISSI_CONFIGURATION_REGISTER, ISSI_NORMAL_OP | ISSI_PWM_MODE );

	// select bank 0 and leave it selected.
	// most usage after initialization is just writing PWM buffers in bank 0
	// as there's not much point in double-buffering
	IS31FL3733_write_register( addr, ISSI_COMMAND_REGISTER, ISSI_PWM_REGISTER );
}
