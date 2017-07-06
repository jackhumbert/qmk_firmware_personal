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

#ifndef IS31FL3733_DRIVER_H
#define IS31FL3733_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Point {
	uint8_t x;
	uint8_t y;
} __attribute__((packed)) Point;

typedef struct is31_led {
	uint8_t driver:2;
	uint8_t matrix:1;
	uint8_t modifier:1;
	uint8_t control_index;
	union {
		uint8_t raw;
		struct {
			uint8_t row:4; // 16 max
			uint8_t col:4; // 16 max
		};
	} matrix_co;
	Point point;
} __attribute__((packed)) is31_led;

extern const is31_led g_is31_leds[DRIVER_LED_TOTAL];

void IS31FL3733_init( uint8_t addr );
void IS31FL3733_write_register( uint8_t addr, uint8_t reg, uint8_t data );
void IS31FL3733_write_pwm_buffer( uint8_t addr, uint8_t *pwm_buffer );

#endif // IS31FL3733_DRIVER_H