/*
 * i2c.h - I2C library for avr-gcc for devices with TWI
 * Version 0.5
 * 12 Oct 2016
 *
 * Copyright (C) 2014 Jason Milldrum <milldrum@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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

#include "defs.h"

#ifndef I2C_H_
#define I2C_H_

//#define CHECK_FOR_I2C_TIMEOUT /* limit number of tries for i2c success */

#define I2C_SDA					PC4
#define I2C_SDA_PORT			PORTC
#define I2C_SDA_DDR				DDRC

#define I2C_SCL					PC5
#define I2C_SCL_PORT			PORTC
#define I2C_SCL_DDR				DDRC

#define TRIES					10

#ifndef BOOL
typedef uint8_t BOOL;
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE !FALSE
#endif

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
BOOL i2c_write_success(uint8_t, uint8_t);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
BOOL i2c_status(uint8_t);

#endif /* I2C_H_ */
