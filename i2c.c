/*
 * i2c.c - I2C library for avr-gcc for devices with TWI
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

#include <util/twi.h>
#include <avr/power.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>

#include "si5351.h"
#include "i2c.h"

void i2c_init(void)
{
	power_twi_enable();

	//set SCL to ~100 kHz for 8 MHz CPU clock
	TWSR = 0; /* Prescale /1 */
	TWBR = 0x25;

	//enable I2C
	TWCR = _BV(TWEN);
}

void i2c_start(void)
{
#ifdef CHECK_FOR_I2C_TIMEOUT
	uint8_t timeout = TRIES;
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)) && --timeout);
#else
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));
#endif
}

void i2c_stop(void)
{
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

BOOL i2c_write_success(uint8_t data, uint8_t success)
{
#ifdef CHECK_FOR_I2C_TIMEOUT
	uint8_t timeout = TRIES;
	TWDR = data;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while (!(TWCR & _BV(TWINT)) && --timeout);
	
	// ignore timeout condition to ensure i2c_stop() gets sent
	if(TW_STATUS != success)
	{
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
		return TRUE;
	}
	
	return FALSE;
#else
	TWDR = data;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));
	
	// ignore timeout condition to ensure i2c_stop() gets sent
	if(TW_STATUS != success)
	{
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
		return TRUE;
	}
	
	return FALSE;
#endif
}

uint8_t i2c_read_ack(void)
{
#ifdef CHECK_FOR_I2C_TIMEOUT
	uint8_t timeout = TRIES;
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while(!(TWCR & _BV(TWINT)) && --timeout);
	return TWDR;
#else
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while(!(TWCR & _BV(TWINT)));
	return TWDR;
#endif
}

uint8_t i2c_read_nack(void)
{
#ifdef CHECK_FOR_I2C_TIMEOUT
	uint8_t timeout = TRIES;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)) && --timeout);
	return TWDR;
#else
	TWCR = _BV(TWINT) | _BV(TWEN);
	while(!(TWCR & _BV(TWINT)));
	return TWDR;
#endif
}

BOOL i2c_status(uint8_t status)
{
	if(TW_STATUS != status)
	{
		TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
		return TRUE;
	}
	
	return FALSE;
}
