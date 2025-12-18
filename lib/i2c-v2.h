/* 
 * File:   i2c-v2.h
 * Author: norberto
 *
 * Created on 29 de junio de 2022, 11:43
 */

#ifndef I2C_V2_H
#define	I2C_V2_H

void i2c_start(void);

void i2c_stop(void);

void i2c_rstart(void);

unsigned char i2c_write(unsigned char I2C_data);

// master_ack = 1 => acknowledge. master_ack = 0 => not acknowledge
// Master must start (or restart) comunications first and send 
// address with the lower bit set.
unsigned char i2c_read(char master_ack);

#endif	/* I2C_V2_H */

