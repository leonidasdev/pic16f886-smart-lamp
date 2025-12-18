/* 
 * File:   spi-master-v1.h
 * Author: norberto
 *
 * Created on 12 de julio de 2022, 11:15
 */

#ifndef SPI_MASTER_V1_H
#define	SPI_MASTER_V1_H

#define spi_clk PORTCbits.RC0
#define spi_dat_out PORTCbits.RC1


char spi_write_read(char one_byte);

#endif	/* SPI_MASTER_V1_H */

