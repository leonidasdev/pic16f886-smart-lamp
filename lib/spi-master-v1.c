
#include "spi-master-v1.h"
#include <xc.h>

#define _XTAL_FREQ 20000000 // necessary for __delay_us


char spi_write_read(char one_byte)
{
    char answer, x;
    
    answer = 0;
    
    for(x = 8; x > 0; x--)
    {
        spi_dat_out = (__bit)((one_byte >> (x - 1)) & 0b00000001);
        //__delay_us(5);
        spi_clk = 1;
        //__delay_us(10);
        // answer |= (char)spi_dat_in; // No usado: solo transmisión (MOSI)
        spi_clk = 0;
        //__delay_us(10);
        if(x > 1)
            answer = (char)(answer << 1);
    }
    
    return answer;
}
