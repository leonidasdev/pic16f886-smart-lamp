// General routines for i2c comunications

#define i2c_ack 1
#define i2c_nack 0

#include <xc.h>

void wait_MSSP(void)
{
    while(!PIR1bits.SSPIF);
    
    PIR1bits.SSPIF = 0;
    return;
}

void i2c_start(void)
{
    SSPCON2bits.SEN = 1;
    wait_MSSP(); //Wait to complete
    return;
}

void i2c_stop(void)
{
    SSPCON2bits.PEN = 1;
    wait_MSSP(); //Wait to complete
    return;
}

void i2c_rstart(void)
{
    SSPCON2bits.RSEN = 1;
    wait_MSSP(); //Wait to complete
    return;
}

unsigned char i2c_write(unsigned char I2C_data)
{
    SSPBUF = I2C_data;
    wait_MSSP(); //Wait to complete
    return((unsigned char)!SSPCON2bits.ACKSTAT);  //Returns 1 if byte is acknowledge
}

// master_ack = 1 => acknowledge. master_ack = 0 => not acknowledge
// Master must start (or restart) comunications first and send 
// address with the lower bit set.
unsigned char i2c_read(char master_ack)
{
    unsigned char I2C_data;
    
    SSPCON2bits.RCEN = 1;
    wait_MSSP(); // Wait for data
    
    I2C_data = SSPBUF; // It is necessary to read the buffer for SCL progress
    
    if(master_ack)
        SSPCON2bits.ACKDT = 0;
    else
        SSPCON2bits.ACKDT = 1;
    
    SSPCON2bits.ACKEN = 1;
    wait_MSSP();
    return(I2C_data);
}