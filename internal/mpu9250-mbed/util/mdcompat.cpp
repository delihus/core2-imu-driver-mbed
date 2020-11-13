#include "mdcompat.h"
#include <mbed.h>

static I2C * i2c = nullptr;
// static Timer * timer = nullptr;

unsigned short constrain(
    unsigned short x,
    unsigned short a,
    unsigned short b)
{
    unsigned short result;
    result = (x < a) ? a : x;
    result = (x > b) ? b : x;
    return result;
}

void mbed_i2c_init(void * i2c_instance)
{
    if(i2c_instance == nullptr)
        return; 
    i2c = (I2C *)i2c_instance;
}

void delay_ms(
    unsigned long num_ms)
{
    ThisThread::sleep_for(num_ms);
}

int mbed_i2c_read(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data)
{
    const char RA[] = {reg_addr};
    int result = i2c->write((int)(slave_addr << 1), RA, 1, 1);
    result += i2c->read((int)(slave_addr << 1), (char *)data, length, 0);
    return result;
}

int mbed_i2c_write(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data) {

    int buffer_length = length + 1;
    char buffer[buffer_length] = {reg_addr};
    const char * RA = buffer;
    memcpy(buffer+1, data, length);
    
    int result = i2c->write((int)(slave_addr << 1), (const char*)buffer, buffer_length, 0);
    return result;
}

void get_ms(unsigned long *count)
{
    *count=Kernel::get_ms_count();
}

int reg_int_cb(
    void (*cb)(void),
    unsigned char port,
    unsigned char pin)
{
    return 0;
}

long labs(long x)
{
    return x > 0 ? x : -x;
}

float fabsf(float x)
{
    return x > 0 ? x : -x;
}

int min(int a, int b)
{
    return a > b ? b : a;
}