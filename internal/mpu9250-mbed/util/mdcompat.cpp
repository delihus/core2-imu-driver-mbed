#include "mdcompat.h"
#include <mbed.h>

static I2C * i2c = nullptr;
// static Timer * timer = nullptr;
char mpu9250_write_buffer[128]; 
int mpu9250_result;

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
    mpu9250_write_buffer[0] = reg_addr;
    mpu9250_result = 0;
    mpu9250_result += i2c->write((int)(slave_addr << 1), mpu9250_write_buffer, 1, 1);
    mpu9250_result += i2c->read((int)(slave_addr << 1), (char *)data, length, 0);
    return mpu9250_result;
}

int mbed_i2c_write(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data) 
{
    mpu9250_write_buffer[0] = reg_addr;
    memcpy(mpu9250_write_buffer+1, data, length);   
    return i2c->write((int)(slave_addr << 1), mpu9250_write_buffer, length + 1, 0);
}

void get_ms(unsigned long *count)
{
    // *count=Kernel::get_ms_count();
    *count = 0;
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