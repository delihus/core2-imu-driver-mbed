#ifndef __IMU_DRIVER_H__
#define __IMU_DRIVER_H__

#include "internal/mpu9250-mbed/SparkFunMPU9250-DMP.h"
#include "internal/bno055-mbed/Adafruit_BNO055.h"

#define DEFAULT_ATTEMPTS_NUM 2
#define FIFO_SAMPLE_RATE_OPERATION 10

class ImuDriver : 
    private NonCopyable<ImuDriver>
{
    public:
        enum Type{
            IMU_BNO055_ADDR_A,
            IMU_BNO055_ADDR_B,
            IMU_MPU9250,
            IMU_MPU9255,
            UNKNOWN
        };

        struct ImuMesurement 
        {
            float orientation[4];
            float angular_velocity[3];
            float linear_acceleration[3];
            uint32_t timestamp;
        };

        static Type getType(I2C * i2c_instance, int attempts = DEFAULT_ATTEMPTS_NUM);
        ImuDriver(I2C * i2c_instance, Type imu_type);
        bool init();
        bool start();
        bool stop();
        bool restart();
        ~ImuDriver(){};

    private:
        bool mpu9250_init();
        void mpu9250_interrupt_cb();
        void mpu9250_loop();

    private:
        bool bno055_init();
        void bno055_loop();

    private:
        I2C * _i2c;
        InterruptIn * _imu_int;
        Type _type;
        Adafruit_BNO055 * _bno055;
        MPU9250_DMP * _mpu9250;
        Thread _thread;
        bool _initialized;
        volatile uint16_t _new_data_flag;
};

#endif