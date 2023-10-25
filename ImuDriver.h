#ifndef __IMU_DRIVER_H__
#define __IMU_DRIVER_H__

#include <mbed.h>
#include "internal/mpu9250-mbed/SparkFunMPU9250-DMP.h"
#include "internal/bno055-mbed/Adafruit_BNO055.h"
#include "internal/bhy2-mbed/src/BHY2.h"

#define DEFAULT_ATTEMPTS_NUM 2
#define FIFO_SAMPLE_RATE_OPERATION 10

#define BHY2_DEFAULT_I2C_ADDR 0x28
#define BHY2_VIRTUAL_SENSORS_RATE 10
#define BHY2_VIRTUAL_SENSORS_LATENCY 0

class ImuDriver :
    private NonCopyable<ImuDriver>
{
    public:
        enum Type{
            IMU_BNO055_ADDR_A,
            IMU_BNO055_ADDR_B,
            IMU_MPU9250,
            IMU_MPU9255,
            IMU_BHY2,
            UNKNOWN
        };

        struct ImuMeasurement
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
        static Type mpu9250_is_connected();
        bool mpu9250_init();
        void mpu9250_interrupt_cb();
        void mpu9250_loop();

        static Type bno055_is_connected();
        bool bno055_init();
        void bno055_loop();

        static Type bhy2_is_connected();
        bool bhy2_init();
        void bhy2_interrupt_cb();
        void bhy2_loop();

        mbed::I2C * _i2c;
        mbed::InterruptIn * _imu_int;
        Type _type;

        Adafruit_BNO055 * _bno055;
        MPU9250_DMP * _mpu9250;
        BHY2 * _bhy2;

        SensorQuaternion _bhy2_orientation_sensor;
        SensorXYZ _bhy2_gyration_sensor;
        SensorXYZ _bhy2_acceleration_sensor;

        rtos::Thread _thread;
        bool _initialized;
        volatile uint16_t _new_data_flag;
};

#endif