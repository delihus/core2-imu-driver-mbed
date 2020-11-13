#include "ImuDriver.h"
#include "internal/mpu9250-mbed/util/mdcompat.h"
#include "internal/mpu9250-mbed/MPU9250_RegisterMap.h"
#define BNO055_SUCCESS 0

#define STOP_FLAG 1
#define START_FLAG 2

Mail<ImuDriver::ImuMesurement, 10> imu_sensor_mail_box;

#pragma region MPU9250_FUNCTIONS

const signed char MPU925X_ORIENTATION_ROSBOT[9] = {
	0, -1, 0,
	-1, 0, 0,
	0, 0, -1
};

void ImuDriver::mpu9250_interrupt_cb()
{
    core_util_atomic_incr_u16(&_new_data_flag,1);
}

bool ImuDriver::mpu9250_init()
{
    inv_error_t err;
    if ((err = _mpu9250->begin(_i2c)) != INV_SUCCESS)
    {
        return false;
    }

    err += _mpu9250->setGyroFSR(2000); // 2000dps for gyro
    err += _mpu9250->setLPF(42); // 42Hz low pass filter

    err += _mpu9250->dmpBegin(DMP_FEATURE_6X_LP_QUAT     | // Enable 6-axis quat
                         DMP_FEATURE_GYRO_CAL       | // Use gyro calibration
                         DMP_FEATURE_SEND_RAW_ACCEL | // Enable raw accel measurements
                         DMP_FEATURE_SEND_CAL_GYRO,   // Enable cal gyro measurements
                         FIFO_SAMPLE_RATE_OPERATION); // Set DMP FIFO rate

    err += _mpu9250->dmpSetOrientation(MPU925X_ORIENTATION_ROSBOT);
    
    // err = _mpu9250->dmpSetOrientation(MPU_ORIENTATION);
    // The interrupt level can either be active-high or low.
    // Configure as active-low, since we'll be using the pin's
    // internal pull-up resistor.
    // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
    err += _mpu9250->setIntLevel(INT_ACTIVE_LOW);

    // The interrupt can be set to latch until data has
    // been read, or to work as a 50us pulse.
    // Use latching method -- we'll read from the sensor
    // as soon as we see the pin go LOW.
    // Options are INT_LATCHED or INT_50US_PULSE
    // Reading any register will clear the interrupt!!!
    err += _mpu9250->setIntLatched(INT_LATCHED);

    // Use enableInterrupt() to configure the MPU-9250's
    // interrupt output as a "data ready" indicator.
    // err += _mpu9250->enableInterrupt(1);
    
    // Disable dmp - it is enabled on demand using enableImu()
    // err = _mpu9250->dmpState(1);
    if(err == INV_SUCCESS)
    {
        _imu_int->mode(PullUp);
        _imu_int->fall(callback(this,&ImuDriver::mpu9250_interrupt_cb));
    }

    return err == INV_SUCCESS ? true : false;
}

void ImuDriver::mpu9250_loop()
{
    bool read_enable = false;

    while (1)
    {
        uint32_t flags = ThisThread::flags_get();

        if (flags & START_FLAG)
        {
            _mpu9250->dmpState(1);
            read_enable = true;
            ThisThread::flags_clear(START_FLAG);
        }
        else if (flags & STOP_FLAG)
        {
            _mpu9250->dmpState(0);
            read_enable = false;
            ThisThread::flags_clear(STOP_FLAG);
        }
        else if (read_enable)
        {
            // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
            // 10.2 Content of DMP Output to FIFO
            // The exact contents of the data output to the FIFO depend on the features that were enabled in Section 6.
            // When all features are enabled, a single set of FIFO data consists of 48 bytes. The data is ordered as shown
            // below:
            // * Low Power 3-Axis Quaternion (16 bytes)
            // * Low Power 6-Axis Quaternion (16 bytes)
            // * Raw Sensor Data (12 bytes)
            // * Gesture Word (Android Orientation + Tap outputs) (4 bytes)
            if (_new_data_flag || _mpu9250->fifoAvailable() >= 32)
            {
                if (_mpu9250->dmpUpdateFifo() == INV_SUCCESS)
                {
                    if (!imu_sensor_mail_box.full())
                    {
                        ImuMesurement *msg = imu_sensor_mail_box.alloc();
                        msg->orientation[0] = _mpu9250->calcQuat(_mpu9250->qx);
                        msg->orientation[1] = _mpu9250->calcQuat(_mpu9250->qy);
                        msg->orientation[2] = _mpu9250->calcQuat(_mpu9250->qz);
                        msg->orientation[3] = _mpu9250->calcQuat(_mpu9250->qw);
                        msg->angular_velocity[0] = _mpu9250->calcGyro(_mpu9250->gx);
                        msg->angular_velocity[1] = _mpu9250->calcGyro(_mpu9250->gy);
                        msg->angular_velocity[2] = _mpu9250->calcGyro(_mpu9250->gz);
                        msg->linear_velocity[0] = _mpu9250->calcAccel(_mpu9250->ax);
                        msg->linear_velocity[1] = _mpu9250->calcAccel(_mpu9250->ay);
                        msg->linear_velocity[2] = _mpu9250->calcAccel(_mpu9250->az);
                        msg->timestamp = _mpu9250->time;
                        imu_sensor_mail_box.put(msg);
                    }
                }
                core_util_atomic_decr_u16(&_new_data_flag, 1);
            }

            ThisThread::sleep_for(20);
        }
    }
}

#pragma endregion MPU9250_FUNCTIONS

#pragma region BNO055_FUNCTIONS

bool ImuDriver::bno055_init()
{
    if(_bno055->begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        _bno055->setExtCrystalUse(true);
        // _bno055->setAxisRemap();
        // _bno055->setAxisSign();
        return true;
    }
    return false;
}

void ImuDriver::bno055_loop()
{
    bool read_enable = false;
    const double scale = (1.0 / (1 << 14));

    uint8_t buffer[8];
    int16_t x, y, z, w;
    
    while (1)
    {
        uint32_t flags = ThisThread::flags_get();
        if (flags & START_FLAG)
        {
            _bno055->enterNormalMode();
            read_enable = true;
            ThisThread::flags_clear(START_FLAG);
        }
        else if (flags & STOP_FLAG)
        {
            _bno055->enterSuspendMode();
            read_enable = false;
            ThisThread::flags_clear(STOP_FLAG);
        }
        else if(read_enable)
        {
            memset(buffer, 0, 8);
            x = y = z = w = 0;
            /* Read quat data (8 bytes) */
            _bno055->readLen(Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
            w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
            x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
            y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
            z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
            uint32_t timestap = Kernel::get_ms_count();
            if (!imu_sensor_mail_box.full())
            {
                ImuMesurement *msg = imu_sensor_mail_box.alloc();
                msg->orientation[0] = (float)(x*scale);
                msg->orientation[1] = (float)(y*scale);
                msg->orientation[2] = (float)(z*scale);
                msg->orientation[3] = (float)(w*scale);
                msg->angular_velocity[0] = 0;
                msg->angular_velocity[1] = 0;
                msg->angular_velocity[2] = 0;
                msg->linear_velocity[0] = 0;
                msg->linear_velocity[1] = 0;
                msg->linear_velocity[2] = 0;
                msg->timestamp = timestap;
                imu_sensor_mail_box.put(msg);
            }
        }
        ThisThread::sleep_for(100);
    }
}

#pragma endregion BNO055_FUNCTIONS

ImuDriver::Type ImuDriver::getType(I2C * i2c_instance, int attempts)
{
    ImuDriver::Type type = UNKNOWN;
    int attempts_tmp = attempts; 
    uint8_t data;
    
    mbed_i2c_init(i2c_instance);
    
    // check if MPU9250/MPU9255 is connected
    while(attempts_tmp--)
    {
        if(mbed_i2c_read(MPU9250_DEFAULT_I2C_ADDR, MPU9250_WHO_AM_I, 1, &data) == INV_SUCCESS)
        {
            switch(data)
            {
                case MPU9250_WHO_AM_I_RESULT:
                    return ImuDriver::IMU_MPU9250;
                case MPU9255_WHO_AM_I_RESULT:
                    return ImuDriver::IMU_MPU9255;
            }
        }
    }

    ThisThread::sleep_for(50);

    // check if BNO055 is connected
    // ADDRESS_A
    int res = BNO055_SUCCESS;
    attempts_tmp = attempts;
    data = 0;
    while(attempts_tmp--)
    {
        res += mbed_i2c_write(BNO055_ADDRESS_A, Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1, &data);
        ThisThread::sleep_for(50);
        res += mbed_i2c_read(BNO055_ADDRESS_A, Adafruit_BNO055::BNO055_CHIP_ID_ADDR, 1, &data);
        ThisThread::sleep_for(50);
        if(res == BNO055_SUCCESS && data == BNO055_ID)
        {
            return ImuDriver::IMU_BNO055_ADDR_A;
        }
    }

    // ADDRESS_B
    res = BNO055_SUCCESS;
    attempts_tmp = attempts;
    data = 0;
    while(attempts_tmp--)
    {
        res += mbed_i2c_write(BNO055_ADDRESS_B, Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1, &data);
        ThisThread::sleep_for(50);
        res += mbed_i2c_read(BNO055_ADDRESS_B, Adafruit_BNO055::BNO055_CHIP_ID_ADDR, 1, &data);
        ThisThread::sleep_for(50);
        if(res == BNO055_SUCCESS && data == BNO055_ID)
        {
            return ImuDriver::IMU_BNO055_ADDR_B;
        }
    }

    return UNKNOWN;
}

ImuDriver::ImuDriver(I2C * i2c_instance, Type imu_type)
: _i2c(i2c_instance)
, _imu_int(nullptr)
, _type(imu_type)
, _bno055(nullptr)
, _mpu9250(nullptr)
, _thread(osPriorityNormal,OS_STACK_SIZE,nullptr,"imu_thread")
, _initialized(false)
{}

bool ImuDriver::init(){
    if(_initialized)
    {
        return false;
    }

    if(_type == IMU_MPU9255 || _type == IMU_MPU9250)
    {
        if(_mpu9250 == nullptr)
            _mpu9250 = new MPU9250_DMP;
        if(_imu_int == nullptr)
            _imu_int = new InterruptIn(SENS2_PIN1);

        if(mpu9250_init())
        {
            _thread.start(callback(this,&ImuDriver::mpu9250_loop));
            _initialized = true;
            return true;
        }
    }
    else if(_type == IMU_BNO055_ADDR_A || _type == IMU_BNO055_ADDR_B)
    {
        if(_bno055 == nullptr)
            _bno055 = new Adafruit_BNO055(_i2c, 1, _type == IMU_BNO055_ADDR_A ? BNO055_ADDRESS_A : BNO055_ADDRESS_B);

        if(bno055_init())
        {
            _thread.start(callback(this,&ImuDriver::bno055_loop));
            _initialized = true;
            return true;
        }
    }
    return false;
}

bool ImuDriver::start()
{
    if(!_initialized)
        return false;
    _thread.flags_set(START_FLAG);
    return true;
}

bool ImuDriver::stop()
{
    if(!_initialized)
        return false;
    _thread.flags_set(STOP_FLAG);
    return true;
}

bool ImuDriver::restart(){}
