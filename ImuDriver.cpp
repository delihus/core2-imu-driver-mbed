#include "ImuDriver.h"
#include "internal/mpu9250-mbed/util/mdcompat.h"
#include "internal/mpu9250-mbed/MPU9250_RegisterMap.h"
#define BNO055_SUCCESS 0

#define STOP_FLAG 1
#define START_FLAG 2

Mail<ImuDriver::ImuMeasurement, 10> imu_sensor_mail_box;

#pragma region MPU9250_FUNCTIONS

const signed char MPU925X_ORIENTATION_ROSBOT[9] = {
	0, -1, 0,
	-1, 0, 0,
	0, 0, -1
};

ImuDriver::Type ImuDriver::mpu9250_is_connected(){
    uint8_t data;
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
    return Type::UNKNOWN;
}

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
                        uint32_t timestap = Kernel::get_ms_count();
                        ImuMeasurement *msg = imu_sensor_mail_box.alloc();
                        msg->orientation[0] = _mpu9250->calcQuat(_mpu9250->qx);
                        msg->orientation[1] = _mpu9250->calcQuat(_mpu9250->qy);
                        msg->orientation[2] = _mpu9250->calcQuat(_mpu9250->qz);
                        msg->orientation[3] = _mpu9250->calcQuat(_mpu9250->qw);

                        msg->angular_velocity[0] = SENSORS_DPS_TO_RADS * _mpu9250->calcGyro(_mpu9250->gx);
                        msg->angular_velocity[1] = SENSORS_DPS_TO_RADS * _mpu9250->calcGyro(_mpu9250->gy);
                        msg->angular_velocity[2] = SENSORS_DPS_TO_RADS * _mpu9250->calcGyro(_mpu9250->gz);

                        // raw accel data is not transformed
                        // [accx]   [ 0 -1  0 ]   [accx_raw]
                        // [accy] = [-1  0  0 ] * [accy_raw]
                        // [accz]   [ 0  0 -1 ]   [accz_raw]

                        msg->linear_acceleration[0] = -SENSORS_GRAVITY_EARTH * _mpu9250->calcAccel(_mpu9250->ay);
                        msg->linear_acceleration[1] = -SENSORS_GRAVITY_EARTH * _mpu9250->calcAccel(_mpu9250->ax);
                        msg->linear_acceleration[2] = -SENSORS_GRAVITY_EARTH * _mpu9250->calcAccel(_mpu9250->az);

                        msg->timestamp = timestap;
                        imu_sensor_mail_box.put(msg);
                    }
                }
                core_util_atomic_decr_u16(&_new_data_flag, 1);
            }

            ThisThread::sleep_for(20);
        }
    }
}

ImuDriver::Type ImuDriver::bno055_is_connected(){
    int res = BNO055_SUCCESS;
    uint8_t data = 0;
    res += mbed_i2c_write(BNO055_ADDRESS_A, Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1, &data);
    ThisThread::sleep_for(50);
    res += mbed_i2c_read(BNO055_ADDRESS_A, Adafruit_BNO055::BNO055_CHIP_ID_ADDR, 1, &data);
    ThisThread::sleep_for(50);
    if(res == BNO055_SUCCESS && data == BNO055_ID)
    {
        return ImuDriver::IMU_BNO055_ADDR_A;
    }

    res = BNO055_SUCCESS;
    res += mbed_i2c_write(BNO055_ADDRESS_B, Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 1, &data);
    ThisThread::sleep_for(50);
    res += mbed_i2c_read(BNO055_ADDRESS_B, Adafruit_BNO055::BNO055_CHIP_ID_ADDR, 1, &data);
    ThisThread::sleep_for(50);
    if(res == BNO055_SUCCESS && data == BNO055_ID)
    {
        return ImuDriver::IMU_BNO055_ADDR_B;
    }
    return ImuDriver::UNKNOWN;
}

bool ImuDriver::bno055_init()
{
    if(_bno055->begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        _bno055->setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
        _bno055->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

        /* !!!!!!!!!!!!!!!! BNO055 UNIT SETTINGS !!!!!!!!!!!!!!!!!! */
        _bno055->write8(Adafruit_BNO055::BNO055_PAGE_ID_ADDR, 0);
        ThisThread::sleep_for(25);
        /* Set the output units (based on section 3.6.1 Unit selection) */
        uint8_t unitsel = (0 << 7) | // Orientation = Android
                          (0 << 4) | // Temperature = Celsius
                          (1 << 2) | // Euler = Rads
                          (1 << 1) | // Gyro = Rads
                          (0 << 0);  // Accelerometer = m/s^2
        _bno055->write8(Adafruit_BNO055::BNO055_UNIT_SEL_ADDR, unitsel);
        ThisThread::sleep_for(10);
        /* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */

        _bno055->setExtCrystalUse(true);
        return true;
    }
    return false;
}

void ImuDriver::bno055_loop()
{
    bool read_enable = false;
    const double quat_scale = (1.0 / (1 << 14)); // normalized
    const double accel_scale = 100.0; // m/s2
    // const double gyro_scale = 16.0;  // dps
    const double gyro_scale = 900.0;  // rps

    uint8_t buffer[8];
    int16_t qx, qy, qz, qw;
    int16_t ax, ay, az;
    int16_t wx, wy, wz;

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
            qx = qy = qz = qw = 0;
            _bno055->readLen(Adafruit_BNO055::BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
            qw = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
            qx = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
            qy = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
            qz = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

            memset(buffer, 0, 6);
            ax = ay = az = 0;
            // VECTOR_ACCELEROMETER -> with gravity vector
            // VECTOR_LINEARACCEL -> removed gravity vector
            _bno055->readLen((Adafruit_BNO055::adafruit_bno055_reg_t)Adafruit_BNO055::VECTOR_ACCELEROMETER, buffer, 6);
            ax = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
            ay = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
            az = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

            memset(buffer, 0, 6);
            wx = wy = wz = 0;
            _bno055->readLen((Adafruit_BNO055::adafruit_bno055_reg_t)Adafruit_BNO055::VECTOR_GYROSCOPE, buffer, 6);
            wx = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
            wy = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
            wz = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

            uint32_t timestap = Kernel::get_ms_count();
            if (!imu_sensor_mail_box.full())
            {
                ImuMeasurement *msg = imu_sensor_mail_box.alloc();
                msg->orientation[0] = (float)((double)qx * quat_scale);
                msg->orientation[1] = (float)((double)qy * quat_scale);
                msg->orientation[2] = (float)((double)qz * quat_scale);
                msg->orientation[3] = (float)((double)qw * quat_scale);
                msg->angular_velocity[0] = (float)((double)wx / gyro_scale);
                msg->angular_velocity[1] = (float)((double)wy / gyro_scale);
                msg->angular_velocity[2] = (float)((double)wz / gyro_scale);
                msg->linear_acceleration[0] = (float)((double)ax / accel_scale);
                msg->linear_acceleration[1] = (float)((double)ay / accel_scale);
                msg->linear_acceleration[2] = (float)((double)az / accel_scale);
                msg->timestamp = timestap;
                imu_sensor_mail_box.put(msg);
            }
        }
        ThisThread::sleep_for(100);
    }
}

ImuDriver::Type ImuDriver::bhy2_is_connected(){
    unsigned char data[2] = "1";
    if(mbed_i2c_write(BHY2_DEFAULT_I2C_ADDR, 1, 1, data) == 0){
        return Type::IMU_BHY2;
    }
    return Type::UNKNOWN;
}

bool ImuDriver::bhy2_init(){
    bool ret = _bhy2->begin();
    ret += _bhy2_orientation_sensor.begin(10, 0);
    ret += _bhy2_gyration_sensor.begin(10, 0);
    ret += _bhy2_acceleration_sensor.begin(10, 0);
    return ret;
}

void ImuDriver::bhy2_loop(){
    bool read_enable = false;
    const double accel_scale = 1.0 / 4096.0 * 9.80665;; // m/s2
    const double gyro_scale = 1.0 / 32.768;  // rps

    while (1)
    {
        uint32_t flags = ThisThread::flags_get();
        if (flags & START_FLAG)
        {
            read_enable = true;
            ThisThread::flags_clear(START_FLAG);
        }
        else if (flags & STOP_FLAG)
        {
            read_enable = false;
            ThisThread::flags_clear(STOP_FLAG);
        }
        else if(read_enable){
            _bhy2->update();

            if (!imu_sensor_mail_box.full())
            {
                uint32_t timestap = Kernel::get_ms_count();
                ImuMeasurement *msg = imu_sensor_mail_box.alloc();
                msg->orientation[0] = _bhy2_orientation_sensor.x();
                msg->orientation[1] = _bhy2_orientation_sensor.y();
                msg->orientation[2] = _bhy2_orientation_sensor.z();
                msg->orientation[3] = _bhy2_orientation_sensor.w();

                msg->angular_velocity[0] = _bhy2_gyration_sensor.x() * gyro_scale;
                msg->angular_velocity[1] = _bhy2_gyration_sensor.y() * gyro_scale;
                msg->angular_velocity[2] = _bhy2_gyration_sensor.z() * gyro_scale;

                msg->linear_acceleration[0] = (float)_bhy2_acceleration_sensor.x() * accel_scale;
                msg->linear_acceleration[1] = (float)_bhy2_acceleration_sensor.y() * accel_scale;
                msg->linear_acceleration[2] = (float)_bhy2_acceleration_sensor.z() * accel_scale;

                msg->timestamp = timestap;
                imu_sensor_mail_box.put(msg);
            }
        }

        ThisThread::sleep_for(100);
    }
}

ImuDriver::Type ImuDriver::getType(I2C * i2c_instance, int attempts)
{
    int attempts_tmp = attempts;
    mbed_i2c_init(i2c_instance);

    // check if MPU9250/MPU9255 is connected
    while(attempts_tmp--)
    {
        auto type = mpu9250_is_connected();
        if(type != Type::UNKNOWN){
            return type;
        }
    }

    ThisThread::sleep_for(50);
    attempts_tmp = attempts;
    while(attempts_tmp--)
    {
        auto type = bno055_is_connected();
        if(type != Type::UNKNOWN){
            return type;
        }
    }

    ThisThread::sleep_for(50);
    attempts_tmp = attempts;
    while(attempts_tmp--)
    {
        auto type = bhy2_is_connected();
        if(type != Type::UNKNOWN){
            return type;
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
, _bhy2(nullptr)
, _bhy2_orientation_sensor(SENSOR_ID_RV)
, _bhy2_gyration_sensor(SENSOR_ID_GYRO)
, _bhy2_acceleration_sensor(SENSOR_ID_LACC)
, _thread(osPriorityNormal,OS_STACK_SIZE, nullptr, "imu_thread")
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
    else if(_type == IMU_BHY2){
        if(_bhy2 == nullptr)
            _bhy2 = new BHY2();

        if(bhy2_init())
        {
            _thread.start(callback(this,&ImuDriver::bhy2_loop));
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

bool ImuDriver::restart(){
    return false;
}
