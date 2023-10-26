#ifndef ARDUINO_BHY2_H_
#define ARDUINO_BHY2_H_

#include "sensors/SensorTypes.h"
#include "sensors/DataParser.h"

#include "sensors/SensorClass.h"
#include "sensors/SensorOrientation.h"
#include "sensors/SensorXYZ.h"
#include "sensors/SensorQuaternion.h"
#include "sensors/SensorBSEC.h"
#include "sensors/SensorActivity.h"
#include "sensors/Sensor.h"

#include "sensors/SensorID.h"

/**
 * @brief Class to interface with the Bosch BHI260 sensor hub on the Nicla Sense ME.
 * Provides functionality for reading/configuring sensors based on sensor ID and accessing debug features.
*/
class BHY2 {

public:
  BHY2();
  virtual ~BHY2();

  bool begin(mbed::I2C &i2c_ptr, mbed::DigitalIn& int_pin);

  /**
   *  @brief Update sensor data by reading the FIFO buffer on the BHI260 and then pass it to a suitable parser.
   *
   *  @param ms (optional) Time (in milliseconds) to wait before returning data.
   */
  void update(); // remove this to enforce a sleep
  void update(unsigned long ms); // Update and then sleep
  /**
  *   @brief Delay method to be used instead of Arduino delay().
  *
  *   @note Unlike the standard Arduino delay, this modified delay method uses a function to check if the elapsed time has passed.
  * This does not stall the microprocessor.
  *
  *   @param ms Time (in milliseconds) to enforce delay.
  *
  *   @code
  *   BHY2.delay(100);
  *   @endcode
  */
  void delay(unsigned long ms); // to be used instead of arduino delay()

  // API for using the Bosch sensortec from sketch
  /**
   * @brief Configure a virtual sensor on the BHI260 to have a set sample rate (Hz) and latency (milliseconds)
   * This can be achieved
   *
   * @param config Instance of @see SensorConfigurationPacket class, with sensorID, sampleRate and latency
   *
   * @code
   * #set virtual sensor SENSOR_ID_DEVICE_ORI_WU to have sample rate of 1 Hz and latency of 500 milliseconds
   * SensorConfigurationPacket config(70, 1, 500);
   * BHY2.configureSensor(config)
   * @endcode
   *
   * @note Alternatively, we can directly configure the virtual sensor without creating a SensorConfigurationPacket class
   *
   * @param sensorId  SensorID for virtual sensor
   * @param sampleRate Polling rate for sensor in Hz
   * @param latency   Latency in milliseconds
   *
   * @note For list of SensorID, see src/SensorID.h. Or see Table 79 in the <a href="https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bhi260ab-ds000.pdf">BHI260 datasheet</a>
   *
   */
  void configureSensor(SensorConfigurationPacket& config);
  void configureSensor(uint8_t sensorId, float sampleRate, uint32_t latency);
  /**
   * @brief Extract data from the FIFO buffer for a specific SensorID and save into an instance of the SensorDataPacket struct. Eliminate oldest data when sensor queue is full.
   *
   * @param sensorData Struct containing SensorID (uint8_t), data payload size (uint8_t) and data corresponding to SensorID (uint8_t).
   */
  void addSensorData(SensorDataPacket &sensorData);
  /**
   * @brief Extract data from the FIFO buffer for a specific SensorID and save into an instance of the SensorLongDataPacket struct. Eliminate oldest data when sensor queue is full.
   *
   * @param sensorData Struct containing SensorID (uint8_t), long data payload size (uint8_t) and data corresponding to SensorID (uint8_t).
   */
  void addLongSensorData(SensorLongDataPacket &sensorData);
  /**
   * @brief Return available sensor data within the FIFO buffer queue
   *
   * @return uint8_t The amount of data in bytes
   */
  uint8_t availableSensorData();
  /**
   * @brief Return available long sensor data within the FIFO buffer queue
   *
   * @return uint8_t The amount of data in bytes
   */
  uint8_t availableLongSensorData();
  /**
   * @brief Read sensor data from the top element of the queue
   *
   * @param data Structure including sensorID, sampleRate and latency
   */
  bool readSensorData(SensorDataPacket &data);
  /**
   * @brief Read long sensor data from the top element of the queue
   *
   * @param data Structure including sensorID, sampleRate and latency
   */
  bool readLongSensorData(SensorLongDataPacket &data);
  /**
   * @brief Check existence of a given sensorID
   *
   * @param sensorID Selected virtual sensor
   * @return True Sensor is present
   */
  bool hasSensor(uint8_t sensorId);
  /**
   * @brief Parse XYZ Cartesian data from a given data packet
   *
   * @param data data packet including SensorID
   * @param vector vector with XYZ
   */
  void parse(SensorDataPacket &data, DataXYZ &vector);
  /**
   * @brief Parse orientation from a given data packet
   *
   * @param data Data packet including SensorID
   * @param vector Vector with heading, pitch and roll
   */
  void parse(SensorDataPacket &data, DataOrientation &vector);
  /**
   * @brief Parse orientation with scale factor
   *
   * @param data Data packet including SensorID
   * @param vector Vector with heading, pitch and roll
   * @param scaleFactor scale factor for vector
   */
  void parse(SensorDataPacket &data, DataOrientation &vector, float scaleFactor);
  /**
   * @brief Define LDO regulator timeout time
   *
   * @param time in milliseconds. 120000 ms by default.
   */
  void setLDOTimeout(int time);
  /**
   * @brief Prints debug stream of multiple library components to the specified stream object.
   *
   * @param Stream object that implements the printf() function.
   */
  void debug(mbed::Stream &stream);

private:
  mbed::Stream *_debug = nullptr;

  void pingI2C();
  int _pingTime;
  int _timeout;
  int _startTime;
};

#endif
