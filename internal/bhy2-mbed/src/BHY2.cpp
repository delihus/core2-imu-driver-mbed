#include "BHY2.h"
#include "BoschSensortec.h"
#include "BoschParser.h"
#include "mbed.h"


BHY2::BHY2() :
  _pingTime(0),
  _timeout(120000),
  _startTime(0)
{
}

BHY2::~BHY2()
{
}

void BHY2::pingI2C() {
//   int currTime = millis();
//   if ((currTime - _pingTime) > 30000) {
//     _pingTime = currTime;
// #ifdef USE_FASTCHG_TO_KICK_WATCHDOG
//     //Read charger reg
//     nicla::checkChgReg();
// #else
//     //Read LDO reg
//     nicla::readLDOreg();
// #endif
//   }
}

void BHY2::setLDOTimeout(int time) {
  _timeout = time;
}

bool BHY2::begin(mbed::I2C &i2c_ptr)
{
  return sensortec.begin(i2c_ptr);
}

void BHY2::update()
{
  pingI2C();

  sensortec.update();
}

// Update and then sleep
void BHY2::update(unsigned long ms)
{
  update();
  delay(ms);
}

void BHY2::delay(unsigned long ms)
{
  wait_us(ms*1000);
}

void BHY2::configureSensor(SensorConfigurationPacket& config)
{
  sensortec.configureSensor(config);
}

void BHY2::configureSensor(uint8_t sensorId, float sampleRate, uint32_t latency)
{
  SensorConfigurationPacket config;
  config.sensorId = sensorId;
  config.sampleRate = sampleRate;
  config.latency = latency;
  sensortec.configureSensor(config);
}

void BHY2::addSensorData(SensorDataPacket &sensorData)
{
  sensortec.addSensorData(sensorData);
}

void BHY2::addLongSensorData(SensorLongDataPacket &sensorData)
{
  sensortec.addLongSensorData(sensorData);
}

uint8_t BHY2::availableSensorData()
{
  return sensortec.availableSensorData();
}

uint8_t BHY2::availableLongSensorData()
{
  return sensortec.availableLongSensorData();
}

bool BHY2::readSensorData(SensorDataPacket &data)
{
  return sensortec.readSensorData(data);
}

bool BHY2::readLongSensorData(SensorLongDataPacket &data)
{
  return sensortec.readLongSensorData(data);
}

bool BHY2::hasSensor(uint8_t sensorId)
{
  return sensortec.hasSensor(sensorId);
}

void BHY2::parse(SensorDataPacket& data, DataXYZ& vector)
{
  DataParser::parse3DVector(data, vector);
}

void BHY2::parse(SensorDataPacket& data, DataOrientation& vector)
{
  DataParser::parseEuler(data, vector);
}

void BHY2::parse(SensorDataPacket& data, DataOrientation& vector, float scaleFactor)
{
  DataParser::parseEuler(data, vector, scaleFactor);
}

void BHY2::debug(mbed::Stream &stream)
{
  _debug = &stream;
  BoschParser::debug(stream);
}
