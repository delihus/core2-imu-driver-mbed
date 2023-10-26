#ifndef DATA_PARSER_H_
#define DATA_PARSER_H_

#include <string>
#include "sensors/SensorTypes.h"
#include "SensorID.h"

typedef std::string String;

struct DataXYZ {
  int16_t x;
  int16_t y;
  int16_t z;

  String toString() {
    return (String)("XYZ values - X: " + std::to_string(x)
                    + "   Y: " + std::to_string(y)
                    + "   Z: " + std::to_string(z) + "\n");
  }
};

struct DataOrientation {
  float heading;
  float pitch;
  float roll;

  String toString() {
    return (String)("Orientation values - heading: " + std::to_string(heading)
                    + "   pitch: " + std::to_string(pitch)
                    + "   roll: " + std::to_string(roll) + "\n");
  }
};

struct DataQuaternion {
  float x;
  float y;
  float z;
  float w;
  float accuracy;

  String toString() {
    return (String)("Quaternion values - X: " + std::to_string(x)
                    + "   Y: " + std::to_string(y)
                    + "   Z: " + std::to_string(z)
                    + "   W: " + std::to_string(w)
                    + "   Accuracy: " + std::to_string(accuracy)
                    + "\n");
  }
};

struct DataBSEC {
  uint16_t  iaq;         //iaq value for regular use case
  uint16_t  iaq_s;       //iaq value for stationary use cases
  float     b_voc_eq;    //breath VOC equivalent (ppm)
  uint32_t  co2_eq;      //CO2 equivalent (ppm) [400,]
  float     comp_t;      //compensated temperature (celcius)
  float     comp_h;      //compensated humidity
  uint32_t  comp_g;      //compensated gas resistance (Ohms)
  uint8_t   accuracy;    //accuracy level: [0-3]

  String toString() {
    return (String)("BSEC output values - iaq: " + std::to_string(iaq)
                    + "   iaq_s: " + std::to_string(iaq_s)
                    + "   b_voc_eq: " + std::to_string(b_voc_eq)
                    + "   co2_eq: " + std::to_string(co2_eq)
                    + "   accuracy: " + std::to_string(accuracy)
                    + "   comp_t: " + std::to_string(comp_t)
                    + "   comp_h: " + std::to_string(comp_h)
                    + "   comp_g: " + std::to_string(comp_g)
                    + "\n");
  }
};


class DataParser {
public:
  static void parse3DVector(SensorDataPacket& data, DataXYZ& vector);
  static void parseEuler(SensorDataPacket& data, DataOrientation& vector);
  static void parseEuler(SensorDataPacket& data, DataOrientation& vector, float scaleFactor);
  static void parseQuaternion(SensorDataPacket& data, DataQuaternion& vector, float scaleFactor);
  static void parseBSEC(SensorLongDataPacket& data, DataBSEC& vector);
  static void parseBSECLegacy(SensorLongDataPacket& data, DataBSEC& vector);
  static void parseData(SensorDataPacket& data, float& value, float scaleFactor, SensorPayload format);
  static void parseActivity(SensorDataPacket& data, uint16_t& value);
};

#endif
