#include "BoschParser.h"
#include "BoschSensortec.h"
#include "sensors/SensorID.h"

mbed::Stream* BoschParser::_debug = NULL;

void BoschParser::debug(mbed::Stream &stream)
{
 _debug = &stream;
}

void BoschParser::convertTime(uint64_t time_ticks, uint32_t *s, uint32_t *ns)
{
    uint64_t timestamp = time_ticks; /* Store the last timestamp */

    timestamp = timestamp * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(timestamp / UINT64_C(1000000000));
    *ns = (uint32_t)(timestamp - ((*s) * UINT64_C(1000000000)));
}

void BoschParser::parseData(const struct bhy2_fifo_parse_data_info *fifoData, void *arg)
{
  int8_t sz;

  sz = fifoData->data_size - 1;
  if (sz <= SENSOR_DATA_FIXED_LENGTH) {
      SensorDataPacket sensorData;
      sensorData.sensorId = fifoData->sensor_id;
      sensorData.size = sz + 1;
      if (sz > 0)
          memcpy(&sensorData.data, fifoData->data_ptr, sz);

      sensortec.addSensorData(sensorData);
  } else {
      SensorLongDataPacket sensorDataLong;
      sensorDataLong.sensorId = fifoData->sensor_id;
      sz = (sz <= SENSOR_LONG_DATA_FIXED_LENGTH) ? sz : SENSOR_LONG_DATA_FIXED_LENGTH;
      sensorDataLong.size = sz + 1;

      memcpy(&sensorDataLong.data, fifoData->data_ptr, sz);
      sensortec.addLongSensorData(sensorDataLong);
  }

  // if (_debug) {
   //_debug->printf("Sensor: ");
   //_debug->printf(fifoData->sensor_id);
   //_debug->printf(" size: ");
   //_debug->printf(fifoData->data_size);
   //_debug->printf("  value: ");
    // for (uint8_t i = 0; i < (fifoData->data_size - 1); i++)
    // {
    //    //_debug->printf(fifoData->data_ptr[i], HEX);
    //    //_debug->printf(" ");
    // }
   //_debug->printf("  ");
  // }
}

void BoschParser::parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint8_t meta_event_type = callback_info->data_ptr[0];
  uint8_t byte1 = callback_info->data_ptr[1];
  uint8_t byte2 = callback_info->data_ptr[2];
  uint32_t s, ns;
  const char *event_text;

  if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT)
  {
    event_text = "[META EVENT]";
  }
  else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU)
  {
    event_text = "[META EVENT WAKE UP]";
  }
  else
  {
    return;
  }

  convertTime(*callback_info->time_stamp, &s, &ns);

  struct parse_ref *parse_table = (struct parse_ref*)callback_ref;
  (void)parse_table;

  // if (_debug) {

  //   switch (meta_event_type)
  //   {
  //     case BHY2_META_EVENT_FLUSH_COMPLETE:
  //       //printf("%s; T: %u.%09u; Flush complete for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Flush complete for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
  //       //printf("%s; T: %u.%09u; Sample rate changed for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Sample rate changed for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_POWER_MODE_CHANGED:
  //       //printf("%s; T: %u.%09u; Power mode changed for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Power mode changed for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_ALGORITHM_EVENTS:
  //       //printf("%s; T: %u.%09u; Algorithm event\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Algorithm event");
  //       break;
  //     case BHY2_META_EVENT_SENSOR_STATUS:
  //       //printf("%s; T: %u.%09u; Accuracy for sensor id %u changed to %u\r\n", event_text, s, ns, byte1, byte2);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Accuracy for sensor id ");
  //      //_debug->printf(byte1);
  //      //_debug->printf(" changed to ");
  //      //_debug->printf(byte2);
  //       break;
  //     case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
  //       //printf("%s; T: %u.%09u; BSX event (do steps main)\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Algorithm event");
  //       break;
  //     case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
  //       //printf("%s; T: %u.%09u; BSX event (do steps calib)\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" BSX event (do steps calib)");
  //       break;
  //     case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
  //       //printf("%s; T: %u.%09u; BSX event (get output signal)\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" BSX event (get output signal)");
  //       break;
  //     case BHY2_META_EVENT_SENSOR_ERROR:
  //       //printf("%s; T: %u.%09u; Sensor id %u reported error 0x%02X\r\n", event_text, s, ns, byte1, byte2);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Sensor id ");
  //      //_debug->printf(byte1);
  //      //_debug->printf(" reported error 0x ");
  //      //_debug->printf(byte2, HEX);
  //       break;
  //     case BHY2_META_EVENT_FIFO_OVERFLOW:
  //       //printf("%s; T: %u.%09u; FIFO overflow\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" FIFO overflow");
  //       break;
  //     case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
  //       //printf("%s; T: %u.%09u; Dynamic range changed for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Dynamic range changed for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_FIFO_WATERMARK:
  //       //printf("%s; T: %u.%09u; FIFO watermark reached\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" FIFO watermark reached");
  //       break;
  //     case BHY2_META_EVENT_INITIALIZED:
  //       //printf("%s; T: %u.%09u; Firmware initialized. Firmware version %u\r\n", event_text, s, ns,
  //             //((uint16_t )byte2 << 8) | byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Firmware initialized. Firmware version ");
  //      //_debug->printf(((uint16_t )byte2 << 8) | byte1);
  //       break;
  //     case BHY2_META_TRANSFER_CAUSE:
  //       //printf("%s; T: %u.%09u; Transfer cause for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Transfer cause for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_SENSOR_FRAMEWORK:
  //       //printf("%s; T: %u.%09u; Sensor framework event for sensor id %u\r\n", event_text, s, ns, byte1);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Sensor framework event for sensor id ");
  //      //_debug->printf(byte1);
  //       break;
  //     case BHY2_META_EVENT_RESET:
  //       //printf("%s; T: %u.%09u; Reset event\r\n", event_text, s, ns);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Reset event");
  //       break;
  //     case BHY2_META_EVENT_SPACER:
  //       break;
  //     default:
  //       //printf("%s; T: %u.%09u; Unknown meta event with id: %u\r\n", event_text, s, ns, meta_event_type);
  //      //_debug->printf(event_text);
  //      //_debug->printf(" Unknown meta event with id: ");
  //      //_debug->printf(meta_event_type);
  //       break;
  //   }
  // }
}

void BoschParser::parseGeneric(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint32_t s, ns;
  convertTime(*callback_info->time_stamp, &s, &ns);

  printf("SID: %u; T: %u.%09u; ", callback_info->sensor_id, (unsigned int)s, (unsigned int)ns);
  for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
  {
      printf("%X ", callback_info->data_ptr[i]);
  }
  printf("\r\n");
}

void BoschParser::parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
  uint32_t s, ns;
  convertTime(*callback_info->time_stamp, &s, &ns);
  //if (_debug)//_debug->printf("Debug message: ");
  // if (_debug) {
  //  //_debug->printf("[DEBUG MSG]; flag: 0x");
  //  //_debug->printf(callback_info->data_ptr[0]);
  //  //_debug->printf(" data: ");
  //  //_debug->printf((char*)&callback_info->data_ptr[1]);
  // }
  //printf("[DEBUG MSG]; T: %u.%09u; flag: 0x%x; data: %s\r\n",
        //s,
        //ns,
        //callback_info->data_ptr[0],
        //&callback_info->data_ptr[1]);
}
