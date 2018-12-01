#include "payload_solar.h"

/*
  uint8_t MessageType;
  uint8_t DeviceId;
  uint8_t MessageId;
  uint8_t Flags;
  uint16_t VCC;
  uint16_t ChargeMv;
  int16_t ChargeMa;
  uint16_t Light;
  int16_t CpuTemperature;
  int16_t Temperature;
 */
void PAYLOAD_Solar_serialize(PAYLOAD_Solar payload, uint8_t buffer[PAYLOAD_Solar_SIZE]) {
    buffer[0] = payload.MessageType;
    buffer[1] = payload.DeviceId;
    buffer[2] = payload.MessageId;
    buffer[3] = payload.Flags;
    buffer[4] = (payload.VCC >> 8);
    buffer[5] = payload.VCC;
    buffer[6] = (payload.ChargeMv >> 8);
    buffer[7] = payload.ChargeMv;
    buffer[8] = (payload.ChargeMa >> 8);
    buffer[9] = payload.ChargeMa;
    buffer[10] = (payload.Light >> 8);
    buffer[11] = payload.Light;
    buffer[12] = (payload.CpuTemperature >> 8);
    buffer[13] = payload.CpuTemperature;
    buffer[14] = (payload.Temperature >> 8);
    buffer[15] = payload.Temperature;
}
