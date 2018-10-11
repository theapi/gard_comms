#ifndef __payload_solar_H
#define __payload_solar_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l0xx_hal.h"
#include "main.h"
#include "stdint.h"

#define PAYLOAD_Solar_SIZE 16

typedef struct {
  uint8_t MessageType;
  uint16_t DeviceId;
  uint8_t MessageId;
  uint16_t VCC;
  uint16_t ChargeMv;
  int16_t ChargeMa;
  uint16_t Light;
  int16_t CpuTemperature;
  int16_t Temperature;
} PAYLOAD_Solar;


// Creates a byte array for sending via the radio
void PAYLOAD_Solar_serialize(PAYLOAD_Solar payload, uint8_t buffer[PAYLOAD_Solar_SIZE]);

// Parse the read byte data from the radio
void PAYLOAD_Solar_unserialize(PAYLOAD_Solar payload, uint8_t buffer[PAYLOAD_Solar_SIZE]);

#ifdef __cplusplus
}
#endif
#endif /*__payload_solar_H */
