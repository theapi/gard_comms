

// http://gammon.com.au/Arduino/SendOnlySoftwareSerial.zip
#include <SendOnlySoftwareSerial.h>

#define PIN_SOFT_SERIAL_TX 11
#define PAYLOAD_SIZE 4

SendOnlySoftwareSerial SoftSerial(PIN_SOFT_SERIAL_TX);

uint8_t payload[PAYLOAD_SIZE];
uint16_t temperature = 0;

void setup() {
  SoftSerial.begin(9600);
}

void loop() {
  payload[0] = '\t'; // Payload start byte
  payload[1] = B00001010; // Flags
  payload[2] = (temperature >> 8);
  payload[3] = temperature;

  SoftSerial.write(payload, PAYLOAD_SIZE);

  temperature++;
  delay(1000);
}
