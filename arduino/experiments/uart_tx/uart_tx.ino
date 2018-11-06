

// http://gammon.com.au/Arduino/SendOnlySoftwareSerial.zip
#include <SendOnlySoftwareSerial.h>

#define PIN_SOFT_SERIAL_TX 0

SendOnlySoftwareSerial SoftSerial(PIN_SOFT_SERIAL_TX);

int num = 0;

void setup() {
  SoftSerial.begin(9600);

}

void loop() {
  SoftSerial.print("id: ");
  SoftSerial.println(num++);
  delay(1000);
}
