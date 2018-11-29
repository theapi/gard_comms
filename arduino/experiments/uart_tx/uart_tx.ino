

// http://gammon.com.au/Arduino/SendOnlySoftwareSerial.zip
#include <SendOnlySoftwareSerial.h>

#define PIN_SOFT_SERIAL_TX 6

SendOnlySoftwareSerial SoftSerial(PIN_SOFT_SERIAL_TX);

int num = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("  ");
  Serial.println("Started");

  SoftSerial.begin(9600);
  SoftSerial.println("\nSOFT Serial started");
  SoftSerial.begin(9600);

}

void loop() {
  Serial.print("id: ");
  Serial.println(num++);
  SoftSerial.print("id: ");
  SoftSerial.println(num);
  delay(1000);
}
