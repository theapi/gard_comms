

#include <SoftwareSerial.h>

SoftwareSerial SoftSerial(10, 11); // RX, TX

int num = 0;

void setup() {
  SoftSerial.begin(9600);

}

void loop() {
  SoftSerial.print("ID: ");
  SoftSerial.println(num++);
  delay(1000);
}
