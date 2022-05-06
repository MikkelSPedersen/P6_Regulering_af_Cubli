#include <VidorFPGA.h>
#define potIn 4

void setup() {
  pinMode(1, OUTPUT);
  pinMode(potIn, INPUT);
  digitalWrite(1, LOW);
  FPGA.begin();
  FPGA.analogWriteResolution(10, 4700);
  FPGA.pinMode(45, 3);
  digitalWrite(1, HIGH);
  FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
}

void loop() {
  if (digitalRead(potIn) == LOW ) {
    FPGA.analogWrite(45, map(60, 0, 100, pow(2, 10), 0));
  } else {
    FPGA.analogWrite(45, map(50, 0, 100, pow(2, 10), 0));
  }
}
