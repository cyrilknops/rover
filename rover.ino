#include <Servo.h>
#include "defines.h"
#include <sbus.h>

Servo motor;
Servo steering;

SBUS crsf;

bool failSafe;
bool lostFrame;


void setup() {
  Serial.begin(9600);
  crsf.begin(OUT1, sbusNonBlocking);
  steering.attach(OUT3);
  motor.attach(OUT2);
}

void loop() {
  steering.writeMicroseconds(crsf.getChannel(CH_RUD)+STEERINGTRIM);
  motor.writeMicroseconds(limitor(crsf.getChannel(CH_ELE),1300,1700));
  delay(20);
}

int limitor(int pwm, int minVal, int maxVal){
  return map(pwm, 1000, 2000, minVal, maxVal);
}
