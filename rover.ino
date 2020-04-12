#include <Servo.h>
#include "defines.h"
#include <sbus.h>
#include "R_PID.h"
#include "Timer.h"
#include "MPU6000.h"
//output
Servo motor;
Servo steering;

//radio
SBUS sbus;
bool failSafe;
bool lostFrame;

int motor1CH = 1500;
int steeringCH = 1500;
int armedCH;

//pid
double Kp=1, Ki=1, Kd=0;
double Setpoint, LastSetpoint, Input, Output;
R_PID pid(Kp, Ki, Kd, 500, -500);

//accel
MPU6000 accel(53);
int level;

//timer
Timer timer(10);

void setup() {
  Serial.begin(9600);
  sbus.begin(SBUS_IN, sbusNonBlocking);
  steering.attach(STEERING);
  motor.attach(MOTOR1);

  //As per APM standard code, stop the barometer from holding the SPI bus
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  
  delay(100);

  accel.Configure();  // configure chip

  //turn the PID on

}

void loop() {
  getChannels();
  if(timer.checkT()){
    if(sbus.getChannel(CH_AUX2) >= 1800){
      if(motor1CH > 1500){
        antiwheel();
      }
    }
  }
  if(sbus.getChannel(CH_AUX3) >= 1800){
    level = accel.AcceDeg(2);
  }
  armed();
  setOutputs();
  delay(20);
}
int limitor(int pwm, int minVal, int maxVal){
  return map(pwm, 1000, 2000, minVal, maxVal);
}
void antiwheel(){
  Serial.print("CH-IN:");
  Serial.print(motor1CH);
  Serial.print("PID-In:");
  Serial.print(accel.AcceDeg(2)-level);
  Serial.print(" PID-Out:");
  Output = pid.calculate(level, (accel.AcceDeg(2)-level)*10);
  Serial.print(Output);
  motor1CH = motor1CH - Output;
  if(motor1CH < 1500)
    motor1CH = 1500;
  Serial.print(" CH-Out:");
  Serial.println(motor1CH);
}
void armed(){
  if(armedCH < 1800){
    motor1CH = 1500;
  }
}
void getChannels(){
  motor1CH = sbus.getChannel(CH_ELE);
  steeringCH = sbus.getChannel(CH_RUD);
  armedCH = sbus.getChannel(CH_AUX1);
}
void setOutputs(){
  steering.writeMicroseconds(steeringCH+STEERINGTRIM);
  //motor.writeMicroseconds(limitor(motor1CH,1300,1700));
  motor.writeMicroseconds(motor1CH);
}
