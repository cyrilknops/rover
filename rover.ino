#include <Servo.h>
#include "defines.h"
#include <sbus.h>
#include "R_PID.h"
#include "Timer.h"
#include "MPU6000.h"
#include <NMEAGPS.h>
#include <GPSport.h>

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
int modeCH;
int auxCH;

//pid
double Kp=1, Ki=1, Kd=0;
double Setpoint, LastSetpoint, Input, Output;
R_PID pid(Kp, Ki, Kd, 500, -500);

//accel
MPU6000 accel(53);
int level;

//gps
static NMEAGPS  gps;
static gps_fix  fix;
NeoGPS::Location_t base( 508296400L, 55396607L );
bool homepoint = false;

//timer
Timer timer(10);
Timer debugT(500);
void setup() {
  DEBUG_PORT.begin(9600);
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  sbus.begin(SBUS_IN, sbusNonBlocking);
  steering.attach(STEERING);
  motor.attach(MOTOR1);

  //As per APM standard code, stop the barometer from holding the SPI bus
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  delay(100);
  accel.Configure();  // configure chip

  //gps
  gpsPort.begin(9600);

  //lights
  pinMode(LIGHTS, OUTPUT);
}

void loop() {
  getChannels();
  if(timer.checkT()){
    if(modeCH >= HI){
      if(motor1CH > 1500){
        antiwheel();
      }
    }
  }
  if(auxCH >= HI){
    //level = accel.AcceDeg(2);
    digitalWrite(LIGHTS, HIGH);
  }else{
     digitalWrite(LIGHTS, LOW);
  }
  armed();
  GPSloop();
  setOutputs();
  #ifdef DEBUG
    debug();
  #endif
  delay(20);
}
int limitor(int pwm, int minVal, int maxVal){
  return map(pwm, 1000, 2000, minVal, maxVal);
}
void antiwheel(){
  DEBUG_PORT.print("CH-IN:");
  DEBUG_PORT.print(motor1CH);
  DEBUG_PORT.print("PID-In:");
  DEBUG_PORT.print(accel.AcceDeg(2)-level);
  DEBUG_PORT.print(" PID-Out:");
  Output = pid.calculate(level, (accel.AcceDeg(2)-level)*10);
  DEBUG_PORT.print(Output);
  motor1CH = motor1CH - Output;
  if(motor1CH < 1500)
    motor1CH = 1500;
  DEBUG_PORT.print(" CH-Out:");
  DEBUG_PORT.println(motor1CH);
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
  modeCH = sbus.getChannel(CH_AUX2);
  auxCH = sbus. getChannel(CH_AUX3);
}
void setOutputs(){
  steering.writeMicroseconds(steeringCH+STEERINGTRIM);
  //motor.writeMicroseconds(limitor(motor1CH,1300,1700));
  motor.writeMicroseconds(motor1CH);
}
static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    if(fix.valid.location){
//      float range = fix.location.DistanceKm( base );
//      DEBUG_PORT.print( F("Range: ") );
//      DEBUG_PORT.print( range );
//      DEBUG_PORT.println( F(" Km") );
        float speedKm = fix.speed_kph();
        DEBUG_PORT.print( F("speed: ") );
        DEBUG_PORT.print( speedKm );
        DEBUG_PORT.println( F(" Km/h") );
    }else{
      DEBUG_PORT.print(".");
    }
  }
}
static void debug(){
  if(debugT.checkT()){
  DEBUG_PORT.print("Channels: ");
  DEBUG_PORT.print(motor1CH);
  DEBUG_PORT.print(", ");
  DEBUG_PORT.print(steeringCH);
  DEBUG_PORT.print(", ");
  DEBUG_PORT.print(armedCH);
  DEBUG_PORT.print(", ");
  DEBUG_PORT.print(modeCH);
  DEBUG_PORT.print(", ");
  DEBUG_PORT.println(auxCH);
  }
}
