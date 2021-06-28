#include <ESP32Servo.h>
//#include <sbus.h>
#include <driver/adc.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <MSP.h>
#include "src/R_PID.h"
#include "src/Timer.h"
#include "src/MSP_OSD.h"
#include "src/flt_modes.h"
#include "defines.h"
#include "OSD_positions_config.h"

//output
Servo motor;
Servo steering;


//pid
R_PID pid(Kp, Ki, Kd, 500, -500);


//timer
Timer timer(9);
Timer debugT(500);
Timer second(1000);
Timer dji(100);
//gps
static NMEAGPS  gps;
static gps_fix  fix;
//NeoGPS::Location_t base( 508296400L, 55396607L );

void setup() {
  DEBUG_PORT.begin(115200);
  //inputs
  pinMode(str_input, INPUT);
  pinMode(thr_input, INPUT);
  pinMode(aux_input, INPUT);
  //servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  steering.setPeriodHertz(50);
  motor.setPeriodHertz(50);
  steering.attach(STEERING);
  motor.attach(MOTOR1);
  //DJI FPV
  mspSerial.begin(115200);
  msp.begin(mspSerial);
  //gps
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  gpsPort.begin(9600);
  //lights
  pinMode(LIGHTS, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
  pinMode(C_LED_PIN, OUTPUT);
  //voltage
  adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
  analogReadResolution(12);
  pinMode(VOLTAGE_IN, INPUT);
}

void loop() {
  
  if(timer.checkT()){
      getChannels();
      armed();

  }
  if(dji.checkT()){
    getSwitches();
    auxiliary();
  }
  switch(mode){
    case 1:
      break;
    case 2:
      cruisecontrol(10); //give max speed in km 
      break;
    default:
      break;
  }
  
  GPSloop();
  setOutputs();
  if(dji.checkT()){
    vbat = mapfloat(getVoltage(), 0, 25.5, 0, 255);
    send_msp_to_airunit();
  }
  #ifdef DEBUG
    debug();
  #endif
  delay(20);
}
void set_battery_cells_number()
{
    if(vbat < 43)batteryCellCount = 1;
    else if(vbat < 85)batteryCellCount = 2;
    else if(vbat < 127)batteryCellCount = 3;
    else if(vbat < 169)batteryCellCount = 4;
    else if(vbat < 211)batteryCellCount = 5;
    else if(vbat < 255)batteryCellCount = 6;
}
void getSwitches(){

  if(modeCH >= HI){
      mode = 2;
  }else if(modeCH == MID){
      mode = 1;
  }else if(modeCH <= LO){
      mode = 0;
  }
  if(auxCH >= HI){
      aux = 2;
  }else if(auxCH == MID){
      aux = 1;
  }else if(auxCH <= LO){
      aux = 0;
  }
  if(auxCH >= HI){
      aux = 2;
  }else if(auxCH == MID){
      aux = 1;
  }else if(auxCH <= LO){
      aux = 0;
  }
  if(armedCH >= HI){
    arming = 2;
  }else if(armedCH == MID){
    arming = 1;
  }else if(armedCH <= LO){
    arming = 0;
  }
}
int limitor(int pwm, int minVal, int maxVal){
  return map(pwm, 1000, 2000, minVal, maxVal);
}
void getChannels(){
  motor1CH = pulseIn(thr_input, HIGH);
  steeringCH = pulseIn(str_input, HIGH);
  armedCH = pulseIn(aux_input, HIGH);
}
void setOutputs(){
  steering.writeMicroseconds(steeringCH+STEERINGTRIM);
  //motor.writeMicroseconds(limitor(motor1CH,1300,1700));
  motor.writeMicroseconds(motor1CH);
}
