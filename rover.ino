#include <Servo.h>
#include "defines.h"
#include <sbus.h>
#include "src/R_PID.h"
#include "src/Timer.h"
#include "src/MPU6000.h"
#include <NMEAGPS.h>
#include <GPSport.h>
#include <MSP.h>
#include "MSP_OSD.h"
#include "flt_modes.h"
#include "OSD_positions_config.h"

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
float speedKm;
float setSpeed;
bool gpsfix;

//djifpv
HardwareSerial &mspSerial = Serial3;
MSP msp;
uint8_t vbat = 0;
uint8_t batteryCellCount = 0;
char craftname[15] = "curiosity";
uint8_t numSat = 0;
int32_t gps_lon = 0;
int32_t gps_lat = 0;
int32_t gps_alt = 0;
int32_t gps_home_lon = 0;
int32_t gps_home_lat = 0;
int32_t gps_home_alt = 0;
uint32_t distanceToHome = 0;    // distance to home in meters
int16_t directionToHome = 0;   // direction to home in degrees
uint8_t thr_position = 0;
uint8_t set_home = 1;

//flags
bool arm = false;
int mode = 0;

//timer
Timer timer(10);
Timer debugT(500);
Timer second(1000);
Timer dji(100);
//debug
char* modes[10] = {"manual", "no wheely", "speed limiter"};

void setup() {
  DEBUG_PORT.begin(9600);
  //sbus
  sbus.begin(SBUS_IN, sbusNonBlocking);
  //servos
  steering.attach(STEERING);
  motor.attach(MOTOR1);
  //DJI FPV
  mspSerial.begin(115200);
  msp.begin(mspSerial);
  //accel
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH);
  delay(100);
  accel.Configure();  // configure chip

  //gps
  DEBUG_PORT.println( F("Looking for GPS device on " GPS_PORT_NAME) );
  gpsPort.begin(9600);

  //lights
  pinMode(LIGHTS, OUTPUT);
  pinMode(A_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
  pinMode(C_LED_PIN, OUTPUT);
}

void loop() {
  getChannels();
  switch(getMode()){
    case 1:
      antiwheel();
      break;
    case 2:
      cruisecontrol(5); //give max speed in km 
      break;
    default:
      break;
  }
  armed();
  GPSloop();
  setOutputs();
  #ifdef DEBUG
    debug();
  #endif
  if(dji.checkT()){
    vbat = random(109,124);
    send_msp_to_airunit();
  }
  if(batteryCellCount == 0 && vbat > 0)set_battery_cells_number();
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
int getMode(){
  if(modeCH >= HI){
      mode = 1;
  }else if(modeCH == MID){
      mode = 2;
  }else if(modeCH <= LO){
    mode = 0;
  }
  if(auxCH >= HI){
    digitalWrite(LIGHTS, HIGH);
  }else{
     digitalWrite(LIGHTS, LOW);
  }
  return mode;
}
int limitor(int pwm, int minVal, int maxVal){
  return map(pwm, 1000, 2000, minVal, maxVal);
}
void cruisecontrol(float speed){
    // if(speedKm > speed){
    //   motor1CH = 1500;
    // }
    DEBUG_PORT.print("CH-IN:");
    DEBUG_PORT.print(motor1CH);
    DEBUG_PORT.print("PID-In:");
    DEBUG_PORT.print(speedKm);
    DEBUG_PORT.print(" PID-Out:");
    Output = pid.calculate(speed, speedKm);
    DEBUG_PORT.print(Output);
    motor1CH = motor1CH - Output*50;
    if(motor1CH < 1500)
      motor1CH = 1500;
    DEBUG_PORT.print(" CH-Out:");
    DEBUG_PORT.println(motor1CH);
}
void antiwheel(){
  if(motor1CH > 1500){
    DEBUG_PORT.print("CH-IN:");
    DEBUG_PORT.print(motor1CH);
    DEBUG_PORT.print("PID-In:");
    DEBUG_PORT.print(accel.AcceDeg(2)-level);
    DEBUG_PORT.print(" PID-Out:");
    Output = pid.calculate(level, (accel.AcceDeg(2)-level));
    DEBUG_PORT.print(Output*10);
    motor1CH = motor1CH - Output;
    if(motor1CH < 1500)
      motor1CH = 1500;
    DEBUG_PORT.print(" CH-Out:");
    DEBUG_PORT.println(motor1CH);
  }
}
void armed(){
  #ifndef NEEDGPSTOARM
    gpsfix = true;
  #endif
  if(armedCH > HI && gpsfix){
    arm = true;
  }else{
    arm = false;
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
  if(gps.available( gpsPort )) {
    fix = gps.read();
    if(fix.valid.location){
      gpsfix = true;
      digitalWrite(A_LED_PIN, LED_ON);
//      float range = fix.location.DistanceKm( base );
//      DEBUG_PORT.print( F("Range: ") );
//      DEBUG_PORT.print( range );
//      DEBUG_PORT.println( F(" Km") );
      speedKm = fix.speed_kph();
      DEBUG_PORT.print( F("speed: ") );
      DEBUG_PORT.print( speedKm );
      DEBUG_PORT.println( F(" Km/h") );
    }else{
      gpsfix = false;
      DEBUG_PORT.print(".");
      digitalWrite(A_LED_PIN,LED_OFF);
    }
  }
}
static void debug(){
  if(debugT.checkT()){
  DEBUG_PORT.print("Armed: ");
  DEBUG_PORT.print(arm);
  DEBUG_PORT.print(" Mode: ");
  DEBUG_PORT.print(modes[mode]);
  DEBUG_PORT.print(" Channels: ");
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
msp_analog_t analog = {0};
msp_battery_state_t battery_state = {0};
msp_name_t name = {0};
msp_raw_gps_t raw_gps = {0};
msp_comp_gps_t comp_gps = {0};

void send_msp_to_airunit()
{

    //MSP_FC_VERSION
    // fc_version.versionMajor = 4;
    // fc_version.versionMinor = 1;
    // fc_version.versionPatchLevel = 1;
    // msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
    //MSP_NAME
    memcpy(name.craft_name, craftname, sizeof(craftname));
    msp.send(MSP_NAME, &name, sizeof(name));
    //MSP_ANALOG
    analog.vbat = vbat;
    msp.send(MSP_ANALOG, &analog, sizeof(analog));

    //MSP_BATTERY_STATE
    battery_state.batteryVoltage = vbat * 10;
    battery_state.batteryCellCount = batteryCellCount;
    //battery_state.batteryState = batteryState;
    battery_state.legacyBatteryVoltage = vbat;
    msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

    //MSP_RAW_GPS
    raw_gps.lat = fix.latitude();
    raw_gps.lon = fix.longitude();
    raw_gps.alt = fix.altitude() / 10;
    raw_gps.groundSpeed = (int16_t)(fix.speed_kph()* 100);
    msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));

    //MSP_COMP_GPS
    comp_gps.distanceToHome = (int16_t)fix.location.DistanceKm( base );;
    comp_gps.directionToHome = fix.location.BearingToDegrees(base);
    msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));
    
    //MSP_OSD_CONFIG
    send_osd_config();
}

msp_osd_config_t msp_osd_config = {0};

void send_osd_config()
{
  
#ifdef IMPERIAL_UNITS
    msp_osd_config.units = 0;
#else
    msp_osd_config.units = 1;
#endif

    msp_osd_config.osd_item_count = 56;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;              // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             //  0

    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    //msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;

    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}
