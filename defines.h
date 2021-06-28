
//CHANNEL MAPPING
#define CH_AIL 2
#define CH_ELE 3
#define CH_THR 1
#define CH_RUD 4
#define CH_AUX1 5
#define CH_AUX2 6
#define CH_AUX3 7
#define CH_RSSI 8
//INPUT
#define SBUS_IN A8
#define HI 1800
#define MID 1500
#define LO 1200
#define str_input 14
#define thr_input 12
#define aux_input 13
//OUTPUT
#define OUT1 19 
#define OUT2 18 
#define OUT3 -1 
#define OUT4 -1 
#define OUT5 -1 
#define OUT6 -1 
#define OUT7 -1
#define OUT8 -1
//OUTPUT MAP
#define MOTOR1 OUT1
#define MOTOR2 -1
#define STEERING OUT2
//OUTPUT SYSTEM
# define A_LED_PIN        27  //red
# define B_LED_PIN        26  //yellow
# define C_LED_PIN        25  //blue
# define LED_ON           LOW
# define LED_OFF          HIGH
# define USB_MUX_PIN      23
# define LIGHTS           A0
//senors
#define VOLTAGE_IN 30
//SETTINGS
#define STEERINGTRIM -100
#define FAILSAFE 1500
#define DEBUG			//uncomment for debug
#define DEBUG_PORT Serial
//#define NEEDGPSTOARM	//uncomment for 3dfix before arm

//radio
bool failSafe;
bool lostFrame;

int motor1CH = 1500;
int steeringCH = 1500;
int armedCH;
int modeCH;
int auxCH;
int rssiCH;

//pid
double Kp=1, Ki=1, Kd=0;
double Setpoint, LastSetpoint, Input, Output;

//accel
int level;

//gps
bool homepoint = false;
float speedKm;
float setSpeed;
bool gpsfix;

//djifpv
HardwareSerial &mspSerial = Serial2;
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

//voltage sensor
float R1 = 9900.0; //resistor 1   
float R2 = 4650.0; //resistor 2
  
//flags
bool arm = false;
int mode = 0;
int aux = 0;
int arming = 0;

//debug
char* modes[10] = {"manual", "cruise control", "no wheely"};
