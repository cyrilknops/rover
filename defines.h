
//CHANNEL MAPPING
#define CH_AIL 2
#define CH_ELE 3
#define CH_THR 1
#define CH_RUD 4
#define CH_AUX1 5
#define CH_AUX2 6
#define CH_AUX3 7
//INPUT
#define SBUS_IN A8
#define HI 1800
#define MID 1500
#define LO 1200
//OUTPUT
#define OUT1 12 
#define OUT2 11 
#define OUT3 8 
#define OUT4 7 
#define OUT5 6 
#define OUT6 3 
#define OUT7 2
#define OUT8 5
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

//SETTINGS
#define STEERINGTRIM -100
#define FAILSAFE 1500
#define DEBUG
