//Code for Menu Based water cooling control system. Provides for 2 channels of High speed (25khz) PWM output & 2 Channels low speed (490 hz) PWM output.
//either should funtion for fan PWM control. Only channels 1 & 2 are suitable for control of MOSFETs (IRL510N tested) for voltage control (lower voltage outputs will work but sing).
//
//Includes
#include <EEPROMVar.h>
#include <EEPROMex.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "M2tk.h"
#include <math.h>
#include "utility/m2ghnlc.h"

//Analog Pins for Thermistor Connections
#define AMBIENTPIN 0
#define WATER1PIN 1
#define WATER2PIN 2
#define CASEPIN 3

//PWM Pins for Channel 3 & 4 (1 and 2 will always be pin 3 (channel 1) and pin 5 (channel 2) as they're tied to timer 2
#define CHANNEL1PIN 3
#define CHANNEL2PIN 5
#define CHANNEL3PIN 5
#define CHANNEL4PIN 6
//PINS for Rotary encoder
#define DTPIN 12
#define CLKPIN 9
#define BTNPIN 10

//Liquid Crystal Initialisation
#define I2C_ADDR    0x27  // Define I2C Address where the SainSmart LCD is I2C pins A4 (SDA) & A5 (SCL
//below here specific to LCD. Try without first.
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7
LiquidCrystal_I2C	lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

//=================================================
// Forward declaration of the menu elements
extern M2tk m2;
M2_EXTERN_ALIGN(el_f1_diag);
M2_EXTERN_ALIGN(el_f2_diag);
M2_EXTERN_ALIGN(el_cont_diag);

//Some waiting variables for the various loops
uint8_t temp_delay;
uint8_t screen_delay;
uint8_t backlight_delay = 500; //default to polling backlight every 500ms ish
uint8_t fan_delay;

//some boolean state variables
boolean backlight_on = true;

//Pad resistor - maybe integrate this into channel struct.
const float pad = 10000;

struct sensor
{
	byte pin;
	uint8_t pad = 10000;
};

struct channel
{
	char name[9];

};


