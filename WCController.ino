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
//M2_EXTERN_ALIGN(el_fan_diag);
//M2_EXTERN_ALIGN(el_cont_diag);

//Some waiting variables for the various loops
uint8_t temp_delay;
uint8_t screen_delay;
uint8_t backlight_delay = 500; //default to polling backlight every 500ms ish
uint8_t fan_delay;

//some boolean state variables
boolean backlight_on = false;
uint8_t backlightmenu = 1; //this is needed as uint due to the way m2tk's toggles work. 

//Pad resistor - maybe integrate this into channel struct.
const float pad = 10000;

//Array of sensor pins and associated names
const byte sensorpins[4] = {AMBIENTPIN, WATER1PIN, WATER2PIN, CASEPIN}
const char sensornames[4][7] = { "Ambient", "Water 1", "Water 2", "Case   " };
int temperatures[4]; //declare as int because we need to multiply by 100 to maintain accuracy whilst not using floats too much.



//struct to describe output channels
struct channel
{
	char name[9]; // what the name is
	byte pin; // what the output pin is
	boolean temp_controlled; //whether we're on temp control
	byte min_duty_cycle; //minimum allowed duty cycle
	byte starting_temp; //starting temperature delta
	byte full_temp; //full speed over and above this
	byte absolute_max; //full speed at this water temp regardless
	byte linked_sensor; //which sensor is linked (index to array)
};

//menu variables to adjust the above struct
char fan_diag_name[9];
uint8_t fan_diag_temp_ctrl;
uint8_t	fan_diag_min_duty;
uint8_t fan_diag_start_temp;
uint8_t fan_diag_full_temp;
uint8_t fan_diag_abs_temp;
uint8_t fan_diag_linked_sensor;


//setup dialog
M2_LABEL(el_fan_diag_l1, NULL, "Temp Control");
M2_TOGGLE(el_fan_diag_t1, "", &fan_diag_temp_ctrl);
M2_LABEL(el_fan_diag_l2, NULL, "Start Delta");
M2_U8NUM(el_fan_diag_u1, "c2", 0, 99, &fan_diag_start_temp);
M2_LABEL(el_fan_diag_l3, NULL, "Max Temp");
M2_U8NUM(el_fan_diag_u2, "c2", 0, 99, &fan_diag_full_temp);
M2_LABEL(el_fan_diag_l4, NULL, "Min Duty");
M2_U8NUM(el_fan_diag_u3, "c3", 0, 100, &fan_diag_min_duty);
M2_LABEL(el_fan_diag_l5, NULL, "Abs Max Temp");
M2_U8NUM(el_fan_diag_u4, "c3", 0, 100, &fan_diag_abs_temp);
M2_LABEL(el_fan_diag_l6, NULL, "Linked Sensor");
M2_COMBO(el_fan_diag_c1, NULL, &fan_diag_linked_sensor, 4, fn_idx_to_sensor);
M2_BUTTON(el_fan_diag_btn, "", "Back", back_to_menu);
M2_LIST(fandiaglist1) = {
	&el_fan_diag_l1, &el_fan_diag_t1, &el_fan_diag_l2, &el_fan_diag_u1, &el_fan_diag_l3, &el_fan_diag_u2, &el_fan_diag_l4, &el_fan_diag_u3, &el_fan_diag_l5, &el_fan_diag_u4, &el_fan_diag_l6,&el_fan_diag_c1, &el_fan_diag_btn);

M2_GRIDLIST(el_fan_diag_grid, "c2", fandiaglist);
M2_VSB(el_fan_diag_vsb, "w16h4l4", 0, 7);
M2_LIST(fandiaglist2) = { el_fan_diag_grid, el_fan_diag_vsb);
M2_HLIST(el_fan_diag, "", &elfandiaglist2);


void back_to_menu(m2_el_fnarg_p fnarg) {
}

const char *fn_idx_to_sensor(uint8_t idx) {
}

void setup() {
	m2_SetNewLiquidCrystal(&lcd, 20, 4);
	// define button for the select message
	m2.setPin(M2_KEY_SELECT, BTNPIN);

	// The incremental rotary encoder is conected to these two pins
	m2.setPin(M2_KEY_ROT_ENC_A, DTPIN); //DT
	m2.setPin(M2_KEY_ROT_ENC_B, CLKPIN); //CLK
	//set the backlight pin, but let the backlight worker update it.
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
}
void loop() {
	m2.checkKey();
	if (m2.handleKey()) {
		m2.draw();
	}
	}

