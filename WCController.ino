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
const byte sensorpins[4] = { AMBIENTPIN, WATER1PIN, WATER2PIN, CASEPIN };
const char sensornames[4][8] = { "Ambient", "Water 1", "Water 2", "Case  " };
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

//setup dialog in two pages (two dialogs really)
M2_LABEL(el_fan_diag_l1, NULL, "Temp Control");
M2_TOGGLE(el_fan_diag_t1, "", &fan_diag_temp_ctrl);
M2_LABEL(el_fan_diag_l2, NULL, "Start Delta");
M2_U8NUM(el_fan_diag_u1, "c2", 0, 99, &fan_diag_start_temp);
M2_LABEL(el_fan_diag_l3, NULL, "Max Temp");
M2_U8NUM(el_fan_diag_u2, "c2", 0, 99, &fan_diag_full_temp);
//arrange to a c2 gridlist
M2_LIST(fandiagoptions1) = { &el_fan_diag_l1, &el_fan_diag_t1, &el_fan_diag_l2, &el_fan_diag_u1, &el_fan_diag_l3, &el_fan_diag_u2 };
M2_GRIDLIST(el_fan_diag_1_labels, "c2", fandiagoptions1);

//labels for second page
M2_LABEL(el_fan_diag_l4, NULL, "Min Duty");
M2_U8NUM(el_fan_diag_u3, "c3", 0, 100, &fan_diag_min_duty);
M2_LABEL(el_fan_diag_l5, NULL, "Abs Max Temp");
M2_U8NUM(el_fan_diag_u4, "c3", 0, 100, &fan_diag_abs_temp);
M2_LABEL(el_fan_diag_l6, NULL, "Linked Sensor");
M2_COMBO(el_fan_diag_c1, NULL, &fan_diag_linked_sensor, 4, fn_idx_to_sensor);
//arrange to a c2 gridlist
M2_LIST(fandiagoptions2) = { &el_fan_diag_l4, &el_fan_diag_u3, &el_fan_diag_l5, &el_fan_diag_u4, &el_fan_diag_l6, &el_fan_diag_c1 };
M2_GRIDLIST(el_fan_diag_2_labels, "c2", fandiagoptions2);

//buttons for  dialog back/next/cancel/commit
M2_BUTTON(el_fan_diag_next, "", "Next", fn_fan_diag_next);
M2_BUTTON(el_fan_diag_back, "", "Prev", fn_fan_diag_back);
M2_BUTTON(el_fan_diag_cancel, "", "Canc", fn_fan_diag_cancel);
M2_BUTTON(el_fan_diag_commit, "", "Done", fn_fan_diag_commit);
//arrange to a 3 column gridlist for page 1
M2_LIST(page1buttons) = { &el_fan_diag_next, &el_fan_diag_commit, &el_fan_diag_cancel };
M2_GRIDLIST(el_fan_page_1_btns, "c3", page1buttons);

//and page 2
M2_LIST(page2buttons) = { &el_fan_diag_back, &el_fan_diag_commit, &el_fan_diag_cancel };
M2_GRIDLIST(el_fan_page_2_btns, "c3", page2buttons);

//assemble pages
M2_LIST(page1) = { &el_fan_diag_1_labels, &el_fan_page_1_btns };
M2_LIST(page2) = { &el_fan_diag_2_labels, &el_fan_page_2_btns };
M2_ALIGN(el_fan_page1, "W64H64", page1);
M2_ALIGN(el_fan_page2, "W64H64", page2);


void back_to_menu(m2_el_fnarg_p fnarg) {
}

const char *fn_idx_to_sensor(uint8_t idx) {
}
void fn_fan_diag_next(m2_el_fnarg_p fnarg) {

}
void fn_fan_diag_back(m2_el_fnarg_p fnarg) {

}
void fn_fan_diag_cancel(m2_el_fnarg_p fnarg) {

}
void fn_fan_diag_commit(m2_el_fnarg_p fnarg) {

}
//definition of root menu
m2_xmenu_entry xmenu_data[] =
{
	{
		"Fan Channel 1", NULL, NULL }
		,		/* expandable main menu entry */
				{
				". Settings", NULL, fn_fan1_load_diag }
				,		/* function opens fan 1 dialog */
				{
					"Fan Channel 2", NULL, NULL }
					,
						/* The label of this menu line is returned by the callback procedure */
						{
							". Settings", NULL, fn_fan2_load_diag }
							,		/* function opens fan 1 dialog */
							{
								"Controller", NULL, fn_cont_load_diag } // goes straight to controller dialo
								,
								{
									"Hide Menu", NULL, fn_menu_hide }
									,      //function hides the menus.
									{
										NULL, NULL, NULL }
										,
};
uint8_t el_x2l_first = 0;
uint8_t el_x2l_cnt = 3;
const char *fn_fan1_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&el_fan_page1);
		lcd.clear();
	}

	return "";
}
const char *fn_fan2_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&el_fan_page2);
		lcd.clear();
	}
	return "";
}
//load the controller settings diag
const char *fn_cont_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		//m2.setRoot(&el_cont_diag);
		lcd.clear();
	}

	return "";
}
const char *fn_menu_hide(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&m2_null_element);
		lcd.clear();
	}
	return "";
}

M2_X2LMENU(el_x2l_main, "l4e1w14", &el_x2l_first, &el_x2l_cnt, xmenu_data, '+', '-', '\0');
M2_VSB(el_x2l_vsb, "l5W1r1", &el_x2l_first, &el_x2l_cnt);
M2_LIST(list_x2l) = {
	&el_x2l_main, &el_x2l_vsb };
M2_HLIST(el_wc_menu, NULL, list_x2l);
// m2 object and constructor
// Note: Use the "m2_eh_4bd" handler, which fits better to the "m2_es_arduino_rotary_encoder" 

M2tk m2(&el_fan_page1, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_nlc);

void setup() {
	m2_SetNewLiquidCrystal(&lcd, 20, 4);

	// define button for the select message
	m2.setPin(M2_KEY_SELECT, BTNPIN);

	// The incremental rotary encoder is conected to these two pins
	m2.setPin(M2_KEY_ROT_ENC_A, DTPIN); //DT
	m2.setPin(M2_KEY_ROT_ENC_B, CLKPIN); //CLK
	//set the backlight pin, but let the backlight worker update it.
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
//	lcd.createChar(0, degree);
	//Serial.begin(9600);
	//begin code to change PWM frequency on Timer 2 HT bens @ arduino forums.
	TCCR2A = 0x23;
	TCCR2B = 0x09;  // select clock
	OCR2A = 79;  // aiming for 25kHz
	pinMode(CHANNEL1PIN, OUTPUT);  // enable the PWM output (you now have a PWM signal on digital pin 3)
	OCR2B = 79;  // set the PWM duty cycle to 100%
}

void loop() {
	//split the workers off into separate functions for ease of reading and sensibleness.
//	worker_monitortemps();
	//  worker_debug();
	//worker_reporting();
	//worker_backlight();
	m2.checkKey();
	if (m2.handleKey()) {
		m2.draw();
	}

}
