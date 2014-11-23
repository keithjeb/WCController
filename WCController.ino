vo//Code for Menu Based water cooling control system. Provides for 2 channels of High speed (25khz) PWM output & 2 Channels low speed (490 hz) PWM output.
//either should funtion for fan PWM control. Only channels 1 & 2 are suitable for control of MOSFETs (IRL510N tested) for voltage control (lower voltage outputs will work but sing).
//
//Includes
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "M2tk.h"
#include <math.h>
#include "utility/m2ghnlc.h"

//Analog Pins for Thermistor Connections
#define WATER1PIN 2
#define WATER2PIN 1
#define AMBIENTPIN 0
#define CASEPIN 4

//PWM Pins for Channel 3 & 4 (1 and 2 will always be pin 3 (channel 1) and pin 5 (channel 2) as they're tied to timer 2
#define CHANNEL1PIN 3
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

//declare variables
//Polling Frequency
uint8_t update_delay = 5000;
//screen update delay on home screen (in ms) - only applies when on monitoring screen.
uint8_t screen_refresh = 1000;
//storage values for graphics/worker routines to avoid delays.
unsigned long last_through_graphics;
unsigned long last_through_worker;
unsigned long last_through_backlight;
//voltage of pad resistor - better accuraccy if done per channel. 
float pad = 10000;
//variable for toggle menu (it can't understand bool)
uint8_t backlightOn = 1;
//set this flag so we're not doing extra work
boolean boolbackon = false;
//fan parameters
boolean fan1full = false;
boolean fan2full = false;
uint8_t fan1starttemp = 2; //temp delta between ambient & water 1 where fan 1 starts to spin
uint8_t fan1minduty = 15; //min duty cycle for fan channel 1
uint8_t fan1maxtemp = 5; //we'll be at 100% duty by this delta
uint8_t fan1current; //store current duty cycle since we'll probably need it.
uint8_t fan2starttemp = 5;
uint8_t fan2minduty = 15;
uint8_t fan2maxtemp = 15;
uint8_t fan2current;
uint8_t fan3starttemp;
uint8_t fan3minduty;
uint8_t fan3maxtemp;
uint8_t fan3current;
float water1temp;
float water2temp;
float ambienttemp;
float casetemp;

//dialog for Fan 1 Temp Settings
M2_LABEL(el_f1_l1, NULL, "Start Delta");
M2_U8NUM(el_f1_u1, "c2", 0, 99, &fan1starttemp);
M2_LABEL(el_f1_l2, NULL, "Max Delta");
M2_U8NUM(el_f1_u2, "c2", 0, 99, &fan1maxtemp);
M2_LABEL(el_f1_l3, NULL, "Min Duty");
M2_U8NUM(el_f1_u3, "c2", 0, 99, &fan1minduty);
M2_BUTTON(el_f1_btn, "", "Back", back_to_menu);
M2_LIST(f1diaglist) = {
	&el_f1_l1, &el_f1_u1, &el_f1_l2, &el_f1_u2, &el_f1_l3, &el_f1_u3, &el_f1_btn };
M2_GRIDLIST(el_f1_grid, "c2", f1diaglist);
M2_ALIGN(el_f1_diag, "-1|1W64H64", &el_f1_grid);

//dialog for Fan 2 Temp Settings
M2_LABEL(el_f2_l1, NULL, "Start Delta");
M2_U8NUM(el_f2_u1, "c2", 0, 99, &fan2starttemp);
M2_LABEL(el_f2_l2, NULL, "Max Delta");
M2_U8NUM(el_f2_u2, "c2", 0, 99, &fan2maxtemp);
M2_LABEL(el_f2_l3, NULL, "Min Duty");
M2_U8NUM(el_f2_u3, "c2", 0, 99, &fan2minduty);
M2_BUTTON(el_f2_btn, "", "Back", back_to_menu);
M2_LIST(f2diaglist) = {
	&el_f2_l1, &el_f2_u1, &el_f2_l2, &el_f2_u2, &el_f2_l3, &el_f2_u3, &el_f2_btn };
M2_GRIDLIST(el_f2_grid, "c2", f2diaglist);
M2_ALIGN(el_f2_diag, "-1|1W64H64", &el_f2_grid);

////dialog for Fan 3 Temp Settings
//M2_LABEL(el_f3_l1, NULL, "Start Delta");
//M2_U8NUM(el_f3_u1, "c2", 0, 99, &fan3starttemp);
//M2_LABEL(el_f3_l2, NULL, "Max Delta");
//M2_U8NUM(el_f3_u2,"c2", 0, 99,&fan3maxtemp);
//M2_LABEL(el_f3_l3, NULL, "Min Duty");
//M2_U8NUM(el_f3_u3,"c2", 0, 99,&fan3minduty);
//M2_BUTTON(el_f3_btn, "", "Back", back_to_menu);
//M2_LIST(f3diaglist) = { 
//  &el_f3_l1, &el_f3_u1, &el_f3_l2,&el_f3_u2, &el_f3_l3,&el_f3_u3,&el_f3_btn};
//M2_GRIDLIST(el_f3_grid, "c2", f3diaglist);
//M2_ALIGN(el_f3_diag, "-1|1W64H64", &el_f3_grid);
////dialog for Fan 4 Temp Settings
//M2_LABEL(el_f4_l1, NULL, "Start Delta");
//M2_U8NUM(el_f4_u1, "c2", 0, 99, &fan4starttemp);
//M2_LABEL(el_f4_l2, NULL, "Max Delta");
//M2_U8NUM(el_f4_u2,"c2", 0, 99,&fan4maxtemp);
//M2_LABEL(el_f4_l3, NULL, "Min Duty");
//M2_U8NUM(el_f4_u3,"c2", 0, 99,&fan4minduty);
//M2_BUTTON(el_f4_btn, "", "Back", back_to_menu);
//M2_LIST(f4diaglist) = { 
//  &el_f4_l1, &el_f4_u1, &el_f4_l2,&el_f4_u2, &el_f4_l3,&el_f4_u3,&el_f4_btn};
//M2_GRIDLIST(el_f4_grid, "c2", f4diaglist);
//M2_ALIGN(el_f4_diag, "-1|1W64H64", &el_f4_grid);
//Dialog for Controller Settings
M2_LABEL(el_cont_l1, NULL, "Backlight: ");
M2_TOGGLE(el_cont_t1, "c2", &backlightOn);
M2_LABEL(el_cont_l2, NULL, "Update Speed ");
M2_U8NUM(el_cont_u2, "c2", 0, 99, &update_delay);
M2_BUTTON(el_cont_btn, "", "Back", back_to_menu);
M2_LIST(contdiaglist) = {
	&el_cont_l1, &el_cont_t1, &el_cont_l2, &el_cont_u2, &el_cont_btn };
M2_GRIDLIST(el_cont_grid, "c2", contdiaglist);
M2_ALIGN(el_cont_diag, "-1|1W64H64", &el_cont_grid);

//definition of root menu
m2_xmenu_entry xmenu_data[] =
{
	{
		"Fan Channel 1", NULL, NULL }
		,		/* expandable main menu entry */
		{
			".", NULL, fn_fan1_temp_value }
			,		/* The label of this menu line is returned by the callback procedure */
			{
				". Settings", NULL, fn_fan1_load_diag }
				,		/* function opens fan 1 dialog */
				{
					"Fan Channel 2", NULL, NULL }
					,
					{
						". Temp Ctrl", NULL, fn_fan2_temp_value }
						,		/* The label of this menu line is returned by the callback procedure */
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

M2_X2LMENU(el_x2l_main, "l4e1w14", &el_x2l_first, &el_x2l_cnt, xmenu_data, '+', '-', '\0');
M2_VSB(el_x2l_vsb, "l5W1r1", &el_x2l_first, &el_x2l_cnt);
M2_LIST(list_x2l) = {
	&el_x2l_main, &el_x2l_vsb };
M2_HLIST(el_wc_menu, NULL, list_x2l);
// m2 object and constructor
// Note: Use the "m2_eh_4bd" handler, which fits better to the "m2_es_arduino_rotary_encoder" 

M2tk m2(&m2_null_element, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_nlc);

//callbacks for the menus go here:

//Called from hide menu option - set the root to null (i.e. release control of screen)
const char *fn_menu_hide(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&m2_null_element);
		lcd.clear();
	}
	return "";
}
char buf[20];
//return the value for the channel 1 toggle
const char *fn_fan1_temp_value(uint8_t idx, uint8_t msg)
{
	if (msg == M2_STRLIST_MSG_GET_STR) {
		if (fan1full) {
			strcpy(buf, " Temp Ctrl Off ");
			return buf;
		}
		else {
			strcpy(buf, " Temp Ctrl On ");
			return buf;
		}
	}
	//if we were picked toggle it.
	if (msg == M2_STRLIST_MSG_SELECT) {
		if (fan1full) {
			fan1full = false;
		}
		else {
			fan1full = true;
		}

	}
	return "";
}
//Skip to the Fan 1 Input Dialog
const char *fn_fan1_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&el_f1_diag);
		lcd.clear();
	}

	return "";
}
//return the value for the channel 2 toggle
const char *fn_fan2_temp_value(uint8_t idx, uint8_t msg)
{
	if (msg == M2_STRLIST_MSG_GET_STR) {
		if (fan2full) {
			strcpy(buf, " Temp Ctrl Off ");
			return buf;
		}
		else {
			strcpy(buf, " Temp Ctrl On ");
			return buf;
		}
	}
	//if we were picked toggle it.
	if (msg == M2_STRLIST_MSG_SELECT) {
		if (fan2full) {
			fan2full = false;
		}
		else {
			fan2full = true;
		}

	}
	return "";
}
//Skip to the Fan 2 Input Dialog
const char *fn_fan2_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&el_f2_diag);
		lcd.clear();
	}
	return "";
}

//Function to return back to main menu from dialogs.
void back_to_menu(m2_el_fnarg_p fnarg) {
	lcd.clear();
	m2.setRoot(&el_wc_menu);
}
//load the controller settings diag
const char *fn_cont_load_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		m2.setRoot(&el_cont_diag);
		lcd.clear();
	}

	return "";
}
//define a degree sign
byte degree[8] = {
	0b00111,
	0b00101,
	0b00111,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};

void setup() {
	m2_SetNewLiquidCrystal(&lcd, 20, 4);
	// define button for the select message
	m2.setPin(M2_KEY_SELECT, BTNPIN);

	// The incremental rotary encoder is conected to these two pins
	m2.setPin(M2_KEY_ROT_ENC_A, DTPIN); //DT
	m2.setPin(M2_KEY_ROT_ENC_B, CLKPIN); //CLK
	//set the backlight pin, but let the backlight worker update it.
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	lcd.createChar(0, degree);
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
	worker_monitortemps();
	//  worker_debug();
	worker_reporting();
	worker_backlight();
	m2.checkKey();
	if (m2.handleKey()) {
		m2.draw();
	}

}
void worker_backlight(void){
	if (millis() < last_through_backlight + 500) { return; }
	if (backlightOn == 1 && !boolbackon) {
		boolbackon = true;
		lcd.setBacklight(HIGH);
	}
	else if (backlightOn == 0 && boolbackon){
		boolbackon = false;
		lcd.setBacklight(LOW);
	}
	return;
}


int scrap;
int water1delta;
int water2delta;
int casedelta;

void worker_monitortemps(void) {
	//return if we are too quick through the rest - defaults give us a 0.2s polling rate
	if (millis() < last_through_worker + update_delay * 100){
		return;
	}
	//read in all the temps
	water1temp = Thermistor(analogRead(WATER1PIN));
	water2temp = Thermistor(analogRead(WATER2PIN));
	ambienttemp = Thermistor(analogRead(AMBIENTPIN));
	casetemp = 90;//Thermistor(analogRead(AMBIENTPIN));
	//now do some work on them compare delta
	water1delta = (int)(100 * water1temp - 100 * ambienttemp);
	water2delta = (int)(water2temp - ambienttemp);
	casedelta = (int)(casetemp - ambienttemp);
	if (fan1full) {
		fan1current = fan1minduty;
	}
	else {
		fan1current = fan_speed_calc(water1delta, 100 * fan1starttemp, 100 * fan1maxtemp, fan1minduty);
	}
	fan2current = fan_speed_calc(water2delta, fan2starttemp, fan2maxtemp, fan2minduty);
	fan3current = fan_speed_calc(casedelta, fan3starttemp, fan3maxtemp, fan3minduty);
	//adjust fan 1 speed
	OCR2B = 79 * fan1current / 100;

}
void worker_debug(void) {
	lcd.setCursor(0, 0);
	lcd.print("W1 :");
	lcd.print(water1temp);
	lcd.print(" AM :");
	lcd.print(ambienttemp);
	lcd.setCursor(0, 1);
	lcd.print("water1delta :");
	lcd.print(water1delta);
	lcd.setCursor(0, 2);
	lcd.print(fan1starttemp);
	lcd.print(" ");
	lcd.print(fan1maxtemp);
	lcd.print(" ");
	lcd.print(fan1minduty);
	lcd.setCursor(0, 3);
	lcd.print(fan1current);
	delay(3000);
}

int fan_speed_calc(int currentdelta, int mindelta, int maxdelta, int minduty) {

	if (currentdelta < mindelta) {
		return 0;
	}
	else if (currentdelta > maxdelta) {
		return 100;
	}
	else{
		scrap = 100 * (currentdelta - mindelta) / (maxdelta - mindelta);
		//    lcd.setCursor (8,2);
		//    lcd.print(scrap);
		//    lcd.setCursor (8,3);
		//    lcd.print(minduty);
		if (scrap < minduty) {
			return minduty;
		}
		return scrap;
	}

}

float Thermistor(int RawADC) {
	long Resistance;
	float Temp;  // Dual-Purpose variable to save space.

	Resistance = pad*((1024.0 / RawADC) - 1);
	Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
	Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
	Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      

	return Temp;                                      // Return the Temperature
}




