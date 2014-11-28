//Code for Menu Based water cooling control system. Provides for 2 channels of High speed (25khz) PWM output & 2 Channels low speed (490 hz) PWM output.
//either should funtion for fan PWM control. Only channels 1 & 2 are suitable for control of MOSFETs (IRL510N tested) for voltage control (lower voltage outputs will work but sing).
//
//depends on EEPROMex New Liquid Crystal & M2TKLIB
//Includes
#include <EEPROMVar.h>
#include <EEPROMex.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "M2tk.h"
#include <math.h>
#include "utility/m2ghnlc.h"

#define DEBUG 1

//Analog Pins for Thermistor Connections
#define AMBIENTPIN 0
#define WATER1PIN 1
#define WATER2PIN 2
#define CASEPIN 3

//PWM Pins for Channel 3 & 4 (1 and 2 will always be pin 3 (channel 1) and pin 11 (channel 2) as they're tied to timer 2
#define CHANNEL1PIN 3
#define CHANNEL2PIN 11
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
#define output_channels 4

#define BACKLIGHTPOLLSPEED 500

//=================================================
// Forward declaration of the menu elements
extern M2tk m2;
//M2_EXTERN_ALIGN(el_fan_diag);
//M2_EXTERN_ALIGN(el_cont_diag);
M2_EXTERN_HLIST(el_wc_menu);

//Some waiting variables for the various loops
struct controller_settings {
	uint8_t temp_delay;
	uint8_t screen_delay;
	//some boolean state variables
	boolean backlight_on;
};
controller_settings settings;
int settings_address;

uint16_t temp_delay = 1000;
uint16_t screen_delay = 1000;
uint8_t backlightmenu = 1; //this is needed as uint due to the way m2tk's toggles work. 


unsigned long last_backlight;
unsigned long last_temp;
unsigned long last_screen;


//Array of sensor pins and associated names
const byte sensorpins[4] = { AMBIENTPIN, WATER1PIN, WATER2PIN, CASEPIN };
const char sensornames[4][8] = { "Ambient", "Water 1", "Water 2", "Case  " };
int temperatures[4]; //declare as int because we need to multiply by 100 to maintain accuracy whilst not using floats too much.
//Pad resistor - asjust these values to calibrate.
const int pad[4] = { 10000, 10000, 10000, 10000 };

//struct to describe output channels
struct channel
{
	boolean temp_controlled; //whether we're on temp control
	byte min_duty_cycle; //minimum allowed duty cycle
	byte starting_temp; //starting temperature delta
	byte full_temp; //full speed over and above this
	byte absolute_max; //full speed at this water temp regardless
	byte linked_sensor; //which sensor is linked (index to array)
	byte current_cycle;
};

channel outputs[output_channels];
const byte output_pins[output_channels] = { CHANNEL1PIN, CHANNEL2PIN, CHANNEL3PIN, CHANNEL4PIN };
uint16_t output_addresses[output_channels];
//set these variables to use the high speed output routines on that channel. only works if the channel pin is defined as 3 or 11
const boolean highspeedouts[output_channels] = { true, true, false, false };
//menu variables to adjust the above struct
uint8_t fan_diag_temp_ctrl;
uint8_t	fan_diag_min_duty;
uint8_t fan_diag_start_temp;
uint8_t fan_diag_full_temp;
uint8_t fan_diag_abs_temp;
uint8_t fan_diag_linked_sensor;
byte fan_diag_idx;

//setup dialog in two pages (two dialogs really)
M2_LABEL(el_fan_diag_l1, NULL, "Temp Control");
M2_TOGGLE(el_fan_diag_t1, "", &fan_diag_temp_ctrl);
M2_LABEL(el_fan_diag_l2, NULL, "Start Delta");
M2_U8NUM(el_fan_diag_u1, "c2", 0, 99, &fan_diag_start_temp);
M2_LABEL(el_fan_diag_l3, NULL, "Max Temp");
M2_U8NUM(el_fan_diag_u2, "c2", 0, 99, &fan_diag_full_temp);
M2_BUTTON(el_fan_diag1_cancel, "", "Canc", fn_fan_diag_cancel);
M2_BUTTON(el_fan_diag1_commit, "", "Done", fn_fan_diag_commit);
//arrange to a c2 gridlist
M2_LIST(fandiag1) = { &el_fan_diag_l1, &el_fan_diag_t1, &el_fan_diag_l2, &el_fan_diag_u1, &el_fan_diag_l3, &el_fan_diag_u2, &el_fan_diag1_cancel, &el_fan_diag1_commit };
M2_GRIDLIST(el_fan_diag1, "c2", fandiag1);



//labels for second page
M2_LABEL(el_fan_diag_l4, NULL, "Min Duty");
M2_U8NUM(el_fan_diag_u3, "c3", 0, 100, &fan_diag_min_duty);
M2_LABEL(el_fan_diag_l5, NULL, "Abs Temp");
M2_U8NUM(el_fan_diag_u4, "c3", 0, 100, &fan_diag_abs_temp);
M2_LABEL(el_fan_diag_l6, NULL, "Sensor");
M2_COMBO(el_fan_diag_c1, NULL, &fan_diag_linked_sensor, 4, fn_idx_to_sensor);
M2_BUTTON(el_fan_diag2_cancel, "", "Canc", fn_fan_diag_cancel);
M2_BUTTON(el_fan_diag2_commit, "", "Done", fn_fan_diag_commit);
//arrange to a c2 gridlist
M2_LIST(fandiagoptions2) = { &el_fan_diag_l4, &el_fan_diag_u3, &el_fan_diag_l5, &el_fan_diag_u4, &el_fan_diag_l6, &el_fan_diag_c1, &el_fan_diag2_cancel, &el_fan_diag2_commit };
M2_GRIDLIST(el_fan_diag2, "c2", fandiagoptions2);



const char *fn_idx_to_sensor(uint8_t idx) {
	return sensornames[idx];
}

void fn_fan_diag_cancel(m2_el_fnarg_p fnarg) {
	m2.setRoot(&el_wc_menu);
}
void fn_fan_diag_commit(m2_el_fnarg_p fnarg) {
	parsemenu(fan_diag_idx);
	m2.setRoot(&el_wc_menu);
}
//definition of root menu - will have number of fan channels *3 + controller settings & hide
m2_xmenu_entry xmenu_data[output_channels];
//Initialise the first channel for copying and the last 2 entries


uint8_t el_x2l_first = 0;
uint8_t el_x2l_cnt = 14;

const char *fn_load_temp_diag(uint8_t idx, uint8_t msg) {
	byte channel;
	if (msg == M2_STRLIST_MSG_SELECT) {
		//use idx to determine which fan we're operating on.
		channel = idx / 3;
		//set the working variable
		fan_diag_idx = channel;
		loadtomenu(channel);
		Serial.println(channel);
		m2.setRoot(&el_fan_diag1);
		lcd.clear();
	}

	return "";
}
const char *fn_load_fan_diag(uint8_t idx, uint8_t msg) {
	byte channel;
	if (msg == M2_STRLIST_MSG_SELECT) {
		Serial.println(idx);
		channel = idx / 3;
		//set the working variable
		fan_diag_idx = channel;
		//helper to load variables into input variables.
		loadtomenu(channel);
		Serial.println(channel);
		m2.setRoot(&el_fan_diag2);
		lcd.clear();
	}
	return "";
}
//load the controller settings diag
const char *fn_load_cont_diag(uint8_t idx, uint8_t msg) {
	if (msg == M2_STRLIST_MSG_SELECT) {
		//m2.setRoot(&el_cont_diag);
		lcd.clear();
		Serial.println(idx);
	}

	return "";
}
void loadtomenu(byte chan){
	if (outputs[chan].temp_controlled){
		fan_diag_temp_ctrl = 1;
	}
	else {
		fan_diag_temp_ctrl = 0;
	}
	fan_diag_abs_temp = (uint8_t)outputs[chan].absolute_max;
	fan_diag_full_temp = (uint8_t)outputs[chan].full_temp;
	fan_diag_linked_sensor = (uint8_t)outputs[chan].linked_sensor;
	fan_diag_min_duty = (uint8_t)outputs[chan].min_duty_cycle;
	fan_diag_start_temp = (uint8_t)outputs[chan].starting_temp;
}

void parsemenu(byte chan){
	if (fan_diag_temp_ctrl = 1){
		outputs[chan].temp_controlled = true;
	}
	else {
		outputs[chan].temp_controlled = false;
	}
	outputs[chan].absolute_max = fan_diag_abs_temp;
	outputs[chan].full_temp = fan_diag_full_temp;
	outputs[chan].linked_sensor = fan_diag_linked_sensor;
	outputs[chan].min_duty_cycle = fan_diag_min_duty;
	outputs[chan].starting_temp = fan_diag_start_temp;
	updatechannel(chan);
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

M2tk m2(&m2_null_element, m2_es_arduino_rotary_encoder, m2_eh_4bd, m2_gh_nlc);

//function to set the EEPROM locations
void readaddresses(){

	for (byte i = 0; i < output_channels; i++){
		output_addresses[i] = EEPROM.getAddress(sizeof(outputs[i]));
	}
	settings_address = EEPROM.getAddress(sizeof(settings));
}
void loadchannels(){
	for (byte i = 0; i < output_channels; i++){
		EEPROM.readBlock(output_addresses[i], outputs[i]);
	}
	//EEPROM.readBlock(settings_address, settings);
}
void updatechannel(byte idx){
	EEPROM.updateBlock(output_addresses[idx], outputs[idx]);
}
void updatesettings(byte idx){
	EEPROM.updateBlock(settings_address, settings);
}
//define a degree
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
void setup_menu(){
	//initialise the remainder of the menu appropriate to the number of fan channels defined.
	for (byte i = 0; output_channels - 4; i++) {
		xmenu_data[i].label = "Fan Channel " + i;
		xmenu_data[i].cb = NULL;
		xmenu_data[i].element = NULL;
		xmenu_data[i + 1].label = ". Temp Settings";
		xmenu_data[i + 1].element = NULL;
		xmenu_data[i + 1].cb = fn_load_temp_diag;
		xmenu_data[i + 2].label = ". Fan Settings";
		xmenu_data[i + 2].element = NULL;
		xmenu_data[i + 2].cb = fn_load_temp_diag;
	}
	//initialise the bottom of the menu
	xmenu_data[output_channels - 3].label = "Controller";
	xmenu_data[output_channels - 3].cb = fn_load_cont_diag;
	xmenu_data[output_channels - 3].label = "Hide Menu";
	xmenu_data[output_channels - 3].cb = fn_menu_hide;
}
void setup() {
	//read in the saved config settings
	EEPROM.setMemPool(0, EEPROMSizeUno);
	readaddresses();
	loadchannels();
	//setup menu

	m2_SetNewLiquidCrystal(&lcd, 20, 4);
	// define button for the select message
	m2.setPin(M2_KEY_SELECT, BTNPIN);
	// The incremental rotary encoder is conected to these two pins
	m2.setPin(M2_KEY_ROT_ENC_A, DTPIN); //DT
	m2.setPin(M2_KEY_ROT_ENC_B, CLKPIN); //CLK

	//set the backlight pin, but let the backlight worker update it.
	lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
	//create our degree char
	lcd.createChar(0, degree);
	setup_menu();

	//begin code to change PWM frequency on Timer 2 HT bens @ arduino forums.
	TCCR2A = B10100011;
	TCCR2B = 0x09;  // set prescale to 8
	pinMode(CHANNEL1PIN, OUTPUT);  // enable the PWM output
	pinMode(CHANNEL2PIN, OUTPUT);  // enable the PWM output
	OCR2B = 255;  // set the PWM duty cycle to 100%
	OCR2A = 255;  // on both channels

#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("Setup COmplete");
	Serial.println(settings.screen_delay);
	Serial.println(settings.temp_delay);
#endif // DEBUG

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
	if (millis() - BACKLIGHTPOLLSPEED < last_backlight) { return; }
#ifdef DEBUG
	Serial.println("backlight");
	Serial.println(last_backlight);
	Serial.println(millis() - BACKLIGHTPOLLSPEED);
#endif // DEBUG

	last_backlight = millis();
	if (backlightmenu == 1 && !settings.backlight_on) {
		settings.backlight_on = true;
		lcd.setBacklight(HIGH);
	}
	else if (backlightmenu == 0 && settings.backlight_on){
		settings.backlight_on = false;
		lcd.setBacklight(LOW);
	}
	return;
}

void worker_monitortemps(void) {
	//return if we are too quick through the rest - defaults give us a 0.2s polling rate
	if (millis() - settings.temp_delay < last_temp){
		return;
	}

#ifdef DEBUG
	Serial.println("temp");
	Serial.println(last_temp);
	Serial.println(millis() - settings.temp_delay);
#endif // DEBUG

	last_temp = millis();
	//read in all the temps
	for (int i = 0; i < 4; i++)
	{
		last_temp = millis();
		temperatures[i] = (int)Thermistor(analogRead(sensorpins[i]), pad[i]) * 100;
		Serial.println(analogRead(sensorpins[i]));
	}
	//now compare based on which sensor we're linked to
	for (int i = 0; i < output_channels; i++) {
		int delta;
		//check for temperature control
		if (!outputs[i].temp_controlled){
			//set the duty cycle to min duty cycle then do the next fan
			writeoutput(i, outputs[i].min_duty_cycle);
			continue;
		}
		//check if we're over our absolutes, if so write max and then next fan
		if (temperatures[outputs[i].linked_sensor] > outputs[i].absolute_max * 100) {
			writeoutput(i, 100);
			continue;
		}

		//we're on delta control so calculate it
		delta = temperatures[i] - temperatures[0];
		//only use positive values between 0 and 100C
		constrain(delta, 0, 10000);
		//check if we need to go to 0
		if (delta <= outputs[i].starting_temp * 100){
			writeoutput(i, 0);
			continue;
		}
		//check if we need to go straight to full speed
		else if (delta >= outputs[i].full_temp * 100) {
			writeoutput(i, 100);
			continue;
		}
		else {
			//map the current temp to somewhere between minimum cycle and 100% and set it.
			writeoutput(i, map(delta, outputs[i].starting_temp * 100, outputs[i].full_temp * 100, outputs[i].min_duty_cycle, 100));
			continue;
		}
	}

}

void worker_reporting(void) {
	if (millis() - settings.screen_delay < last_screen) {
		return;
	}
	if (m2.getRoot() != &m2_null_element){ return; }

#ifdef DEBUG
	Serial.println("Screen");
	Serial.println(last_screen);
	Serial.println(millis() - settings.screen_delay);
#endif // DEBUG

	last_screen = millis();
	char buffer[5];
	for (byte i = 0; i < 4; i++) {
		lcd.setCursor(0, i);
		lcd.print(sensornames[i]);
		lcd.setCursor(9, i);
		lcd.print(temperatures[i] / 100);
		lcd.write((byte)0);
		lcd.print("C");
		lcd.setCursor(16, i);
		sprintf(buffer, "%3d%%", outputs[i].current_cycle);
		lcd.print(buffer);
	}
	if (m2.getKey() != M2_KEY_NONE)
		m2.setRoot(&el_wc_menu);

}


void writeoutput(byte outputidx, byte pct) {
	//function takes an output index and a number between 0 and 100 to set the output appropriately.
	outputs[outputidx].current_cycle = pct;
	switch (output_pins[outputidx]){
	case 3:
		if (highspeedouts[outputidx]) {
			//this means we're adjusting OCR2B between 0 and 255
			OCR2B = map(pct, 0, 100, 0, 255);
		}
		else{
			analogWrite(output_pins[outputidx], map(pct, 0, 100, 0, 255));
		}
		break;
	case 11:
		if (highspeedouts[outputidx]) {
			//this means we're adjusting OCR2AB between 0 and 255
			OCR2A = map(pct, 0, 100, 0, 255);
		}
		else{
			analogWrite(output_pins[outputidx], map(pct, 0, 100, 0, 255));
		}
		break;
	default:
		//if we're here we're just doing an analog write because we can't high speed PWM on those pins
		analogWrite(output_pins[outputidx], map(pct, 0, 100, 0, 255));
	}

}


float Thermistor(int RawADC, int pad) {
	long Resistance;
	float Temp;  // Dual-Purpose variable to save space.

	Resistance = pad*((1024.0 / RawADC) - 1);
	Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
	Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
	Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      

	return Temp;                                      // Return the Temperature
}
