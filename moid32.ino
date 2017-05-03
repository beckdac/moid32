/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2017 David A. C. Beck
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/******************************************************************************
 * i2c
 *****************************************************************************/
#include <HardWire.h>
HardWire HWire(1, I2C_FAST_MODE | I2C_REMAP);


/******************************************************************************
 * SSD1306 OLED
 *****************************************************************************/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
// this pin is not used but needs to be provided to the driver, use the LED pin
#define OLED_RESET	PC13
Adafruit_SSD1306 oled(OLED_RESET);


/******************************************************************************
 * Rotary encoders
 *****************************************************************************/
#define ENCODERS 2
#define ENCODER_PERIOD_uS	1000
#define ENCODER_SLOW_MS	3
#define ENCODER_FAST_STEPS	5
boolean encoder_clk[ENCODERS];
boolean encoder_dt[ENCODERS];
volatile int16_t encoder_value[ENCODERS];
volatile int  encoder_millis[ENCODERS]; 
int encoder_clk_pin[ENCODERS] = { PB4, PB6 };
int encoder_dt_pin[ENCODERS] = { PB5, PB7 };
int encoder_min[ENCODERS] = { 0 };
int encoder_max[ENCODERS] = { 20 };
unsigned int encoder_last_value[ENCODERS];
HardwareTimer encoder_timer(1);


/******************************************************************************
 * LED pins
 *****************************************************************************/
#define LED_PIN PC13
// should the LED pin be pulsed with PWM (won't work on PC13)
#define PWM_OUT 1
#undef PWM_OUT


/******************************************************************************
 * Serial
 *****************************************************************************/
#define SERIAL_BAUD	9600
#undef SERIAL
#define SERIAL


/******************************************************************************
 * Misc 
 *****************************************************************************/
#undef DEBUG
#define DEBUG


/******************************************************************************
 * Setup
 *****************************************************************************/
void setup() {
	// start i2c bus
	HWire.begin();

#ifdef SERIAL
	// setup serial and send welcome message
	Serial1.begin(SERIAL_BAUD);
	Serial1.println("Welcome, give us a spin...");
#endif

	// scan the i2c bus for devices
	if (i2c_scan() == 0) {
#ifdef SERIAL
		Serial1.println("WARNING: no OLED found!");
#endif
	}
	oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	oled.display();
 
#ifdef PMW_OUT
	pinMode(PWM_PIN, PWM);
	pwmWrite(PWM_PIN, 0);
#else
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
#endif

	encoders_init();

	oled.clearDisplay();
	oled.setTextSize(1);
	oled.setTextColor(WHITE);
	oled.setCursor(0, 0);
	oled.println("Welcome!");
	oled.display();

#ifdef DEBUG
	pinMode(PB6, OUTPUT);
	digitalWrite(PB6, LOW);
	pinMode(PB7, OUTPUT);
	digitalWrite(PB7, HIGH);
#endif
}

/******************************************************************************
 * Main loop
 *****************************************************************************/
#ifndef PWM_OUT
uint16_t loopc = 0;
#endif

void loop() {
	boolean oled_update = false;

	for (uint8_t i = 0; i < ENCODERS; ++i) {
#ifndef PWM_OUT
		if (loopc >= (65535 / encoder_max[i]) * encoder_value[i]) {
			digitalWrite(LED_PIN, !digitalRead(LED_PIN));
			loopc = 0;
#ifdef DEBUG
			digitalWrite(PB6, !digitalRead(PB6));
			digitalWrite(PB7, !digitalRead(PB7));
#endif
		} else
			++loopc;
#endif
		
		if ((encoder_last_value[i] != encoder_value[i])) {
#ifdef SERIAL
			Serial1.print("Encoder ");
			Serial1.print(i, DEC);
			Serial1.print(" = ");
			Serial1.print(encoder_value[i]);
			Serial1.print(", loopc = ");
			Serial1.println(loopc);
#endif
			encoder_last_value[i] = encoder_value[i];
#ifdef PWM_OUT
			pwmWrite(PWM_PIN, map(encoder_value[i], encoder_min[i], encoder_max[i], 0, 65535));
#endif
			oled_update = true;
    	}
  	}
	if (oled_update) {
		oled.clearDisplay();
		for (uint8_t i = 0; i < ENCODERS; ++i) {
			#define ENCODER_GRAPH_SEP 16
			oled.setCursor(i * ENCODER_GRAPH_SEP, 0);
			oled.print(i + 1, DEC);
			oled.setCursor(i * ENCODER_GRAPH_SEP, 9);
			oled.println(encoder_value[i], DEC);
			#define ENCODER_GRAPH_MAX 48
			gfx_bar_graph(i * ENCODER_GRAPH_SEP, 64, 10, ENCODER_GRAPH_MAX, map(encoder_value[i], encoder_min[i], encoder_max[i], 0, ENCODER_GRAPH_MAX));
		}
		oled.display();
	}
}


/******************************************************************************
 * Encoder interupt and data structure initialization
 *****************************************************************************/
int i2c_scan(void) {
	uint8_t error, addr;
	int n_devices = 0;

#ifdef SERIAL
	Serial1.println("starting i2c bus scan...");
#endif

	for (addr = 1; addr < 127; ++addr) {
		HWire.beginTransmission(addr);
		error = HWire.endTransmission();

		if (error == 0) {
#ifdef SERIAL
			Serial1.println("hai!");
			Serial1.print("i2c device found @ 0x");
			if (addr < 16)
				Serial1.print("0");
			Serial1.println(addr, HEX);
#endif
			++n_devices;
		} else if (error == 4) {
#ifdef SERIAL
			Serial1.print("unknown error @ 0x");
			if (addr < 16)
				Serial1.print("0");
			Serial1.println(addr, HEX);
#endif
		}
	}
#ifdef SERIAL
	if (n_devices == 0)
		Serial1.println("no i2c devices found");
	else {
		Serial1.print("found ");
		Serial1.print(n_devices, DEC);
		Serial1.println(" i2c devices");
	}
#endif
	return n_devices;
}


/******************************************************************************
 * Encoder interupt and data structure initialization
 *****************************************************************************/
void encoders_read() {
	for (uint8_t i = 0; i < ENCODERS; ++i) {
		// gpio_read_bit is faster than digitalRead http://forums.leaflabs.com/topic.php?id=1107 
		if ((gpio_read_bit(PIN_MAP[encoder_clk_pin[i]].gpio_device, PIN_MAP[encoder_clk_pin[i]].gpio_bit) ? HIGH : LOW) != encoder_clk[i]) {
			encoder_clk[i] = !encoder_clk[i];
			if (encoder_clk[i] && !encoder_dt[i]) {
				if (millis() - encoder_millis[i] > ENCODER_SLOW_MS)
					encoder_value[i] += 1;
				else
					encoder_value[i] += ENCODER_FAST_STEPS;
				if (encoder_value[i] < encoder_min[i])
					encoder_value[i] = encoder_min[i];
				else if (encoder_value[i] > encoder_max[i])
					encoder_value[i] = encoder_max[i];
			}
			encoder_millis[i] = millis();
		}
		if ((gpio_read_bit(PIN_MAP[encoder_dt_pin[i]].gpio_device, PIN_MAP[encoder_dt_pin[i]].gpio_bit) ? HIGH : LOW) != encoder_dt[i]) {
			encoder_dt[i] = !encoder_dt[i];
			if (encoder_dt[i] && !encoder_clk[i]) {
				if (millis() - encoder_millis[i] > ENCODER_SLOW_MS)
					encoder_value[i] -= 1;
        		else
          			encoder_value[i] -= ENCODER_FAST_STEPS;
				if (encoder_value[i] < encoder_min[i])
					encoder_value[i] = encoder_min[i];
				else if (encoder_value[i] > encoder_max[i])
					encoder_value[i] = encoder_max[i];
			}
      		encoder_millis[i] = millis();
    	}
	}
}

void encoders_init() {
	for (uint8_t i = 0; i < ENCODERS; ++i) {
		encoder_clk[i] = true;
		encoder_dt[i] = true;
		encoder_value[i] = 0;
		encoder_last_value[i] = 1;
		pinMode(encoder_clk_pin[i], INPUT);
		pinMode(encoder_dt_pin[i], INPUT);
		encoder_millis[i] = millis();
	}

	encoder_timer.pause();
	encoder_timer.setPeriod(ENCODER_PERIOD_uS);
	encoder_timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
	encoder_timer.setCompare(TIMER_CH1, 1);
	encoder_timer.attachCompare1Interrupt(encoders_read);
	encoder_timer.refresh();
	encoder_timer.resume();
}


/******************************************************************************
 * Graphics visuals
 *****************************************************************************/
void gfx_bar_graph(uint8_t base_x, uint8_t base_y, uint8_t width, 
					uint8_t height, uint8_t value) {
	uint8_t x0, y0, fill_height;
	if (value > height)
		value = height;
	x0 = base_x;
	y0 = base_y - height;
	fill_height = value;
	oled.fillRect(x0, y0, width, fill_height, 1);
}
