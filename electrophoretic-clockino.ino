// written 2021-08-05 by mza
// last updated 2023-06-16 by mza

// cobbled together from:
// inkplate10 example code/docs https://inkplate.readthedocs.io/en/latest/arduino.html#inkplate-drawthickline
// rtc example code https://e-radionica.com/hr/blog/2020/05/21/kkm-pcf85063a-rtc/
// arduino ntp example code https://www.arduino.cc/en/Tutorial/LibraryExamples/Wifi101UdpNTPClient
// adafruit example code
// Watchy code https://github.com/mzandrew/Watchy
// custom stuff

// installation:
// sudo apt remove -y brltty
// sudo apt install -y arduino python3-pip
// pip3 install pyserial
// preferences / board definitions: add // https://github.com/SolderedElectronics/Dasduino-Board-Definitions-for-Arduino-IDE/raw/master/package_Dasduino_Boards_index.json
// boards manager: install inkplate
// select board: "e-radionica.com inkplate10"
// library manager: install adafruit-gfx, inkplate, timelib
// "soldered pcf85063a" library doesn't work, so see git clone note below

#ifndef ARDUINO_INKPLATE10
	#error "Wrong board selection for this example, please select Inkplate 10 in the boards menu."
#endif

#include <WiFi.h> // for ntp
#include <WiFiUdp.h> // for ntp
#include <Inkplate.h> // install "inkplatelibrary" for inkplate electrophoretic displays
#include <TimeLib.h>
#include <driver/rtc_io.h> // Include ESP32 library for RTC pin I/O (needed for rtc_gpio_isolate() function)
#include <rom/rtc.h>       // Include ESP32 library for RTC (needed for rtc_get_reset_reason() function)
#include <PCF85063A.h> // ~/build/Arduino/libraries$ git clone https://github.com/e-radionicacom/PCF85063A-Arduino-Library
#include "secrets.h" // for wifi name/password
#include <time.h>

#define SERIAL_NUMBER 5

bool fake_ntp_server_connection = false;

#if SERIAL_NUMBER==1
	char hostname[] = "electrophoretic-clockino-1";
	bool rtc_is_broken = true; // I guess this happens sometimes...
	//bool rtc_is_broken = false; // I guess this happens sometimes...
#elif SERIAL_NUMBER==2
	char hostname[] = "electrophoretic-clockino-2";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#elif SERIAL_NUMBER==3
	char hostname[] = "electrophoretic-clockino-3";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#elif SERIAL_NUMBER==4
	char hostname[] = "electrophoretic-clockino-4";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#elif SERIAL_NUMBER==5
	char hostname[] = "electrophoretic-clockino-5";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#elif SERIAL_NUMBER==6
	char hostname[] = "electrophoretic-clockino-6";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#else
	char hostname[] = "electrophoretic-clockino-unknown";
	bool rtc_is_broken = false; // I guess this happens sometimes...
#endif

#define DEBUG
//#define USE_DEEP_SLEEP // it does a full-refresh after coming out of deep sleep, so we don't use it
#define MAX_WIFI_RETRIES (60)
#define TIME_SET_DELAY_S (2) // positive fudge factor to allow for upload time, etc. (seconds, YMMV)
#define TIME_SET_DELAY_MS (1500) // extra negative fudge factor to tweak time (milliseconds, should be at least 1000 to wait for the ntp packet response)
#define SECONDS_TO_SLEEP (60)
#define RTC_PIN GPIO_NUM_39
#define NUMBER_OF_SLICES (20)
#define TIME_SLICE (1000/NUMBER_OF_SLICES)

#define LIGHT_COLOR (0)
#define DARK_COLOR (1)
Inkplate display(INKPLATE_1BIT); // Create an object on Inkplate library and also set library into 1 Bit mode (BW)
//#define LIGHT_COLOR (7)
//#define DARK_COLOR (0)
//Inkplate display(INKPLATE_3BIT); // Create an object on Inkplate library and also set library into 3 Bit mode
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to hold incoming and outgoing packets
WiFiUDP UDP;
PCF85063A rtc;
bool should_draw_clock_face;
unsigned int duration;

// Inkplate10 is 1200x825
//#define PORTRAIT_MODE
#ifdef PORTRAIT_MODE
	#define CLOCK_CENTER_X (E_INK_HEIGHT/2)
	//#define CLOCK_CENTER_Y (E_INK_WIDTH/2)
	#define CLOCK_CENTER_Y (E_INK_HEIGHT/2) // to top-justify it
#else
	#define CLOCK_CENTER_X (E_INK_WIDTH/2)
	//#define CLOCK_CENTER_X (E_INK_HEIGHT/2) // to left-justify it
	#define CLOCK_CENTER_Y (E_INK_HEIGHT/2)
#endif
#define CIRCLE_RADIUS (410)
#define ANNULAR_RING_SIZE (8)
#define HOUR_TICK_SIZE (8)
#define MINUTE_TICK_SIZE (6)
#define HOUR_TICK_THICKNESS (8)
#define MINUTE_TICK_THICKNESS (4)
#define HOUR_HAND_RADIUS (220)
#define MINUTE_HAND_RADIUS (295)
#define HOUR_HAND_THICKNESS (26.)
#define MINUTE_HAND_THICKNESS (16.)
#define LITTLE_CIRCLE_RADIUS (HOUR_HAND_THICKNESS/2.)
#define DIGIT_POSITION_RADIUS (350)
#define DIGIT_POSITION_X_OFFSET (3)
#define DIGIT_POSITION_Y_OFFSET (2)
#define DIGIT_FONT_SIZE (8)
#define ANGLE_IN_DEGREES_FOR_ONE_HOUR (360/12)
#define ANGLE_IN_DEGREES_FOR_ONE_MINUTE (360/60)
#define RADIANS_PER_DEGREE (PI/180.)
//#define TIME_IN_MILLISECONDS_TO_DRAW_CLOCKFACE (4000)
//#define TIME_IN_MILLISECONDS_TO_DRAW_CLOCKHANDS (3000)
#define TIME_IN_MILLISECONDS_FOR_PARTIAL_REFRESH_OF_THE_DISPLAY (1100)
#define TIME_IN_MILLISECONDS_FOR_FULL_REFRESH_OF_THE_DISPLAY (2100)

#ifdef USE_DEEP_SLEEP
	RTC_DATA_ATTR tmElements_t currentTime;
	RTC_DATA_ATTR tmElements_t lastTime;
	RTC_DATA_ATTR bool wifi_active = false;
#else
	tmElements_t currentTime;
	tmElements_t lastTime;
	bool wifi_active = false;
#endif

int connectWiFi() {
	if (wifi_active) {
		Serial.print("IP address: "); Serial.println(WiFi.localIP());
		return 1;
	}
	Serial.print("Connecting to " WLAN_SSID "... ");
	WiFi.setHostname(hostname);
	WiFi.begin(WLAN_SSID, WLAN_PASS);
	uint8_t retries = MAX_WIFI_RETRIES;
	while (WiFi.status() != WL_CONNECTED) {
		delay(100);
		Serial.print(".");
		retries--;
		if (retries == 0) { Serial.println("  failed"); return 0; }
	}
	if (WiFi.status() == WL_CONNECTED) {
		Serial.print("  IP address: "); Serial.println(WiFi.localIP());
		int tenths = MAX_WIFI_RETRIES - retries;
		Serial.print("wifi connection took "); Serial.print(tenths/10.); Serial.println(" seconds");
		wifi_active = true;
		return tenths ? tenths : 0;
	} else {
		return 0;
	}
}

void disconnectWiFi() {
	if (wifi_active) {
		WiFi.disconnect();
		WiFi.mode(WIFI_OFF);
		Serial.println("wifi disconnected");
		wifi_active = false;
	}
}

void sendNTPpacket(IPAddress &address) {
	memset(packetBuffer, 0, NTP_PACKET_SIZE);
	packetBuffer[0] = 0b11100011; // LI, Version, Mode
	packetBuffer[1] = 0;          // Stratum, or type of clock
	packetBuffer[2] = 6;          // Polling Interval
	packetBuffer[3] = 0xEC;       // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12]  = 49;
	packetBuffer[13]  = 0x4E;
	packetBuffer[14]  = 49;
	packetBuffer[15]  = 52;
	if (!fake_ntp_server_connection) {
		UDP.beginPacket(address, 123); // NTP requests are to port 123
		UDP.write(packetBuffer, NTP_PACKET_SIZE);
		UDP.endPacket();
	}
}

void setTimeViaNTP() {
	// from https://randomnerdtutorials.com/esp32-ntp-timezones-daylight-saving/
	struct tm timeinfo;
	if (connectWiFi()) {
		if (!fake_ntp_server_connection) {
			configTime(0, 0, "pool.ntp.org");
			if(!getLocalTime(&timeinfo)){
				Serial.println("Failed to obtain time from ntp server");
				return;
			}
			Serial.println("Got the time from ntp server");
		} else {
			Serial.println("faked the ntp server transaction");
		}
	}
	setenv("TZ", TZ_STRING, 1); tzset();
	if(!getLocalTime(&timeinfo)){
		Serial.println("Failed to obtain time");
		return;
	}
	Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
	rtc.setTime(timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
	rtc.setDate(timeinfo.tm_wday, timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year+1900);
//	Serial.println(timeinfo.tm_year+1900);
}

// sendNTPpacket and NTP function code are from Arduino/libraries/WiFi101/examples/WiFiUdpNtpClient/WiFiUdpNtpClient.ino
void old_setTimeViaNTP() {
	if (connectWiFi()) {
		unsigned int localPort = 2390; // local port to listen for UDP packets
		IPAddress timeServer(132, 163, 97, 4); // ntp1.glb.nist.gov NTP server
		UDP.begin(localPort);
		sendNTPpacket(timeServer); // send an NTP packet to a time server
		delay(TIME_SET_DELAY_MS);
		if ( UDP.parsePacket() ) {
			UDP.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
			unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
			unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
			unsigned long secsSince1900 = highWord << 16 | lowWord;
			const unsigned long seventyYears = 2208988800UL;
			time_t epoch = secsSince1900 - seventyYears;
			char timestring[20];
	//		Serial.print("Seconds since Jan 1 1900 = " ); Serial.println(secsSince1900);
	//		Serial.print("epoch time = "); Serial.println(epoch);
	//		sprintf(timestring, "%02d:%02d:%02d", (epoch%86400)/3600, (epoch%3600)/60, epoch%60);
	//		Serial.print("UTC time is "); Serial.println(timestring);
			epoch += (UTC_OFFSET_HOURS) * 3600; // UTC_OFFSET_HOURS is a signed quantity
	//		Serial.print("epoch time (local) = "); Serial.println(epoch);
			sprintf(timestring, "%02ld:%02ld:%02ld", (epoch%86400)/3600, (epoch%3600)/60, epoch%60);
			Serial.print("ntp server responded with "); Serial.println(timestring); Serial.flush();
			const time_t fudge(TIME_SET_DELAY_S);
			epoch += fudge;
			currentTime.Hour   = (epoch%86400)/3600;
			currentTime.Minute = (epoch%3600)/60;
			currentTime.Second = epoch%60;
			sprintf(timestring, "%02ld:%02ld:%02ld", currentTime.Hour, currentTime.Minute, currentTime.Second);
			Serial.print("setting time to "); Serial.println(timestring); Serial.flush();
			rtc.setTime(currentTime.Hour, currentTime.Minute, currentTime.Second);
		} else {
			Serial.println("didn't get a response");
		}
//		if (!rtc_is_broken) {
			disconnectWiFi();
//		}
	} else {
		Serial.println("Couldn't connect to wifi");
	}
//	return currentTime.Second - TIME_SET_DELAY_MS/1000;
}

void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst){
	struct tm timeinfo;
	timeinfo.tm_year = yr - 1900;
	timeinfo.tm_mon = month-1;
	timeinfo.tm_mday = mday;
	timeinfo.tm_hour = hr;
	timeinfo.tm_min = minute;
	timeinfo.tm_sec = sec;
	timeinfo.tm_isdst = isDst; // 1 or 0
	time_t t = mktime(&timeinfo);
	Serial.printf("Setting time: %s", asctime(&timeinfo));
	struct timeval now = { .tv_sec = t };
	settimeofday(&now, NULL);
}

void showtime() {
	struct tm timeinfo;
	if(!getLocalTime(&timeinfo)){
		Serial.println("Failed to obtain time");
		return;
	}
	Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S zone %Z %z ");
}

void old_showtime() {
	int hour = currentTime.Hour;
	String ampm = int(hour/12) ? "pm" : "am";
	hour %= 12; if (hour==0) { hour = 12; }
	int minute = currentTime.Minute;
	int second = currentTime.Second;
	int year = currentTime.Year;
	int month = currentTime.Month;
	int day = currentTime.Day;
//	String dst = tm.tm_isdst
	char timestring[23];
	sprintf(timestring, "%2d:%02d:%02d %s %04d-%02d-%02d", hour, minute, second, ampm.c_str(), year, month, day);
	Serial.println(timestring); Serial.flush();
}

void new_rtc_fetch() {
	struct tm timeinfo;
	if(!getLocalTime(&timeinfo)){
		Serial.println("Failed to obtain time");
		return;
	}
	currentTime.Hour = timeinfo.tm_hour;
	currentTime.Minute = timeinfo.tm_min;
	currentTime.Second = timeinfo.tm_sec;
	currentTime.Year = timeinfo.tm_year;
	currentTime.Month = timeinfo.tm_mon;
	currentTime.Day = timeinfo.tm_mday;
}

void rtc_fetch() {
	currentTime.Hour = rtc.getHour();
	currentTime.Minute = rtc.getMinute();
	currentTime.Second = rtc.getSecond();
	currentTime.Year = rtc.getYear();
	currentTime.Month = rtc.getMonth() + 1;
	currentTime.Day = rtc.getDay();
	setTime(currentTime.Year, currentTime.Month, currentTime.Day, currentTime.Hour, currentTime.Minute, currentTime.Second, 0);
}

void time_advance_old(uint8_t seconds) {
	currentTime.Second += seconds;
	if (59<currentTime.Second) {
		currentTime.Second -= 60;
		if (59<++currentTime.Minute) {
			currentTime.Minute = 0;
			if (23<++currentTime.Hour) { currentTime.Hour = 0; }
		}
	}
}

void time_advance() {
	currentTime.Second = 0;
	if (59<++currentTime.Minute) {
		currentTime.Minute = 0;
		if (23<++currentTime.Hour) { currentTime.Hour = 0; }
	}
}

// https://inkplate.readthedocs.io/en/latest/arduino.html
void drawClockFace(int color=DARK_COLOR) {
//	Serial.println("drawing clock face...");
	Serial.println("starting annular ring");
	display.fillCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CIRCLE_RADIUS, color);
	Serial.println("done with big circle");
	display.fillCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, CIRCLE_RADIUS-ANNULAR_RING_SIZE, DARK_COLOR - color);
	Serial.println("done with annular ring");
	int16_t x, y, x1, y1;
	uint16_t w, h;
	int i, j, k;
	display.setTextSize(DIGIT_FONT_SIZE);
	char hour_label[3];
	for (i=0; i<12; i++) {
		sprintf(hour_label, "%d", i==0 ? 12 : i);
//		Serial.print(hour_label);
		display.getTextBounds(hour_label, 0, 0, &x, &y, &w, &h);
		x = CLOCK_CENTER_X + DIGIT_POSITION_RADIUS * sin(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR) + DIGIT_POSITION_X_OFFSET;
		y = CLOCK_CENTER_Y - DIGIT_POSITION_RADIUS * cos(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR) + DIGIT_POSITION_Y_OFFSET;
//		Serial.print("         i, x, y = "); Serial.print(i); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y);
		x -= w/2; y -= h/2;
		if (i==10 || i==11) { x += 10; y += 10; }
//		Serial.print("         i, x, y = "); Serial.print(i); Serial.print(" "); Serial.print(x); Serial.print(" "); Serial.print(y);
		display.setCursor(x, y);
		display.print(hour_label);
//		Serial.println("");
		x  = CLOCK_CENTER_X + (CIRCLE_RADIUS-ANNULAR_RING_SIZE-HOUR_TICK_SIZE) * sin(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR);
		x1 = CLOCK_CENTER_X + (CIRCLE_RADIUS-ANNULAR_RING_SIZE               ) * sin(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR);
		y  = CLOCK_CENTER_Y - (CIRCLE_RADIUS-ANNULAR_RING_SIZE-HOUR_TICK_SIZE) * cos(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR);
		y1 = CLOCK_CENTER_Y - (CIRCLE_RADIUS-ANNULAR_RING_SIZE               ) * cos(RADIANS_PER_DEGREE*i*ANGLE_IN_DEGREES_FOR_ONE_HOUR);
		display.drawThickLine(x, y, x1, y1, color, HOUR_TICK_THICKNESS);
		for (j=1; j<5; j++) { // five lengths of fence between six posts, but the two endposts are already drawn, hence a difference of 4 here
			k = 5 * i + j; // 5=60/12
			x  = CLOCK_CENTER_X + (CIRCLE_RADIUS-ANNULAR_RING_SIZE-MINUTE_TICK_SIZE) * sin(RADIANS_PER_DEGREE*k*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
			x1 = CLOCK_CENTER_X + (CIRCLE_RADIUS-ANNULAR_RING_SIZE                 ) * sin(RADIANS_PER_DEGREE*k*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
			y  = CLOCK_CENTER_Y - (CIRCLE_RADIUS-ANNULAR_RING_SIZE-MINUTE_TICK_SIZE) * cos(RADIANS_PER_DEGREE*k*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
			y1 = CLOCK_CENTER_Y - (CIRCLE_RADIUS-ANNULAR_RING_SIZE                 ) * cos(RADIANS_PER_DEGREE*k*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
			display.drawThickLine(x, y, x1, y1, color, MINUTE_TICK_THICKNESS);
		}
//		Serial.print("done with hour #"); Serial.println(i);
	}
}

void drawClock(tmElements_t time, int color=DARK_COLOR) {
	int16_t x, y;
	int hour = time.Hour;
	int minute = time.Minute;
	int hour_hand_degrees = (60*hour+minute)*ANGLE_IN_DEGREES_FOR_ONE_HOUR/60;
//	Serial.print("hour hand angle (degrees) = "); Serial.println(hour_hand_degrees);
	x = CLOCK_CENTER_X + HOUR_HAND_RADIUS * sin(RADIANS_PER_DEGREE*hour_hand_degrees);
	y = CLOCK_CENTER_Y - HOUR_HAND_RADIUS * cos(RADIANS_PER_DEGREE*hour_hand_degrees);
	display.drawThickLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, x, y, color, HOUR_HAND_THICKNESS);
	x = CLOCK_CENTER_X + MINUTE_HAND_RADIUS * sin(RADIANS_PER_DEGREE*minute*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
	y = CLOCK_CENTER_Y - MINUTE_HAND_RADIUS * cos(RADIANS_PER_DEGREE*minute*ANGLE_IN_DEGREES_FOR_ONE_MINUTE);
	display.drawThickLine(CLOCK_CENTER_X, CLOCK_CENTER_Y, x, y, color, MINUTE_HAND_THICKNESS);
	display.fillCircle(CLOCK_CENTER_X, CLOCK_CENTER_Y, int(LITTLE_CIRCLE_RADIUS+.5), color);
}

void drawColors() {
	int x=10, y=10, w=50, h=50;
	y=10; y= 10; display.fillRect(x, y, w, h, 0);
	x=10; y=110; display.fillRect(x, y, w, h, 1);
	x=10; y=210; display.fillRect(x, y, w, h, 2);
	x=10; y=310; display.fillRect(x, y, w, h, 3);
	x=10; y=410; display.fillRect(x, y, w, h, 4);
	x=10; y=510; display.fillRect(x, y, w, h, 5);
	x=10; y=610; display.fillRect(x, y, w, h, 6);
	x=10; y=710; display.fillRect(x, y, w, h, 7);
}

void draw_fresh_clock_face_if_necessary_or_just_clear_previous_clock_hands() {
	if (should_draw_clock_face) {
		display.clearDisplay();
		Serial.println("done with cleardisplay");
		drawClockFace(DARK_COLOR);
		Serial.println("done with drawclockface(DARK_COLOR)");
	} else {
		drawClock(lastTime, LIGHT_COLOR);
		Serial.println("done with drawclock(LIGHT_COLOR)");
	}
	Serial.flush();
}

void get_time_from_ntp_or_rtc() {
	bool should_get_time_from_ntp = false;
//	if (rtc_is_broken) {
//		should_get_time_from_ntp = true;
//	}
	if (0==currentTime.Hour && 0==currentTime.Minute) {
		should_get_time_from_ntp = true;
	}
#ifdef DEBUG
	if (57==currentTime.Minute) {
		should_get_time_from_ntp = true;
	}
#else
	if (23==currentTime.Hour && 57==currentTime.Minute) {
		should_get_time_from_ntp = true;
	}
#endif
	if (should_get_time_from_ntp) {
		setTimeViaNTP();
	}
	rtc_fetch();
//	showtime();
}

void draw_new_clock_hands() {
	drawClock(currentTime, DARK_COLOR);
	lastTime = currentTime;
	Serial.println("done with drawclock(DARK_COLOR)"); Serial.flush();
}

void update_the_display() {
	if (should_draw_clock_face) {
		display.display();
		Serial.println("done with full refresh");
		should_draw_clock_face = false;
	} else {
		display.partialUpdate(); // Updates only the changed parts of the screen. (monochrome/INKPLATE_1BIT mode only!)
		Serial.println("done with partial refresh");
	}
	Serial.flush();
}

void wait_for_next_second() {
	Serial.flush();
	rtc_fetch(); showtime();
	uint8_t second = currentTime.Second;
	for (int i=0; i<NUMBER_OF_SLICES; i++) {
		rtc_fetch();
		if (second != currentTime.Second) {
			Serial.print("start of next second is now: ");
			showtime();
			return;
		}
		second = currentTime.Second;
		delay(TIME_SLICE);
	}
}

unsigned int how_long_to_wait_until_just_before_the_end_of_the_minute(unsigned int extra=0) {
	signed int duration_ms = 60000 - 1000*currentTime.Second - 1000*extra;
	if (should_draw_clock_face) {
		duration_ms -= TIME_IN_MILLISECONDS_FOR_FULL_REFRESH_OF_THE_DISPLAY;
	} else {
		duration_ms -= TIME_IN_MILLISECONDS_FOR_PARTIAL_REFRESH_OF_THE_DISPLAY;
	}
	if (duration_ms<0) {
		duration_ms = 0;
	}
	if (55000<duration_ms) {
		duration_ms = 30000;
	}
	return (unsigned int) duration_ms;
}

void setup() {
	Serial.begin(115200);
	display.begin(); // Init Inkplate library (you should call this function ONLY ONCE)
	#ifdef PORTRAIT_MODE
		display.setRotation(3);
	#endif
	Serial.println("\n\n");
	Serial.println(hostname);
	#ifndef USE_DEEP_SLEEP
//		display.clearDisplay();
//		display.display();
	#else
		if (rtc_get_reset_reason(0) == DEEPSLEEP_RESET) {
			Serial.println("\n\nwoke up from deep sleep");
			//Serial.println("partial refresh of display");
//			drawClock(); // with the old data
//			display.preloadScreen();
//			display.clearDisplay();
//			RTC.read(currentTime);
//			drawClock(); // with the new data
//			display.partialUpdate();
		} else {
			Serial.println("\n\nwoke up from reset/reprogram/boot");
//			Serial.println("full refresh of display");
//			display.clearDisplay();
//			RTC.read(currentTime);
//			drawClock(); // with the new data
//			display.display();
		}
		esp_sleep_wakeup_cause_t wakeup_reason;
		wakeup_reason = esp_sleep_get_wakeup_cause(); // get wake up reason
		switch (wakeup_reason) {
			case ESP_SLEEP_WAKEUP_TIMER: // ESP Internal RTC
				Serial.println("\n\nwoke up from internal rtc deep sleep");
				break;
			case ESP_SLEEP_WAKEUP_EXT0: // RTC Alarm
				Serial.println("\n\nwoke up from external rtc deep sleep");
//				RTC.alarm(ALARM_2); // resets the alarm flag in the RTC
				break;
			default: //reset
				Serial.println("\n\nwoke up from reset/reprogram/boot");
				break;
		}
		if (0) {
			//display.print("\nConnecting to WiFi...");
			//Serial.println("\nConnecting to WiFi...");
			//display.partialUpdate();
		}
		//disconnectWiFi();
		//https://github.com/JChristensen/DS3232RTC
//		RTC.squareWave(SQWAVE_NONE); //disable square wave output
//		RTC.setAlarm(ALM2_EVERY_MINUTE, 0, 0, 0, 0); //alarm wakes up every minute
//		RTC.alarmInterrupt(ALARM_2, true); //enable alarm interrupt
		Serial.println("going to deep sleep mode...");
		esp_sleep_enable_timer_wakeup(SECONDS_TO_SLEEP * 1000000);
		rtc_gpio_isolate(GPIO_NUM_12); // Isolate/disable GPIO12 on ESP32 (only to reduce power consumption in sleep)
		esp_sleep_enable_ext0_wakeup(RTC_PIN, 0); //enable deep sleep wake on RTC interrupt
		esp_deep_sleep_start();
	#endif
//	if (rtc_is_broken) {
//		rtc.reset();
//	}
	get_time_from_ntp_or_rtc();
	should_draw_clock_face = true;
	duration = how_long_to_wait_until_just_before_the_end_of_the_minute(10);
	if (duration<10000) {
		time_advance();
	}
	lastTime = currentTime;
	draw_fresh_clock_face_if_necessary_or_just_clear_previous_clock_hands();
	draw_new_clock_hands();
	duration = how_long_to_wait_until_just_before_the_end_of_the_minute(30);
	//drawColors();
	//Serial.println("done with drawColors()");
}

void loop() {
	#ifndef USE_DEEP_SLEEP
		Serial.println("");
		update_the_display(); // clears should_draw_clock_face
		Serial.print("delaying for "); Serial.println(duration);
		delay(duration);
		if (currentTime.Minute==59) {
			should_draw_clock_face = true;
		}
		if (INKPLATE_3BIT==display.getDisplayMode()) {
			should_draw_clock_face = true;
		}
		draw_fresh_clock_face_if_necessary_or_just_clear_previous_clock_hands();
		time_advance();
		draw_new_clock_hands();
		//wait_for_next_second();
		get_time_from_ntp_or_rtc();
		showtime();
		duration = how_long_to_wait_until_just_before_the_end_of_the_minute();
		Serial.print("delaying for "); Serial.println(duration);
		delay(duration);
//		Serial.flush();
		duration = 30000; // aim for the center of the eye
	#endif
}
