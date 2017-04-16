/*
   Sends GPS position + time + configurable message
   Log sent and received messages with timestamp, snr and rssi
   Web GUI
     Set send interval
     Set message text
     Show current position + time
     Show live log (websocket?) - buffer last x messages
     Fetch entire log file
     Clear/rotate log file
     Set spreading factor
     Set power setTxPower(x) where 5 <= x <=20
*/

/* Dependencies:
   TinyGPSPlus: https://github.com/mikalhart/TinyGPSPlus
   Radiohead: www.airspayce.com/mikem/arduino/RadioHead/
   Time: Board Manager, search for "gps" (yes I know the search term is strange)
*/
extern "C" {
#include "user_interface.h"
}
#include <ESP8266WiFi.h>

#include <SPI.h>
#include <RH_RF95.h>
#include "settings.h"

#ifdef ESP8266
#define RFM_CS D8
#define RFM_INT D1
#else
#define RFM_CS 10
#define RFM_INT 2
#endif
#define RFM_FREQ 433.92

#define PAYLOADSIZE 128

//#define SYSLOG serialLocal // define as "none" to disable
#define SYSLOG ramBuf
#if SYSLOG==none
class None {
  public:
    void log(...) {}
    void logf(...) {}
};
None none;
#endif
#define MAX_LOG_LENGTH 128
#define NUM_MESSAGES 50

#if SYSLOG==ramBuf
#include <ESP8266WebServer.h>
ESP8266WebServer server(80);
char messages[NUM_MESSAGES][MAX_LOG_LENGTH] = {{0}};
byte nextBuf = 0;
class RamBuf {
  public:
    void log(int level, const char* str) {
      memcpy(messages[nextBuf], str, MAX_LOG_LENGTH);
      nextBuf = (nextBuf + 1) % NUM_MESSAGES;
    }
    void logf(int level, const char *format, ...) {
      va_list ap;
      va_start(ap, format);
      vsnprintf(messages[nextBuf], MAX_LOG_LENGTH, format, ap);
      va_end(ap);
      nextBuf = (nextBuf + 1) % NUM_MESSAGES;
    }
};
RamBuf ramBuf;

#endif

#include <stdarg.h>
#include <Syslog.h> // Needs to be included to get access to LOG_INFO, etc
#if SYSLOG==serialLocal
class SerialLocal {
  public:
    void log(int level, const char* str) {
      Serial.println(str);
    }
    void logf(int level, const char *format, ...) {
      char buf[MAX_LOG_LENGTH]; // resulting string limited to 128 chars
      va_list ap;
      va_start(ap, format);
      vsnprintf(buf, MAX_LOG_LENGTH, format, ap);
      va_end(ap);
      Serial.println(buf);
    }
};
SerialLocal serialLocal;
#endif

RH_RF95 radio(RFM_CS, RFM_INT);

RH_RF95::ModemConfigChoice modem_config = RH_RF95::Bw125Cr48Sf4096;

// This is where the pin TX pin of your GPS sensor is connected to the arduino
#ifdef ESP8266
#define GPS_PIN D2
#define UNUSED_PIN A0
#else
#define GPS_PIN A0
#define UNUSED_PIN A5
#endif
// GPS Baud rate (note this is not the same as your serial montor baudrate). Most GPS modules uses 9600 bps.
static const uint32_t GPSBaud = 9600;
// Offset hours adjustment from gps time (UTC)
const int offset = 1;
#include <TimeLib.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
// TinyGPS++ is used for parsing serial gps data
TinyGPSPlus gps;
// The serial connection to the GPS device
// A5 pin can be left unconnected
SoftwareSerial ss(GPS_PIN, UNUSED_PIN);
//HardwareSerial ss = Serial;
unsigned int sendInterval = 30000; // Send interval in milliseconds

// Last time message was sent
unsigned long lastSent = 0;

// Some buffers
char latBuf[11];
char lngBuf[11];
char altBuf[6];
char payload[30];
char sz[64] = {0};

void setup() {
#if SYSLOG==ramBuf
  WiFi.mode(WIFI_AP);
  WiFi.softAP(MY_ESP8266_AP_SSID, MY_ESP8266_AP_PASSWORD);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", []() {
    String s = "";
    for (int pos = nextBuf; pos < NUM_MESSAGES; pos++) {
      if (messages[pos][0] != '\0') {
        s += messages[pos];
        s += '\n';
      }
    }
    if (nextBuf != 0) {
      for (int pos = 0; pos < nextBuf; pos++) {
        if (messages[pos][0] != '\0') {
          s += messages[pos];
          s += '\n';
        }
      }
    }
    Serial.println(s);
    server.send(200, "text/plan", s);
  });
  server.begin();
#endif

  //#if SYSLOG==serialLocal
  Serial.begin(115200);
  //#endif
  SYSLOG.log(LOG_INFO, "FatLink mobile node starting up");
  while (!radio.init()) {
    SYSLOG.log(LOG_INFO, "LoRa radio init failed");
    delay(5000);
  }
  SYSLOG.log(LOG_INFO, "LoRa radio init OK!");

  if (!radio.setFrequency(RFM_FREQ)) {
    SYSLOG.log(LOG_INFO, "setFrequency failed");
    delay(5000);
  }

  radio.setModemConfig(modem_config);
  radio.setTxPower(23);
  // Set baudrate form gps communication
  ss.begin(GPSBaud);

  SYSLOG.log(LOG_INFO, "Setup finished");
}

void loop() {
  check_incoming();

  unsigned long currentTime = millis();

  // Evaluate if it is time to send a new position
  bool timeToSend = currentTime - lastSent > sendInterval;

  // Read gps data
  while (ss.available())
    gps.encode(ss.read());

  if (timeToSend) {
    // Sync gps time with Arduino
    updateTime();

    // Send current gps location
    if (gps.location.isValid() && gps.altitude.isValid()) {
      // Build position and altitude string to send
      dtostrf(gps.location.lat(), 1, 6, latBuf);
      dtostrf(gps.location.lng(), 1, 6, lngBuf);
      dtostrf(gps.altitude.meters(), 1, 0, altBuf);
      sprintf(payload, "%s;%s;%s", latBuf, lngBuf, altBuf);
    } else {
      if (millis() > 5000 && gps.charsProcessed() < 10) {
        sprintf(payload, "%s", "No GPS data: check wiring");
      } else {
        sprintf(payload, "%s%s", "No GPS data yet...", sz);
      }
    }
    radio.send((uint8_t *)payload, strlen(payload));
    SYSLOG.log(LOG_INFO, payload);
    radio.waitPacketSent();
    SYSLOG.log(LOG_INFO, "Message sent");
    lastSent = currentTime;
  }

#if SYSLOG==ramBuf
  server.handleClient();
#endif

  delay(100); // Let the ESP go to power save
}

void set_led(bool state) {
  //TODO: Set global variable, trigger websocket, switch physical led
}

void check_incoming() {
  if (radio.waitAvailableTimeout(10))
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (radio.recv(buf, &len))
    {
      RH_RF95::printBuffer("Received: ", buf, len);
      SYSLOG.log(LOG_INFO, "Got: ");
      SYSLOG.log(LOG_INFO, (char*)buf);
      SYSLOG.logf(LOG_INFO, "RSSI: %i", radio.lastRssi());
      SYSLOG.logf(LOG_INFO, "SNR: %i", radio.lastSNR());
    }
    else
    {
      SYSLOG.log(LOG_INFO, "Receive failed");
    }
  }
}

void updateTime()
{
  TinyGPSDate d = gps.date;
  TinyGPSTime t = gps.time;
  if (d.isValid() && t.isValid()) {
    // set the Time to the latest GPS reading if less then 0.2 seconds old
    setTime(t.hour(), t.minute(), t.second(), d.day(), d.month(), d.year());
    adjustTime(offset * SECS_PER_HOUR);
    sprintf(sz, "%02d-%02d-%02d %02d:%02d:%02d", year(), month(), day(),  hour(), minute(), second());
    SYSLOG.log(LOG_INFO, sz);
    return;
  }
  SYSLOG.log(LOG_INFO, "Unable to adjust time from GPS");
}

#ifdef ESP8266
long readVcc() {
  //TODO Implement battery reading for ESP if applicable
  return 0;
}
#else
long readVcc() {
  // From http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
#endif
