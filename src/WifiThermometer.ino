/**
 * Uses a p channel mosfet for high level switching
 * requires a NPN transistor with 10k Ohm resistor in front of NPN to reduce the 3v out of the MC to 1.6V
 * and 10k Ohm as well after NPN to VCC (source 10V)
 * PWM signal from MC ground with 10k Ohm resistor
 * alternative code for relays is commented out
 * relays are accessed through port expander
 *
 */

#include <Arduino.h>

/*
   Test the PWM function of digital PINs
 */
#include <Streaming.h>
#include <DHT.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <Timezone.h>
#include <ArduinoOTA.h>
#include <Constants.h>
#include <FS.h>
#include <Wire.h>
//#include <pcf8574a.h>
//#include <LiquidCrystal_I2C.h>
#include <RH_ASK.h>
#include <BME280.h>
#include <Adafruit_Sensor.h>
#include "SSD1306.h"


// Set the LCD address to 0x27 for a 16 chars and 2 line display
//LiquidCrystal_I2C lcd(0x27, 20, 4);


//temparure sensor
BME280 bme;                   // Default : forced mode, standby time = 1000 ms
                              // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,
/**
 * @brief mDNS and OTA Constants
 * @{
 */

#define HOSTNAME "ESP8266-OTA-" ///< Hostename. The setup function adds the Chip ID at the end.
/// @}
/**
 * @brief Default WiFi connection information.
 * @{
 */
const char* ap_default_ssid = "esp8266"; ///< Default SSID.
const char* ap_default_psk = "esp8266esp8266"; ///< Default PSK.
/// @}

/// Uncomment the next line for verbose output over UART.
//#define SERIAL_VERBOSE
/**
 * @brief Read WiFi connection information from file system.
 * @param ssid String pointer for storing SSID.
 * @param pass String pointer for storing PSK.
 * @return True or False.
 *
 * The config file have to containt the WiFi SSID in the first line
 * and the WiFi PSK in the second line.
 * Line seperator can be \r\n (CR LF) \r or \n.
 */

ESP8266WebServer server ( 80 );

unsigned int localPort = 2390;      // local port to listen for UDP packets

/* Don't hardwire the IP address or we won't get the benefits of the pool.
    Lookup the IP address for the host name instead */
//IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "pool.ntp.org";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;

//time zone setting
//Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};     //Central European Summer Time
TimeChangeRule CET = {"CET ", Last, Sun, Oct, 3, 60};       //Central European Standard Time
Timezone CE(CEST, CET);

#define DHTPIN 0
//#define RFPIN D8
//RH_ASK rf_driver(2000, D8);  //will initialise the driver at 2000 bps, recieve on GPIO2, transmit on GPIO4, PTT on GPIO5
#define DHTTYPE DHT22 //DHT11, DHT21, DHT22
DHT dht(DHTPIN, DHTTYPE);

// only for ESP-91
#define SDAPIN 0
#define SCLPIN 2
//for all other ESP
//#define SDAPIN 4
//#define SDAPIN 5

//SSD1306  display(ADDRESS, SDA, SDC);
SSD1306 display(0x3c, SDAPIN, SCLPIN);

//#define CAPBTN D3
//#define LEDPIN D3
//#define GATEPIN D7
//#define POTIPIN A0  // no poti anymore
//#define VOLTAGESENSORPIN A0
//pcf8574a mcp(0x38); //instance
//#define I2CDHT 0
// #define I2CBTNAUTO 5
// #define I2CBTNUP  6
// #define I2CBTNDOWN  7
// #define BTNUP D5
// #define BTNDOWN D6
// #define BTNAUTO D7
// #define RELAY0 0
// #define RELAY1 1
// #define RELAY2 2
// #define RELAY3 3

//#define LEDAUTOPIN D8
//#define LEDMANPIN D9

int stateUp = HIGH;      // the current state of the output pin
int readingUp;           // the current reading from the input pin
int previousAuto;
int readingAuto;
int previousUp = LOW;    // the previous reading from the input pin
int stateDown = HIGH;      // the current state of the output pin
int readingDown;           // the current reading from the input pin
int previousDown = LOW;    // the previous reading from the input pin
int readingCapBtn;

bool enableDht = ENABLEDHT;
bool enableBme = ENABLEBME;

bool readEnvironment = true;
bool turnDisplayOff = false;
bool displayAlarmSet = false;
unsigned long displayTime;
bool displayOn = true;
float temperature(NAN), humidity(NAN), pressure(NAN);

// ntp timestamp
unsigned long ulSecs2000_timer=0;
// storage for Measurements; keep some mem free; allocate remainder
#define KEEP_MEM_FREE 10240
#define MEAS_SPAN_H 1
unsigned long ulMeasCount=0;    // values already measured
unsigned long ulNoMeasValues=0; // size of array
unsigned long ulMeasDelta_ms;   // distance to next meas time
unsigned long ulNextMeas_ms;    // next meas time
time_t *pulTime;         // array for time points of measurements
float *pfTemp,*pfHum,*pfPres;         // array for temperature and humidity measurements

float humidities[100];
float tempertures[100];
time_t times[100];
long indexTable = 0;

String station_ssid = WIFI_SSID;
String station_psk = WIFI_PASSWORD;

time_t getTime() {
        return CE.toLocal(getNtpTime());      //adjust utc to local time
}
//
// bool loadConfig(String *ssid, String *pass)
// {
//         // open file for reading.
//         File configFile = SPIFFS.open("/cl_conf.txt", "r");
//         if (!configFile)
//         {
//                 Serial.println("Failed to open cl_conf.txt.");
//
//                 return false;
//         }
//
//         // Read content from config file.
//         String content = configFile.readString();
//         configFile.close();
//
//         content.trim();
//
//         // Check if ther is a second line available.
//         int8_t pos = content.indexOf("\r\n");
//         uint8_t le = 2;
//         // check for linux and mac line ending.
//         if (pos == -1)
//         {
//                 le = 1;
//                 pos = content.indexOf("\n");
//                 if (pos == -1)
//                 {
//                         pos = content.indexOf("\r");
//                 }
//         }
//
//         // If there is no second line: Some information is missing.
//         if (pos == -1)
//         {
//                 Serial.println("Infvalid content.");
//                 Serial.println(content);
//
//                 return false;
//         }
//
//         // Store SSID and PSK into string vars.
//         *ssid = content.substring(0, pos);
//         *pass = content.substring(pos + le);
//
//         ssid->trim();
//         pass->trim();
//
// #ifdef SERIAL_VERBOSE
//         Serial.println("----- file content -----");
//         Serial.println(content);
//         Serial.println("----- file content -----");
//         Serial.println("ssid: " + *ssid);
//         Serial.println("psk:  " + *pass);
// #endif
//
//         return true;
// } // loadConfig
// /**
//  * @brief Save WiFi SSID and PSK to configuration file.
//  * @param ssid SSID as string pointer.
//  * @param pass PSK as string pointer,
//  * @return True or False.
//  */
// bool saveConfig(String *ssid, String *pass)
// {
//         // Open config file for writing.
//         File configFile = SPIFFS.open("/cl_conf.txt", "w");
//         if (!configFile)
//         {
//                 Serial.println("Failed to open cl_conf.txt for writing");
//
//                 return false;
//         }
//
//         // Save SSID and PSK.
//         configFile.println(*ssid);
//         configFile.println(*pass);
//
//         configFile.close();
//
//         return true;
// } // saveConfig


void setup() {

        //pinMode(LEDPIN, OUTPUT);
        //pinMode(DHTPIN, INPUT);
        //pinMode(CAPBTN, INPUT);
        //pinMode(BTNUP, INPUT);
        //pinMode(BTNDOWN, INPUT);
        //pinMode(BTNAUTO, INPUT);

        //pinMode(LEDAUTOPIN, OUTPUT);
        //pinMode(LEDMANPIN, OUTPUT);

        //pinMode(VOLTAGESENSORPIN, INPUT);

        // set the pins for I2C
        Wire.begin(SDAPIN,SCLPIN);
        //Wire.begin(0,2);
        //Wire.setClockStretchLimit(1500);    // in µs
        // mcp.begin();
        // mcp.gpioPinMode(I2CDHT,INPUT);
        //mcp.gpioPinMode(RELAY0,OUTPUT);
        // mcp.gpioPinMode(RELAY1,OUTPUT);
        // mcp.gpioPinMode(RELAY2,OUTPUT);
        // mcp.gpioPinMode(RELAY3,OUTPUT);
        //
        // mcp.gpioPinMode(I2CBTNAUTO,INPUT);
        // mcp.gpioPinMode(I2CBTNDOWN,INPUT);
        // mcp.gpioPinMode(I2CBTNUP,INPUT);

// initialize the LCD
        //lcd.init();
        //lcd.home();
        //lcd.print("Hello, World!");

        Serial.begin(115200);


        // Initialising the UI will init the display too.
        display.init();
        display.displayOn();
        display.flipScreenVertically();
        // Alarm.alarmOnce(10, calledOnce);
        // // display.setFont(ArialMT_Plain_10);

        delay(100);

        Serial.println("\r\n");
        Serial.print("Chip ID: 0x");
        Serial.println(ESP.getChipId(), HEX);

        // Set Hostname.
        String hostname(HOSTNAME);
        hostname += String(ESP.getChipId(), HEX);
        WiFi.hostname(hostname);

        // Print hostname.
        Serial.println("Hostname: " + hostname);
        //Serial.println(WiFi.hostname());


        // // Initialize file system.
        // if (!SPIFFS.begin())
        // {
        //         Serial.println("Failed to mount file system");
        //         return;
        // }
        //
        // // Load wifi connection information.
        // if (!loadConfig(&station_ssid, &station_psk))
        // {
        //         station_ssid = WIFI_SSID;
        //         station_psk = WIFI_PASSWORD;
        //
        //         Serial.println("No saved WiFi connection information available.");
        //
        // }

        station_ssid = WIFI_SSID;
        station_psk = WIFI_PASSWORD;

        // Check WiFi connection
        // ... check mode
        if (WiFi.getMode() != WIFI_STA)
        {
                WiFi.mode(WIFI_STA);
                delay(10);
        }

        // ... Compare file config with sdk config.
        if (WiFi.SSID() != station_ssid || WiFi.psk() != station_psk)
        {
                Serial.println("WiFi config changed.");

                // ... Try to connect to WiFi station.
                WiFi.begin(station_ssid.c_str(), station_psk.c_str());

                // ... Pritn new SSID
                Serial.print("new SSID: ");
                Serial.println(WiFi.SSID());

                // ... Uncomment this for debugging output.
                //WiFi.printDiag(Serial);
        }
        else
        {
                // ... Begin with sdk config.
                WiFi.begin();
        }

        Serial.println("Wait for WiFi connection.");

        // ... Give ESP 10 seconds to connect to station.
        unsigned long startTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000)
        {
                Serial.write('.');
                //Serial.print(WiFi.status());
                delay(500);
        }
        Serial.println();

        // Check connection
        if(WiFi.status() == WL_CONNECTED)
        {
                // ... print IP Address
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
        }
        else
        {
                Serial.println("Can not connect to WiFi station. Go into AP mode.");

                // Go into software AP mode.
                WiFi.mode(WIFI_AP);

                delay(10);

                WiFi.softAP(ap_default_ssid, ap_default_psk);

                Serial.print("IP address: ");
                Serial.println(WiFi.softAPIP());
        }

        // Start OTA server.
        ArduinoOTA.setHostname((const char *)hostname.c_str());
        ArduinoOTA.setPassword((const char *) OTAPASSWORD);
        ArduinoOTA.begin();

        // Start the server
        server.on ( "/", handleRoot );
        server.on ( "/temperature", handleTemperature );
        server.on ( "/humidity", handleHumidity );
        server.on ( "/pressure", handlePressure );
        server.on ( "/time", handleTime );
        server.on ( "/table", handleTable );

        server.onNotFound ( handleNotFound );
        server.begin();
        Serial.println("Server started");

        // Print the IP address
        Serial.print("Use this URL to connect: ");
        Serial.print("http://");
        Serial.print(WiFi.localIP());
        Serial.println("/");

        Serial.println("Starting UDP");
        udp.begin(localPort);
        Serial.print("Local port: ");
        Serial.println(udp.localPort());

        if (enableDht) {
                Serial << "enabling DHT" << endl;
                dht.begin();
        }

        if (enableBme) {
                Serial << "enabling BME" << endl;
                bme.begin();
        }

        //setSyncProvider((time_t)getNtpTime());
        Serial << "get NTP time" << endl;
        setSyncProvider(getTime);
        //setTime((time_t)getNtpTime());
        //setTime(15, 39, 0, 9, 3, 2016);
        Serial.println(now());
        Serial << ("Zeit: ") << hour() << ":" << minute() << ":" << second() << endl;

        // if (!rf_driver.init())
        //         Serial.println("init failed");

        Alarm.timerRepeat(MEASUREINTERVAL, repeat);

        display.clear();
        display.displayOn();
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 32, "Ready..." );
        display.display();
        Alarm.timerOnce(5, calledOnce);

        //ulNoMeasValues = 1000;
        // allocate ram for data storage
        #define KEEP_MEM_FREE 10240
        uint32_t free = ESP.getFreeHeap() - KEEP_MEM_FREE;
        ulNoMeasValues = free / (sizeof(float)*3+sizeof(unsigned long)); // humidity & temp --> 2 + time
        pulTime = new long int[ulNoMeasValues];
        pfTemp = new float[ulNoMeasValues];
        pfHum = new float[ulNoMeasValues];
        pfPres = new float[ulNoMeasValues];
        Serial << "ulNoMeasValues: " << ulNoMeasValues << endl;

}
void LED_SimpleBlink(int LEDPin, long interval = 60, long duration = 5) {
        for (int i = 0; i < duration; i++) {
                analogWrite(LEDPin, 1023);
                delay(interval);
                analogWrite(LEDPin, 0);
                delay(interval);
        }
}
void loop() {


        // {
        //         uint8_t buf[12];
        //         uint8_t buflen = sizeof(buf);
        //         if (rf_driver.recv(buf, &buflen)) // Non-blocking
        //         {
        //                 int i;
        //                 // Message with a good checksum received, dump it.
        //                 Serial.print("Message: ");
        //                 Serial.println((char*)buf);
        //         } { Serial.println("no data received"); }
        // }

        if (readEnvironment) {
                // float humidity = dht.readHumidity();
                // float temperature = dht.readTemperature();

                // humidities[indexTable] = dht.readHumidity();
                // tempertures[indexTable] = dht.readTemperature();
                // times[indexTable] = now();
                // //pulTime[ulMeasCount%ulNoMeasValues] = millis()/1000+ulSecs2000_timer;
                // indexTable++;
                if (enableDht) {
                        pfHum[ulMeasCount%ulNoMeasValues] = dht.readHumidity();
                        pfTemp[ulMeasCount%ulNoMeasValues] = dht.readTemperature();
                        pfPres[ulMeasCount%ulNoMeasValues] = 0.0;
                        pulTime[ulMeasCount%ulNoMeasValues] = now();
                }
                if (enableBme) {
                        pfHum[ulMeasCount%ulNoMeasValues] = bme.ReadHumidity();
                        pfTemp[ulMeasCount%ulNoMeasValues] = bme.ReadTemperature(true);
                        pfPres[ulMeasCount%ulNoMeasValues] = bme.ReadPressure(1);
                        pulTime[ulMeasCount%ulNoMeasValues] = now();
                }
                if (ulMeasCount >= ulNoMeasValues) {
                        ulMeasCount = 0;
                } else {
                        ulMeasCount++;
                }

                readEnvironment = false;

                Serial.printf("Heap size: %u\n", ESP.getFreeHeap());
        }
        //webserver
        server.handleClient();
        ArduinoOTA.handle();
        // Set LEDPIN according to the request
        //digitalWrite(LEDPIN, value);

        //readEnvironment();

        // if (millis()>=ulNextMeas_ms)
        // {
        //         ulNextMeas_ms = millis()+ulMeasDelta_ms;
        //
        //         pfHum[ulMeasCount%ulNoMeasValues] = dht.readHumidity();
        //         pfTemp[ulMeasCount%ulNoMeasValues] = dht.readTemperature();
        //         //pulTime[ulMeasCount%ulNoMeasValues] = millis()/1000+ulSecs2000_timer;
        //         pulTime[ulMeasCount%ulNoMeasValues] = now();
        //
        //         Serial.print("Logging Temperature: ");
        //         Serial.print(pfTemp[ulMeasCount%ulNoMeasValues]);
        //         Serial.print(" deg Celsius - Humidity: ");
        //         Serial.print(pfHum[ulMeasCount%ulNoMeasValues]);
        //         Serial.print("% - Time: ");
        //         Serial.println(pulTime[ulMeasCount%ulNoMeasValues]);
        //         // File file = SPIFFS.open("/data.txt", "a");
        //         // if (!f) {
        //         //     Serial.println("file open failed");
        //         // } else {
        //         //    file.println()
        //         // }
        //         ulMeasCount++;
        // }

        Alarm.delay(200);

        //delay(200);
        //yield();
}

void calledOnce() {
        Serial << "displayoff called" << endl;
        turnDisplayOff = true;
        displayAlarmSet = false;
}

void repeat() {
        readEnvironment = true;
}

void writeDisplay() {
        float temperature(NAN), humidity(NAN), pressure(NAN);

        if ((enableDht)) {
                ////dht22 sensor
                Serial.println("Reading DHT22");
                humidity = dht.readHumidity(); //Luftfeuchte auslesen
                temperature = dht.readTemperature(); //Temperatur auslesen
                Serial << ("DHT22 - Luftfeuchte: ") << humidity << " Temperatur: " << temperature << " C" << " Pressure: " << pressure <<endl;
                pressure = 0.0;
                // Prüfen ob eine gültige Zahl zurückgegeben wird. Wenn NaN (not a number) zurückgegeben wird, dann Fehler ausgeben.
                if ((isnan(temperature) || isnan(humidity)) )
                {
                        Serial.println("Cannot read DHT22");
                }
        }
        if (enableBme) {
                Serial.println("Reading BME");
                bool metric = true;
                uint8_t pressureUnit(1); // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
                //bme.ReadData(pressure, temperature, humidity, metric, pressureUnit);          // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
                // Alternatives to ReadData():
                temperature = bme.ReadTemperature(metric);
                pressure = bme.ReadPressure(1);
                humidity = bme.ReadHumidity();
                Serial << ("BME280 - Luftfeuchte: ") << humidity << " Temperatur: " << temperature << " C" << " Pressure: " << pressure <<endl;
        }
        display.setTextAlignment(TEXT_ALIGN_LEFT);
        display.setFont(ArialMT_Plain_16);
        display.clear();
        display.displayOn();

        char buffer[24];
        sprintf(buffer, "Time: %02d:%02d:%02d", hour(), minute(), second());
        display.drawString(0, 0, buffer );

        char str_temp[3];
        dtostrf(temperature, 2, 1, str_temp); // da %f nicht im arduino implementiert is
        sprintf(buffer, "Temp: %sC", str_temp);
        display.drawString(0, 16, buffer );
        Serial << buffer << endl;

        dtostrf(humidity, 2, 0, str_temp);         // da %f nicht im arduino implementiert is
        sprintf(buffer, "Hum: %s%%", str_temp);
        display.drawString(0, 32, buffer );
        Serial << humidity << " / " << buffer  << " / " << str_temp << endl;
        Serial << buffer << endl;

        dtostrf(pressure, 2, 0, str_temp); // da %f nicht im arduino implementiert is
        sprintf(buffer, "Pres: %shPa", str_temp);
        display.drawString(0, 48, buffer );
        Serial << buffer << endl;

        display.display();
}


void handleTemperature() {
        float value = bme.ReadTemperature(true);
        if (isnan(value)) {
                value = dht.readTemperature(); //Temperatur auslesen
        }
        char str_temp[3];
        dtostrf(value, 2, 1, str_temp);
        server.send ( 200, "text/html", str_temp );
}

void handleHumidity() {
        float value = bme.ReadHumidity();
        if (isnan(value)) {
                value = dht.readHumidity(); //Temperatur auslesen
        }
        char str_temp[3];
        dtostrf(value, 2, 0, str_temp);
        server.send ( 200, "text/html", str_temp );
}

void handlePressure() {
        float value = bme.ReadPressure(1);
        char str_temp[3];
        dtostrf(value, 4, 0, str_temp);
        server.send ( 200, "text/html", str_temp );
}

void handleTime() {
        char buffer[24];
        sprintf(buffer, "%02d:%02d:%02d", hour(), minute(), second());
        server.send ( 200, "text/html", buffer );

}

void handleRoot() {
        //readEnvironmentVars();
        char temp[600];
        int sec = millis() / 1000;
        int min = sec / 60;
        int hr = min / 60;
        //<meta http-equiv='refresh' content='5'/> //veranlasst den refresh nach 5s

        String answer = \
                "<html>\
  <head>\
    <title>";
        answer += NAME;
        answer += "</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1><a href=\"/\">";
        answer += NAME;
        answer += "</a></h1>";
        char buffer[24];
        //Serial << ("Zeit: ") << hour() << ":" << minute() << ":" << second() << endl;
        answer +="<p>";
        sprintf(buffer, "Time: %02d:%02d:%02d %02d.%02d.%04d", hour(), minute(), second(), day(), month(), year());
        answer += buffer;
        answer +="</p>";

        if (enableBme) {
                float temperature(NAN), humidity(NAN), pressure(NAN);
                Serial.println("Reading BME");
                bool metric = true;
                uint8_t pressureUnit(1); // unit: B000 = Pa, B001 = hPa, B010 = Hg, B011 = atm, B100 = bar, B101 = torr, B110 = N/m^2, B111 = psi
                //bme.ReadData(presssure, temperature, humidity, metric, pressureUnit);          // Parameters: (float& pressure, float& temp, float& humidity, bool hPa = true, bool celsius = false)
                // Alternatives to ReadData():
                temperature = bme.ReadTemperature(true);
                pressure = bme.ReadPressure(1);
                humidity = bme.ReadHumidity();
                Serial << ("Luftfeuchte: ") << humidity << " Temperatur: " << temperature << " C" << " Pressure: " << pressure <<endl;

                answer += "<p>";
                char str_temp[3];
                dtostrf(temperature, 2, 1, str_temp); // da %f nicht im arduino implementiert is
                sprintf(buffer, "BME280: Temp: %sC", str_temp);
                answer += buffer;

                dtostrf(humidity, 2, 0, str_temp);         // da %f nicht im arduino implementiert is
                sprintf(buffer, " Hum: %s%%", str_temp);
                answer += buffer;

                dtostrf(pressure, 4, 0, str_temp); // da %f nicht im arduino implementiert is
                sprintf(buffer, " Pres: %shPa", str_temp);
                answer += buffer;
                answer +="</p>";

                // char str_temp[3];
                // char str_temp2[3];
                // char str_temp3[3];
                // dtostrf(temperature, 2, 1, str_temp); // da %f nicht im arduino implementiert is
                // dtostrf(humidity, 2, 0, str_temp2); // da %f nicht im arduino implementiert is
                // dtostrf(presssure, 2, 0, str_temp3); // da %f nicht im arduino implementiert is
                // char buffer[24];
                // sprintf(buffer, "BME280: Temp: %sC Hum: %s%% Press: %shPa", str_temp, str_temp2, str_temp3);
                // answer += "<p>";
                // answer += buffer;
                // answer += "</p>";
        }
        if (enableDht) {
                ////dht22 sensor
                Serial.println("Reading DHT22");
                float humidity = dht.readHumidity(); //Luftfeuchte auslesen
                float temperature = dht.readTemperature(); //Temperatur auslesen

                // Prüfen ob eine gültige Zahl zurückgegeben wird. Wenn NaN (not a number) zurückgegeben wird, dann Fehler ausgeben.
                if ((isnan(temperature) || isnan(humidity)) )
                {
                        Serial.println("Cannot read DHT22");
                }
                else
                {
                        Serial << ("Luftfeuchte: ") << humidity << " Temperatur: " << temperature << " C" <<endl;
                }
                answer += "<p>";
                char str_temp[3];
                dtostrf(temperature, 2, 1, str_temp); // da %f nicht im arduino implementiert is
                sprintf(buffer, "DHT22: Temp: %sC", str_temp);
                answer += buffer;

                dtostrf(humidity, 2, 0, str_temp);         // da %f nicht im arduino implementiert is
                sprintf(buffer, " Hum: %s%%", str_temp);
                answer += buffer;
                answer +="</p>";
        }

        answer += "<p><A HREF=\"javascript:history.go(0)\">Click to refresh the page</A></p>";
        answer += "<p><A HREF=\"/table\">Table view</A></p>";
        answer += "</body></html>";
        server.sendHeader("Cache-Control", "no-cache");
        server.send ( 200, "text/html", answer );
        //digitalWrite ( led, 0 );
}

void handleNotFound() {
        //digitalWrite ( led, 1 );
        String message = "File Not Found\n\n";
        message += "URI: ";
        message += server.uri();
        message += "\nMethod: ";
        message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
        message += "\nArguments: ";
        message += server.args();
        message += "\n";

        for ( uint8_t i = 0; i < server.args(); i++ ) {
                message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
        }

        server.send ( 404, "text/plain", message );
        //digitalWrite ( led, 0 );
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
        Serial.println("sending NTP packet...");
        // set all bytes in the buffer to 0
        memset(packetBuffer, 0, NTP_PACKET_SIZE);
        // Initialize values needed to form NTP request
        // (see URL above for details on the packets)
        packetBuffer[0] = 0b11100011; // LI, Version, Mode
        packetBuffer[1] = 0; // Stratum, or type of clock
        packetBuffer[2] = 6; // Polling Interval
        packetBuffer[3] = 0xEC; // Peer Clock Precision
        // 8 bytes of zero for Root Delay & Root Dispersion
        packetBuffer[12]  = 49;
        packetBuffer[13]  = 0x4E;
        packetBuffer[14]  = 49;
        packetBuffer[15]  = 52;

        // all NTP fields have been given values, now
        // you can send a packet requesting a timestamp:
        udp.beginPacket(address, 123); //NTP requests are to port 123
        udp.write(packetBuffer, NTP_PACKET_SIZE);
        udp.endPacket();
}

long int getNtpTime() {
        for (int i = 0; 3; i++) {
                //get a random server from the pool
                WiFi.hostByName(ntpServerName, timeServerIP);

                sendNTPpacket(timeServerIP); // send an NTP packet to a time server
                // wait to see if a reply is available
                delay(1000);

                int cb = udp.parsePacket();
                if (!cb) {
                        Serial.println("no packet yet");
                }
                else {
                        Serial.print("packet received, length=");
                        Serial.println(cb);
                        // We've received a packet, read the data from it
                        udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

                        //the timestamp starts at byte 40 of the received packet and is four bytes,
                        // or two words, long. First, esxtract the two words:

                        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
                        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
                        // combine the four bytes (two words) into a long integer
                        // this is NTP time (seconds since Jan 1 1900):
                        unsigned long secsSince1900 = highWord << 16 | lowWord;
                        Serial.print("Seconds since Jan 1 1900 = " );
                        Serial.println(secsSince1900);

                        // now convert NTP time into everyday time:
                        Serial.print("Unix time = ");
                        // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
                        const unsigned long seventyYears = 2208988800UL;
                        // subtract seventy years:
                        unsigned long epoch = secsSince1900 - seventyYears;
                        // print Unix time:
                        Serial.println(epoch);
                        return epoch;
                }
        }
}
// crashes this program but does work otherwhere
void LED_Blink(int LEDPin, long interval = 1000, long duration = 5000) {
        int ledState = LOW;
        unsigned long previousMillis = 0;
        unsigned long startMillis = millis();
        unsigned long currentMillis = 0;
        while   ((millis() - startMillis) <= duration) {
                currentMillis = millis();
                if (currentMillis - previousMillis >= interval) {
                        // save the last time you blinked the LED
                        previousMillis = currentMillis;
                        // if the LED is off turn it on and vice-versa:
                        if (ledState == LOW) {
                                ledState = HIGH;
                        } else {
                                ledState = LOW;
                        }
                        // set the LED with the ledState of the variable:
                        //digitalWrite(LEDPIN, ledState);
                }
        }
        //delay(10);
        yield();
}
void handleTable ()
{
        unsigned long ulLength=0;

        // here we build a big table.
        // we cannot store this in a string as this will blow the memory
        // thus we count first to get the number of bytes and later on
        // we stream this out
        if (ulMeasCount==0)
        {
                String sAnswer = "No data available yet.";
                server.send ( 200, "text/plain", sAnswer );
        }
        else
        {
                unsigned long ulEnd;
                if (ulMeasCount>ulNoMeasValues)
                {
                        ulEnd=ulMeasCount-ulNoMeasValues;
                }
                else
                {
                        ulEnd=0;
                }

                String sAnswer = \
                        "<html>\
          <head>\
            <title>";
                sAnswer += NAME;
                sAnswer += "</title>\
            <style>\
              body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
            </style>\
          </head>\
          <body>\
          <h1><a href=\"/\">";
                sAnswer += NAME;
                sAnswer += "</a></h1>";
                //sAnswer += "<table>";
                sAnswer += "<table style=\"width:100%\"><tr><th>Time</th><th>Temperature</th><th>Humidity</th><th>Pressure</th></tr>";
                sAnswer += "<style>table, th, td {border: 2px solid black; border-collapse: collapse;} th, td {padding: 5px;} th {text-align: left;}</style>";
                for (unsigned long li=ulMeasCount; li>ulEnd; li--)
                {
                        unsigned long ulIndex=(li-1)%ulNoMeasValues;
                        sAnswer += "<tr><td>";
                        char buffer[24];
                        sprintf(buffer, "%02d:%02d:%02d %02d.%02d.%04d", \
                                hour(pulTime[ulIndex]), \
                                minute(pulTime[ulIndex]), \
                                second(pulTime[ulIndex]), \
                                day(pulTime[ulIndex]), \
                                month(pulTime[ulIndex]), \
                                year(pulTime[ulIndex]));
                        sAnswer += buffer;
                        //sAnswer += epoch_to_string(pulTime[ulIndex]).c_str();
                        sAnswer += "</td><td>";
                        sAnswer += pfTemp[ulIndex];
                        sAnswer += "</td><td>";
                        sAnswer += pfHum[ulIndex];
                        sAnswer += "</td><td>";
                        sAnswer += pfPres[ulIndex];
                        sAnswer += "</td></tr>";


                }
                // for (unsigned long li=indexTable; li>=0; li--)
                // {
                //         sAnswer += "<tr><td>";
                //         char buffer[24];
                //         sprintf(buffer, "%02d:%02d:%02d %02d.%02d.%04d", \
                //                 hour(time_t times[index]), minute(time_t times[index]), second(time_t times[index]), day(time_t times[index]), month(time_t times[index]), year(time_t times[index]));
                //         sAnswer += buffer;
                //         //sAnswer += epoch_to_string(pulTime[ulIndex]).c_str();
                //         sAnswer += "</td><td>";
                //         sAnswer += tempertures[index];
                //         sAnswer += "</td><td>";
                //         sAnswer += humidities[index];
                //         sAnswer += "</td></tr>";
                //
                // }

                // remaining chunk
                sAnswer+="</table>";
                sAnswer += "</body></html>";
                server.sendHeader("Cache-Control", "no-cache");
                server.send ( 200, "text/html", sAnswer );
                //pclient->print(sTable);
                //pclient->write(sTable.c_str(),sTable.length());
        }

}
