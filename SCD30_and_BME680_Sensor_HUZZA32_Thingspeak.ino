
// CO2 measurement with the following components:
// board:  Adafruit Feather HUZZAH-32
// wing:   by Guido Burger (probably prototype: has BME680 and either one Neopixel or three LEDs on it
// Sensor: Sensirion SCD30  &  Bosch BME680
// 
// connected via WiFi
// operations

// Sensor:   Sensirion SCD30             (Mouser 403-SCD30 (47.--))
// 
// open item (--):   **  solved: (++):
// -- implement forced calibration
// -- battery usage not tested yet
// -- SCD30 does not sleep (20-75mA power usage) --> implement setMeasurementInterval(uint16_t interval)
// -- long term usage not tested yet.  possible to attach a solar cell loading the LiPo?
// -- Sensirion SPS30 particulate matter sensor
// 
// -- other sensors should be added e.g. SPS30 Particle Sensor (Mouser 403-SPS30 (39.--)), SVM30-J Air Quality Sensor with SGP30 and SHTC1(Mouser 403-SVM30-J (20.--))
// -- case  https://www.thingiverse.com/search?q=TTGO+T5&type=things&sort=relevant
// -- board for attaching the sensors to make it stable

// ERROR:
//
// 

/*  
 ********************* ONLY LOCAL VERSION.   No PubSub, No BMx ********************************************************************   
*/


//Includes
#include <HTTPClient.h>
#include <WiFiCred_multi.h>                              // see dummy-file or below  (home, Fablab, Bitwäscherei
                                                         //    #define ssid_H          "SSID_H"
                                                         //    #define password_H      "Hxyz"
#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>              // 
#include "Zanshin_BME680.h"                              // Include the BME680 Sensor library
#include <ThingSpeak.h>                                  // 
#include <ThingspeakCred.h>                              // write Key : writeAPIKey
#include <Adafruit_NeoPixel.h>                           // 


//Defines
#define uS_TO_S_FACTOR     1000000                       // Conversion factor for micro seconds to seconds
#define TIME_TO_BOOT            10                       // on error (no wifi, no sensors) reboot to probably fix it
#define LoopUpdateDelay      60000                       // 1 minute
#define LoopMultiplier           1                       // 
#define Neopi_pin               18                       // 
#define NUMPIXELS                1                       // Popular NeoPixel ring size
#define NPixDelay               10                       // 10 msec delay between NPixel
#define ANALOGPIN              A13                       // Feather specific: read Bat-Level via resistor divider !!!!!! CHECK VALUES
#define MINVOLT             1830.0                       // 3.3*4096/6.6
#define MAXVOLT             2330.0                       // 4.2*4096/6.6

//Global variables
unsigned long previousMillis  =      0;                  // 
unsigned int  TPupdateCnt     =      0;
unsigned int  BRIGHTNESS      =     20;                  // 50 is very bright

// Global Sensor Data
int   scd30_co2_val;                                     // CO2 from Sensirion SCD30 CO2-Sensor
float scd30_temp_val;                                    // temperature  "
float scd30_humi_val;                                    // humidity     "
float batt_val;                                          // battery
char  scd30_co2Char[9];                                  // 1457ppm
char  scd30_tempChar[8];                                 // 99.9°C
char  scd30_humiChar[8];                                 // 99.9%RF
char  batteryChar[5];                                    // 
static int32_t  temp, humidity, pressure, gas;           // Variable to store readings
static int32_t bme680_temp_val;                          // returned without dezimal sign!
static int32_t bme680_humi_val;                          //    "
static int32_t bme680_press_val;                         //    "
static int32_t bme680_airequi_val;                       //    "
float bme680_temp;                                       // convert into data of the correct size
float bme680_humi;                                       //    "
int   bme680_pres;                                       //    "
int   bme680_airq;                                       //    "



//Define services
BME680_Class BME680;                              // adress 0x76
SCD30 airSensor;                                            // address 0x61
WiFiClient espClient;                                       // 
Adafruit_NeoPixel pixels(NUMPIXELS, Neopi_pin, NEO_GRB + NEO_KHZ800);


// enable debug statements to Serial Monitor if uncommented
//   #define DEBUG1                               // comment to ...
//   #define DEBUG2                               // comment for disabling timerDelayDebug
#define my_DEBUG                             // un-/comment that statement in your program to enable/disable debugging output
// Debugging included 
#ifdef my_DEBUG
#define DEBUG_WRT(...) { Serial.write(__VA_ARGS__); }
#define DEBUG_PRT(...) { Serial.print(__VA_ARGS__); }
#define DEBUG_PLN(...) { Serial.println(__VA_ARGS__); }
#define DEBUG_BEGIN()  { Serial.begin(57600); }
#else
#define DEBUG_WRT(...) {}
#define DEBUG_PRT(...) {}
#define DEBUG_PLN(...) {}
#define DEBUG_BEGIN()  {}
#endif


void blink_RED(int count)                                    // expand for other external LEDs
{
  for (int i = 0; i != count; i++) {
    digitalWrite(BUILTIN_LED, HIGH);    delay(50);
    digitalWrite(BUILTIN_LED, LOW);     delay(50);
   }
}

void reboot()
{
   esp_sleep_enable_timer_wakeup(TIME_TO_BOOT * uS_TO_S_FACTOR);    // wakeup via built-in timer sleep for xx seconds, then reboot after wakeup
   esp_deep_sleep_start();                                          // do so
}

void setup_wifi() {
  IPAddress wrongIP(0,0,0,0);
  int maxTryCnt = 0;
  int maxTry    = 10;
  
  DEBUG_PRT("\nConnecting to: ");
  DEBUG_PLN(ssid_H);
  maxTryCnt = 0; 
  WiFi.begin(ssid_H, password_H);

   while (WiFi.status() != WL_CONNECTED) {
    maxTryCnt++;
    delay(500);
    DEBUG_PRT(".");
    blink_RED(2);
    if (maxTry == maxTryCnt) {
      DEBUG_PLN(" WiFi connect tries for home reached. reboot");
      reboot();
    }
   }
    IPAddress localIP = WiFi.localIP();
    DEBUG_PLN();
    DEBUG_PRT("WiFi connected. IP: ");
    DEBUG_PLN(localIP);
    if (localIP == wrongIP)
      reboot();
}

void updateThingSpeak() {                                        // please refer to the documentation on ThingSpeak
    DEBUG_PRT("calling ThingSpeak  Channel-ID: ");               // temp, hum, pressure, CO2, batt
    DEBUG_PLN(ChannelID);
    ThingSpeak.setField(1, scd30_temp_val);
    ThingSpeak.setField(2, scd30_humi_val);
    ThingSpeak.setField(3, bme680_pres);
    ThingSpeak.setField(4, bme680_airq);
    ThingSpeak.setField(5, scd30_co2_val);
    ThingSpeak.setField(6, batt_val);
 
    // write to the ThingSpeak channel
     int x = ThingSpeak.writeFields(ChannelID, writeAPIKey);     // taken from credential file
     if(x == 200) {
      DEBUG_PLN("Channel update successful.");
      DEBUG_PLN();
     } else {
      DEBUG_PLN("Problem updating channel. HTTP error code " + String(x));
      DEBUG_PLN();
     }
}

void readSensor() {                                              // insert here all sensor communication and make their results public
  int maxRetry = 5;                                              // as many times each sensor could be read until ZERO will be filled in each value
  int retryCnt = 0;

  DEBUG_PLN("Sensor data reading ...");
  batt_val = BatteryLevel();                                     // get Battery-Level from Analog
  DEBUG_PRT("Battery-Level: ");
  DEBUG_PLN(batt_val);
  
  if ((airSensor.dataAvailable()) && (retryCnt != maxRetry)) {       // 
    scd30_co2_val  = airSensor.getCO2();                             // read Sensor
    scd30_temp_val = airSensor.getTemperature();                     // 
    scd30_humi_val = airSensor.getHumidity();                        // 
   } else {
    retryCnt++; 
    scd30_co2_val  = 0;                                              // Sensor not available show ZERO as error status
    scd30_temp_val = 0.0;                                            // 
    scd30_humi_val = 0.0;
    DEBUG_PRT(" SCD30 retry: "); 
    DEBUG_PLN(retryCnt);
    delay(2000);
   }
   retryCnt = 0;
   DEBUG_PRT("SCD30 CO2:  ");   DEBUG_PLN(scd30_co2_val);
   DEBUG_PRT("SCD30 Temp: ");   DEBUG_PLN(scd30_temp_val);
   DEBUG_PRT("SCD30 Humi: ");   DEBUG_PLN(scd30_humi_val);
   DEBUG_PLN();

   if ((BME680.getSensorData(bme680_temp_val,bme680_humi_val,bme680_press_val,bme680_airequi_val)) 
       && (retryCnt != maxRetry))	   {                             // Get the most recent readings
    bme680_temp = bme680_temp_val/100.0;
    bme680_humi = bme680_humi_val/1000.0;
    bme680_pres = bme680_press_val/100;
    bme680_airq = bme680_airequi_val/100;
   } else {
    retryCnt++; 
	bme680_temp = 0.0;
	bme680_humi = 0.0;
	bme680_pres = 0;
	bme680_airq = 0;
    DEBUG_PRT(" SCD30 retry: "); 
    DEBUG_PLN(retryCnt);
    delay(2000);
   }
    DEBUG_PRT("BME680 Temp: ");   DEBUG_PLN(bme680_temp,2);
    DEBUG_PRT("BME680 Humi: ");   DEBUG_PLN(bme680_humi,2);
    DEBUG_PRT("BME680 P:    ");   DEBUG_PLN(bme680_pres);
    DEBUG_PRT("BME680 AirQ: ");   DEBUG_PLN(bme680_airq);
    DEBUG_PLN();
}   

void transferData()                                                // add here all specific transfers
{
  if (TPupdateCnt == 5) {                                          // transfer to TP after 5 loops
    DEBUG_PLN("Transfering data to ...: ");                        //
    updateThingSpeak();                                            //
    DEBUG_PLN(" ... Transfer to ThinkSpeak finished");
    TPupdateCnt = 0;
  }
  TPupdateCnt++;
}


void led_ALL_off() {                                               // not nice with one Neopixel
  for(int i=0; i<NUMPIXELS; i++) {                                 // but for further improvement
     pixels.setPixelColor(i, pixels.Color(i, 254, 0));
     pixels.show();
     delay(NPixDelay);
  }   
     pixels.clear();                // does clear really work ???
     delay(100);
     DEBUG_PLN("all off");
}
void led_GREEN_on() {
  for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(0, 254, 0));
     pixels.show();
     delay(NPixDelay);
  }   
     DEBUG_PLN("GREEN on");
}
void led_YELLOW_on() {
  for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(254, 90, 0));
     pixels.show();
     delay(NPixDelay);
  }   
     DEBUG_PLN("YELLOW on");
}
void led_RED_on() {
  for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(254, 0, 0));
     pixels.show();
     delay(NPixDelay);
  }
     DEBUG_PLN("RED on");
}
void led_BLUE_on() {
  for(int i=0; i<NUMPIXELS; i++) {
     pixels.setPixelColor(i, pixels.Color(0, 0, 250));
     pixels.show();
     delay(NPixDelay);
  }
     DEBUG_PLN("BLUE on");
}

void action_on_values() {                                      // add here all local action based on sensor data
  int switch_val;
   
  led_ALL_off();

  if  (scd30_co2_val <700)   switch_val = 1;                   // with more Neopixel a mapping function would be useful
  if ((scd30_co2_val >701) && 
      (scd30_co2_val <1000)) switch_val = 2;
  if  (scd30_co2_val >1001)  switch_val = 3;
  switch (switch_val) {
    case 1:
      led_GREEN_on();
      break;
    case 2:
      led_YELLOW_on();
      break;
    case 3:
      led_RED_on();
      break;  
    default:
      break;
  }
}


float BatteryLevel()
{
  float batteryPercent;
  unsigned int batteryData = 0;

  for (int i = 0; i < 64; i++) {
    batteryData = batteryData + analogRead(ANALOGPIN);
  }
  batteryData = batteryData >> 6; //divide by 64
  batteryPercent = (float(batteryData) - MINVOLT) / (MAXVOLT - MINVOLT) * 100.0;
  if (batteryPercent < 0.0) {
    batteryPercent = 0.0;
  }
  if (batteryPercent > 100.0) {
    batteryPercent = 100.0;
  }
  return batteryPercent;
}

//----------------------------------------------------------------------------------------------------------------------------------------------
void setup()                                     // 
{
  DEBUG_BEGIN();
  pinMode(BUILTIN_LED, OUTPUT);                  // initialize all used GPIOs
  pixels.begin();                                // initialize NeoPixel object
  delay(100);                                    // 
  pixels.setBrightness(20);
  delay(100);
  led_ALL_off();
  led_BLUE_on();
  delay(100);

  setup_wifi();                                  // connect to the home network

  Wire.begin();    delay(100);
                                                 // initiate here all attached sensors
  while (!BME680.begin(I2C_STANDARD_MODE))                              // Start BME680 using I2C, use first device found
  {
    DEBUG_PLN("Unable to find BME680. Trying again in 5 seconds.");
    led_RED_on();
    delay(5000);
  }
  DEBUG_PLN("- Setting 16x oversampling for all sensors");
  BME680.setOversampling(TemperatureSensor,Oversample16);               // Use enumerated type values
  BME680.setOversampling(HumiditySensor,   Oversample16);               // Use enumerated type values
  BME680.setOversampling(PressureSensor,   Oversample16);               // Use enumerated type values
  DEBUG_PLN("Setting IIR filter to a value of 4 samples");
  BME680.setIIRFilter(IIR4);                                            // Use enumerated type values
  DEBUG_PLN("Setting gas measurement to 320\xC2\xB0\x43 for 150ms");    // "°C" symbols
  BME680.setGas(320,150);                                               // 320°C for 150 milliseconds

  DEBUG_PRT("BME680 I2C-Addresse: 0x"); 
  DEBUG_PRT(BME680.getI2CAddress(),HEX); DEBUG_PRT("h");
  DEBUG_PLN();
  
  if (airSensor.begin() == false) {              // 
    DEBUG_PLN("Sensor not available");
    delay(100);
    led_RED_on();                               // 
    while(1);                                   // REBOOT HERE TOO ????????????????????????????????????????????????????????????????????
  }
  airSensor.setAltitudeCompensation(430);        // Set altitude of the sensor in m

  ThingSpeak.begin(espClient);                       // Initialize ThingSpeak via WiFi

  DEBUG_PLN("Setup done");
  pixels.clear(); delay(100);
  
  readSensor();

}


void loop()                                            // 
{

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= (LoopUpdateDelay * LoopMultiplier)) {   
    previousMillis = currentMillis;                    // save the last time the inside loop was executed

    readSensor();                                      // get all data from the sensors and
    transferData();                                    // transfer them to the external devices (ThingSpeak, IFTTT, MQTT, ... )
    action_on_values();                                // 

  } 
}
//----------------------------------------------------------------------------------------------------------------------------------------------


/*
15:06:47.474 -> BLUE on
15:06:47.570 -> 
15:06:47.570 -> Connecting to: FabLab
15:06:48.088 -> ..... WiFi connect tries for FabLab reached. using home
15:06:51.095 -> 
15:06:51.095 -> Connecting to: UMAhome24
15:06:51.612 -> ....
15:06:53.904 -> WiFi connected. IP: 192.168.1.165
15:06:53.997 -> - Setting 16x oversampling for all sensors
15:06:54.279 -> Setting IIR filter to a value of 4 samples
15:06:54.279 -> Setting gas measurement to 320°C for 150ms
15:06:54.279 -> BME680 I2C-Addresse: 0x76h
15:06:54.324 -> Setup done
15:06:54.420 -> all off
15:06:56.543 -> Sensor data reading ...
15:06:56.543 -> Battery-Level: 100.00
15:06:56.543 -> SCD30 CO2:  789
15:06:56.543 -> SCD30 Temp: 30.32
15:06:56.543 -> SCD30 Humi: 33.83
15:06:56.543 -> 
15:06:56.543 -> BME680 Temp: 32.56
15:06:56.543 -> BME680 Humi: 28.50
15:06:56.543 -> BME680 P:    968
15:06:56.543 -> BME680 AirQ: 156
15:06:56.543 -> 

 */
