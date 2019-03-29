
#include <SoftwareSerial.h>
#include <PM2005Lib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2 

#define DHTTYPE           DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);
#include <MQ135.h>

// The load resistance on the board https://hackaday.io/project/3475-sniffing-trinket/log/12363-mq135-arduino-library https://blog.robberg.net/mq-135-arduino/ 
#define RLOAD 22.0
// Calibration resistance at atmospheric CO2 level
#define RZERO 879.13 
#include "MQ135.h" 
int sensorPin = A0; 

MQ135 gasSensor = MQ135(sensorPin); 
int val; 
int sensorValue = 0;
 
uint32_t delayMS;

float 2um = 0;
float 10um = 0;
float dh_temperature= 0;
float dh_humidity= 0;
//NO2 sensor variables

const int NO2=5; //NO2 sensor on analog 3
float ADCreading = 0; //variable to hold ADC reading
float Vout = 0; //output voltage
float Rs = 0; //Sensor Resistance
float ppbNO2 = 0; //ppb NO2

SoftwareSerial pm2005Serial(8, 9);
PM2005Lib pm2005(/*PM2005 device*/pm2005Serial, /*user message device*/Serial);

void setup() {
  // put your setup code here, to run once:

  // Initialize the USB serial port
  Serial.begin(115200);
  Serial.println("PM2005 TEST PROGRAM");
  Serial.println(PM2005LIB_VERSION);

  // Open the (software)serial port to the PM2005
  pm2005Serial.begin(PM2005_BAUD);

  // Initialize the PM2005 and start continous measurements
  pm2005.init();

////////////////////////////////////////////

    sht20.initSHT20();                                  // Init SHT20 Sensor
    delay(100);
    sht20.checkSHT20(); 
///////////////////////////////////////////
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  ////////////////////////
  pinMode(sensorPin, INPUT); 




  
}
void loop() {
///////////////////
  // Read out PM2005 measurement data and print current readings
  pm2005.measure();
  2um= pm2005.concentration_2um);
  10um= pm2005.concentration_10um > 500) {
///////////////////
    float humd = sht20.readHumidity();                  // Read Humidity
    float temp = sht20.readTemperature()
//////////////////////////////

 sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
  dh_temperature = dh_temperature;
}
  else {
    dh_temperature=event.temperature);
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    dh_humidity= dh_humidity;
  }
  else {
    dh_humidity= event.relative_humidity);
  }
///////////////////////////////////
  val = analogRead(sensorPin); 

  float zero = gasSensor.getRZero(); 
  float ppm = gasSensor.getPPM();
//////////////////////////////////////////
 ADCreading = analogRead(NO2);   
  Vout = ADCreading/409.2;  //convert from ADC value back to voltage
  Rs = 22000/((5/Vout) - 1);   // find sensor resistance from Vout, using 5V input & 22kOhm load resistor
  ppbNO2 = (.000008*Rs - .0194)*1000;    //convert Rs to ppm concentration NO2 (equation derived from data found on http://airpi.es/sensors.php
  
  float sensorReading = ppbNO2;


///////////////////////////////////////////

Serial.println(ppbNO2,ppm, dh_humidity, dh_temperature,humd,temp,10um,2um )
/////
  // Wait some time (e.g. 3 seconds) until next readout
  delay(1000);
}
