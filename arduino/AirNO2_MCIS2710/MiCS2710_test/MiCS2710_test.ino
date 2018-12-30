/*
 Arduino AirQuality Tracker - NO2 in PartsPerMillion (PPM)
 
 Data transmited to Xively account using Telefonica GSM/GPRS Shield.
 
Code from the following public domain has been modified and built upon: 
 http://arduino.cc/en/Tutorial/GSMExamplesXivelyClient
 
 */

//NO2 sensor variables
const int NO2=5; //NO2 sensor on analog 3
float ADCreading = 0; //variable to hold ADC reading
float Vout = 0; //output voltage
float Rs = 0; //Sensor Resistance
float ppbNO2 = 0; //ppb NO2

void setup()
{
  // initialize serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.println("Starting debugging");
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  */

}

void loop()
{  
  // read the  sensor:
  ADCreading = analogRead(NO2);   
  Vout = ADCreading/409.2;  //convert from ADC value back to voltage
  Rs = 22000/((5/Vout) - 1);   // find sensor resistance from Vout, using 5V input & 22kOhm load resistor
  ppbNO2 = (.000008*Rs - .0194)*1000;    //convert Rs to ppm concentration NO2 (equation derived from data found on http://airpi.es/sensors.php
  Serial.print("PPB NO2= ");
  Serial.println(ppbNO2);
  delay(1000);
  float sensorReading = ppbNO2;

}
