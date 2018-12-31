
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
void setup() { 
  Serial.begin(9600);
  pinMode(sensorPin, INPUT); 
} 

void loop() { 
  val = analogRead(sensorPin); 
  Serial.print ("raw = "); 
  Serial.println (val); 
  float zero = gasSensor.getRZero(); 
  Serial.print ("rzero: "); 
  Serial.println (zero); 
  float ppm = gasSensor.getPPM(); 
  Serial.print ("ppm: "); 
  Serial.println (ppm); 
  delay(5000); 
} 
