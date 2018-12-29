// The load resistance on the board https://hackaday.io/project/3475-sniffing-trinket/log/12363-mq135-arduino-library https://blog.robberg.net/mq-135-arduino/ 
#define RLOAD 22.0
// Calibration resistance at atmospheric CO2 level
#define RZERO 879.13 
#include "MQ135.h" 
MQ135 gasSensor = MQ135(A6); 
int val; 
int sensorPin = A6; 
int sensorValue = 0; 
void setup() { 
  Serial.begin(9600);
  pinMode(sensorPin, INPUT); 
} 

void loop() { 
  val = analogRead(A6); 
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
