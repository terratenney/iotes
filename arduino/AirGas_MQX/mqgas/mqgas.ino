
// select the input pin for the MQ-X (135,8,2) sensors
int mq135Pin = A6; 
int mq9Pin = A5;
int mq2Pin = A4;

// variable to store the value coming from the sensor.
int val = 0; 
 
void setup() {
  Serial.begin(9600);
}
void loop() {
  // read the value from the sensor
  val = analogRead(mq135Pin);
  Serial.println (val);
  delay(1000);

  // read the value from the sensor
  val = analogRead(mq9Pin);
  Serial.println (val);
  delay(1000);
   // read the value from the sensor
  val = analogRead(mq2Pin);
  Serial.println (val);
  delay(1000);


  
}
