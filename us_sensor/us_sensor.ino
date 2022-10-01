#include <Arduino.h>

int trigPin = 13;                     //Trig
int echoPin = 3;    //Echo
long duration, cm, inches;
int LED_PIN=2;
int val;
 int a=255/40;
 long ledLevel = 10;
void setup() {
  

 
  //Serial Port begin
  Serial.begin (115200);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
   //pinMode(ledPin, OUTPUT);
}
 
void loop()
{
 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

  cm = (duration/2) / 29;
// inches = (duration/2) / 74; 
  if(cm<40){
      Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
    while (Serial.available()) {
    ledLevel = Serial.parseInt();
    analogWrite(LED_PIN, ledLevel);
  }

 

  }
  delay(1000);

}
