#include <Arduino.h>

void setupIR();
uint16_t getIRDistance();

uint16_t adcResult = 0;
uint8_t reading = 0;
uint8_t spinLock = 0;
int trigPin = 13;                     //Trig
int echoPin = 3;    //Echo
long duration, cm, inches;
int LED_PIN=2;
int val;
int a=255/40;
long ledLevel = 10;
bool IRnUS = 1; //1 to use IR 0 to use US
 
void setup() {
  setupIR();
  TCCR2A = 0x81; //standard phase correct PWM on channel B
  TCCR2B = 0x05; //phase correct pwm, prescaler = 1
  OCR2A = 0xe0; //very high duty cycle
  DDRD |= 1 << 3;
  ///PORTD |= 1 << 3;
  DDRB |= 1 << 5;
  //PORTB |= 1 << 5;
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  delay(50);
  //Serial.println("starting");
}

void loop() {
  if (IRnUS) {
    Serial.println(getIRDistance());
    //Serial.println(OCR2A);
    OCR2A = ADC >> 2;
    if (ADC <= 448 || ADC >= 813) {
      PORTB |= 1 << 5;
    } else {
      PORTB &= ~(1 << 5);
    }
    delay(50);
  } else {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH);
    if (duration <= 875 || duration >= 2282) {
      PORTB |= 1 << 5;
    } else {
      PORTB &= ~(1 << 5);
    }
    if (duration < 65529) {
      OCR2A = (65530 - duration); 
    } else {
      OCR2A = 1;
    }
    cm = (duration/2) / 29;
  // inches = (duration/2) / 74; 
    Serial.println(duration);
    if(cm<40){
        Serial.print(inches);
    Serial.print("in, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    }
  }
}

void setupIR() {
  DDRC &= ~(1 << 3); //set ADC3 or PC3 to input
  ADMUX = 0x03; //using AREF as reference with ADC3 selected as input
  ADCSRA = 0x80; //enable ADC, with prescaler = 2
}

uint16_t getIRDistance() {
  ADCSRA |= 0x40; //start conversion
  while(ADCSRA & (1 << 6)) {//bit 6 is cleared when conversion is complete
    spinLock++;
    //Serial.println("spinning");
    //delay(500);
  }
  //OCR2A = ADC >> 2;
  return ADC;
}
