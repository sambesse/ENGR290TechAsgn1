#include <Arduino.h>

void setupIR();
uint16_t getIRDistance();

uint16_t adcResult = 0;
uint8_t reading = 0;
uint8_t spinLock = 0;

void setup() {
  setupIR();
  TCCR2A = 0x81; //standard phase correct PWM on channel B
  TCCR2B = 0x05; //phase correct pwm, prescaler = 1
  OCR2A = 0xe0; //very high duty cycle
  DDRD |= 1 << 3;
  ///PORTD |= 1 << 3;
  DDRB |= 1 << 5;
  PORTB |= 1 << 5;
  Serial.begin(9600);
  delay(50);
  //Serial.println("starting");
}

void loop() {
  Serial.println(getIRDistance());
  Serial.println(OCR2A);
  delay(500);
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
    Serial.println("spinning");
    delay(500);
  }
  OCR2A = ADC >> 2;
  return ADC;
}
