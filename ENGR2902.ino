#include <Arduino.h>

void setupIR();
uint16_t getIRDistance();

uint16_t adcResult = 0;
uint8_t reading = 0;
uint8_t spinLock = 0;
uint8_t duty = 0;
int trigPin = 13;                     //Trig
int echoPin = 3;    //Echo
int duration, cm, inches;
int LED_PIN=2;
int val;
bool IRnUS = 0; //1 to use IR 0 to use US
 

void pwm_PD3_write(uint8_t val){
  OCR2A = val;        //set value into timer register
  }


void setup() {
  setupIR();
  //TCCR2A = 0x81; //standard phase correct PWM on channel B
  //TCCR2B = 0x05; //phase correct pwm, prescaler = 1
  DDRB |= (1 << DDB3);

  TCCR2A |= (1<<WGM20)|(1<<WGM21)|(0<<COM2A0)|(1<<COM2A1);
  TCCR2B |=(1<<CS20)|(0<<CS21)|(0<<CS22);
  OCR2A = 0; //very high duty cycle
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
    
    delay(50);
    cm=1/(((((float)ADC/(float)1023)*2.29)-0.19)/21.84);

    if(cm>=40){duty=255; PORTB|=(1<<DDB5);}
		else if(cm<=15){duty=0; PORTB|=(1<<DDB5);}
		else if(cm<40&cm>15)
    {duty=(255+10*(cm-40)); PORTB&=~(1<<DDB5);}
		pwm_PD3_write(duty);
  // inches = (duration/2) / 74; 
    
    
      Serial.print(duration);
    Serial.print("duration, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    
    delay(5);









  } else {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(5);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH);
	

    /*if (duration < 65529) {
      OCR2A = (65530 - duration); 
    } else {
      OCR2A = 1;
    }*/
    cm =duration*0.034/2;
    if(cm>=40){duty=255; PORTB|=(1<<DDB5);}
		else if(cm<=15){duty=0; PORTB|=(1<<DDB5);}
		else if(cm<40&cm>15)
    {duty=(255+10*(cm-40)); PORTB&=~(1<<DDB5);}
		pwm_PD3_write(duty);
  // inches = (duration/2) / 74; 
    
    
      Serial.print(duration);
    Serial.print("duration, ");
    Serial.print(cm);
    Serial.print("cm");
    Serial.println();
    
    delay(5);
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
