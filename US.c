#include "US.h"

uint16_t initialReading = 0;
uint16_t finalReading = 0;
uint8_t reading = 0;
void INT0_isr(void) {
    uint8_t cntL = TCNT1L; //important to read low byte first because of temp register
    uint8_t cntH = TCNT1H;
    if(!reading) {
        initialReading = cntH << 8 | cntL;
        reading = 1;
    } else {
        finalReading = cntH << 8 | cntL;
        reading = 0;
    }
}

void setupUS() {//need to configure timer and pin interrupt on rising and falling edge
    TCCR1A = 0x00; //set Timer Control register A to 0, normal operation
    TCCR1C = 0x00; //no force output compare
    TCCR1B = 0x01; //no input capture, prescalar = 1, start timer
    //not touching interrupt config, as this module doesn't use it, but others may.
}

uint16_t getDistance() {
    uint8_t spinLock = 0;
    while(reading) {
        spinLock++;
    }
    if(finalReading > initialReading) {
        return finalReading - initialReading;
    } else {
        return (65535 - initialReading) + finalReading;
    }
}