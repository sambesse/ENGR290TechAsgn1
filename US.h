/*
Library for reading distance from the Ultra Sonic Distance sensor HC-SR04
*/
#ifndef US_H
#define US_H
#include <Arduino.h>

//function to setup the US
void setupUS();

//function to return the distance read by the sensor in terms of clock cycles 
uint16_t getDistance();

#endif