/* ENGR290 Technical Assignement 1
 * Samuel Besse 40088455
 * 
 * 
 * 
 */
#include "US.h"

uint16_t distanceCnt = 0;

void setup() {
  // put your setup code here, to run once:
  setupUS();
}

void loop() {
  // put your main code here, to run repeatedly:
  distanceCnt = getDistance();
  delay(80); //datasheet recommends at least 80ms between reads
}
