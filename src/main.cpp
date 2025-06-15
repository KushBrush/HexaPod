#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"
#include "IK.h"
#include "FK.h"
#include "Parse.h"




void setup() {

  Serial.begin(115200);
  // Wire.setClock(400000);  // Boost I2C speed to 400kHz (max supported)

  initMotion();
  Serial.println("Hexapod control ready.");

}

void loop() {
  
  supportPhase(posA, posB);
  delay(1000);  // Wait for a second before the next iteration
  supportPhase(posB, posA);
  delay(1000);

}