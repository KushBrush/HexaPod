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

  bezierPhase(posA, posB);
  delay(1000);
  supportPhase(posA, posB);  // swing back with lift
  delay(1000);
}
