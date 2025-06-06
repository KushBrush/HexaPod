#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"
#include "IK.h"
#include "FK.h"
#include "Parse.h"



struct Vec3 {
    float x, y, z;
};


Vec3 posA = {225, 0, 0};
Vec3 posB = {50, 0, 175};


Vec3 degA = {90,90,90};
Vec3 degB = {90,90,180};



// float theta1_deg, theta2_deg, theta3_deg;

void setup() {
    Serial.begin(115200);
    initMotion();
    pwm.begin();
    pwm.setPWMFreq(50);  // 50Hz for servos
    Serial.println("Hexapod control ready.");

}

void loop() {

  moveToAngles(degA.x, degA.y, degA.z);
  delay(2000);  // Wait for 2 seconds
  moveToAngles(degB.x, degB.y, degB.z);
  delay(2000);  // Wait for 2 seconds 
} 

