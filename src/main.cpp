#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "MotionControl.h"
#include "IK.h"
#include "FK.h" 



struct Vec3 {
    float x, y, z;
};


Vec3 pointA = {225, 0, 0};
Vec3 pointB = {50, 0, 175};


Vec3 degA = {90,90,90};
Vec3 degB = {90,90,180};



int PWMmapper(int angle) {
  return map(angle, 0, 180, 120, 600);  //ServoMin&Max = PWM for 0 - 180 degrees
}

float theta1_deg, theta2_deg, theta3_deg;

void setup() {
    Serial.begin(115200);
    initMotion();
    pwm.begin();
    pwm.setPWMFreq(50);  // 50Hz for servos
    Serial.println("Hexapod control ready.");

}

void loop() {


  pwm.setPWM(13, 0, PWMmapper(degA.x));
  delay(50);
  pwm.setPWM(14, 0, PWMmapper(degA.y));
  delay(50);
  pwm.setPWM(15, 0, PWMmapper(degA.z));
  delay(1000);

  Serial.println(PWMmapper(degA.x));
  Serial.println(PWMmapper(degA.y));
  Serial.println(PWMmapper(degA.z));
  Serial.println("--------");

  

  pwm.setPWM(13, 0, PWMmapper(degB.x));
  delay(50);
  pwm.setPWM(14, 0, PWMmapper(degB.y));
  delay(50);
  pwm.setPWM(15, 0, PWMmapper(degB.z));
  delay(1000);

  Serial.println(PWMmapper(degB.x));
  Serial.println(PWMmapper(degB.y));
  Serial.println(PWMmapper(degB.z));
  Serial.println("--------");
  
  // IK(pointA.x, pointA.y, pointA.z, theta1_deg, theta2_deg, theta3_deg);
  // delay(10);  // Small delay to allow IK calculations to stabilize
  // moveToAngles(theta1_deg, theta2_deg, theta3_deg);
  // delay(10);



  // Serial.println("==== MOVE A DEBUG ====");
  // Serial.print("Theta[A]: ");
  // Serial.print(theta1_deg); Serial.print(", ");
  // Serial.print(theta2_deg); Serial.print(", ");
  // Serial.println(theta3_deg);

  // Serial.print("PWM:      ");
  // Serial.print(angleToPWM(theta1_deg)); Serial.print(", ");
  // Serial.print(angleToPWM(theta2_deg)); Serial.print(", ");
  // Serial.println(angleToPWM(theta3_deg));

  // Serial.print("X Y Z[A]: ");
  // Serial.print(pointA.x); Serial.print(" ");
  // Serial.print(pointA.y); Serial.print(" ");
  // Serial.println(pointA.z);

  // Serial.println();



  // delay(1000);  // Wait for servos to stabilize 



  // // delay(10);
  // // IK(pointB.x, pointB.y, pointB.z, theta1_deg, theta2_deg, theta3_deg);
  // // delay(10); // Small delay to allow IK calculations to stabilize
  // // moveToAngles(theta1_deg, theta2_deg, theta3_deg);  

  // Serial.println("==== MOVE B DEBUG ====");
  // Serial.print("Theta[B]: ");
  // Serial.print(theta1_deg); Serial.print(", ");
  // Serial.print(theta2_deg); Serial.print(", ");
  // Serial.println(theta3_deg);

  // Serial.print("PWM:      ");
  // Serial.print(angleToPWM(theta1_deg)); Serial.print(", ");
  // Serial.print(angleToPWM(theta2_deg)); Serial.print(", ");
  // Serial.println(angleToPWM(theta3_deg));
  
  // Serial.print("X Y Z[B]: ");
  // Serial.print(pointB.x); Serial.print(" ");
  // Serial.print(pointB.y); Serial.print(" ");
  // Serial.println(pointB.z);

  // Serial.println("====================");

  // delay(2500);  // Wait for servos to stabilize 
  // Serial.println("--------");

  
}

