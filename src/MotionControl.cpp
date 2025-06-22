#include <Wire.h>
#include <Arduino.h>
#include "IK.h"
#include "Globals.h"
#include "MotionControl.h"

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Adjust based on your servos:
const int SERVO_MIN = 120;   // ~500us
const int SERVO_MAX = 500;   // ~2500us

const int servo1 = 13;  // Coxa
const int servo2 = 14;  // Femur
const int servo3 = 15;  // Tibia

const int steps = 25;         // Number of interpolation steps
const int delayPerStep = 10;  // Fixed time between points

extern const int liftHeight = 50; // Lift height for the leg

// Vec3 posA = {125, -125, -50};
// Vec3 posB = {125,  125, -50};




void initMotion() {
    pwm.begin();
    pwm.setPWMFreq(50);  // 50Hz for servos
    delay(10);          // Allow time for the PWM driver to initialize
}



int angleToPWM(float angle_deg) {

    angle_deg = constrain(angle_deg, 0, 180);  // Ensure angle is within bounds
    float pulse = map(angle_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    // Serial.print("Pulse "); Serial.println(pulse);            //Prints PWM for theta[3]
    return static_cast<int>(pulse);
}

void moveToAngles(float theta1_deg, float theta2_deg, float theta3_deg) {

    pwm.setPWM(servo1, 0, angleToPWM(theta1_deg));
    delay(3);  
    pwm.setPWM(servo2, 0, angleToPWM(theta2_deg));
    delay(3);
    pwm.setPWM(servo3, 0, angleToPWM(theta3_deg));
    delay(3);

    // Serial.print("MC-theta[3] = ");
    // Serial.print(theta1_deg);
    // Serial.print(", ");     
    // Serial.print(theta2_deg);
    // Serial.print(", ");
    // Serial.println(theta3_deg);
    // // Serial.print("PWM: ");
    // Serial.print(angleToPWM(theta1_deg));
    // Serial.print(", ");
    // Serial.print(angleToPWM(theta2_deg));
    // Serial.print(", ");
    // Serial.println(angleToPWM(theta3_deg));

}

void supportPhase(Vec3 posA, Vec3 posB){

    for (int i = 0; i <= steps; ++i) {
    float t = (float)i / steps;

    Vec3 interp;
    interp.x = posB.x + t * (posA.x - posB.x);
    interp.y = posB.y + t * (posA.y - posB.y);
    interp.z = posB.z + t * (posA.z - posB.z);

    float theta1, theta2, theta3;
    IK(interp.x, interp.y, interp.z, theta1, theta2, theta3);
    moveToAngles(theta1, theta2, theta3);

    delay(delayPerStep);
}
}

void bezierPhase(Vec3 posA, Vec3 posB) {
    Vec3 P0 = posA;
    Vec3 P2 = posB;

    // Midpoint
    Vec3 P1;
    P1.x = (P0.x + P2.x) / 2.0;
    P1.y = (P0.y + P2.y) / 2.0;
    P1.z = (P0.z + P2.z) / 2.0 + liftHeight;

    for (int i = 0; i <= steps; ++i) {
        float t = (float)i / steps;

        // Quadratic Bézier interpolation
        Vec3 interp;
        interp.x = (1 - t) * (1 - t) * P0.x + 2 * (1 - t) * t * P1.x + t * t * P2.x;
        interp.y = (1 - t) * (1 - t) * P0.y + 2 * (1 - t) * t * P1.y + t * t * P2.y;
        interp.z = (1 - t) * (1 - t) * P0.z + 2 * (1 - t) * t * P1.z + t * t * P2.z;

        float theta1, theta2, theta3;
        IK(interp.x, interp.y, interp.z, theta1, theta2, theta3);
        moveToAngles(theta1, theta2, theta3);

        delay(delayPerStep);
    }
}
