#include "MotionControl.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Adjust based on your servos:
const int SERVO_MIN = 120;   // ~500us
const int SERVO_MAX = 600;   // ~2500us

const int servo1 = 13;  // Coxa
const int servo2 = 14;  // Femur
const int servo3 = 15;  // Tibia

void initMotion() {
    pwm.begin();
    pwm.setPWMFreq(50);  // 50Hz for servos
}

int angleToPWM(float angle_deg) {
    // Map angle to servo pulse range
    float pulse = map(angle_deg, 0, 180, SERVO_MIN, SERVO_MAX);
    Serial.print("Pulse "); Serial.println(pulse);            //Prints PWM for theta[3]
    return static_cast<int>(pulse);
}

void moveToAngles(float theta1_deg, float theta2_deg, float theta3_deg) {


    pwm.setPWM(servo1, 0, angleToPWM(theta1_deg));
    delay(100);  // Small delay to allow servo to move
    pwm.setPWM(servo2, 0, angleToPWM(theta2_deg));
    delay(100);
    pwm.setPWM(servo3, 0, angleToPWM(theta3_deg));
    delay(100);

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
