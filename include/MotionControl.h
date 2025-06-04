#pragma once
#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm;

void initMotion();
void moveToAngles(float theta1, float theta2, float theta3);
int angleToPWM(float angle_deg);