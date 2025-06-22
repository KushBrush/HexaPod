#pragma once
#include <Adafruit_PWMServoDriver.h>
#include "Globals.h"

extern Adafruit_PWMServoDriver pwm;

void initMotion();
void moveToAngles(float theta1, float theta2, float theta3);
int angleToPWM(float angle_deg);
void supportPhase(Vec3 posA, Vec3 posB);
void bezierPhase(Vec3 posA, Vec3 posB);