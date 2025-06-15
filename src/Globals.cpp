
#include "globals.h"

const float L1 = 50;             // Coxa length
const float L2 = 75;             // Femur length
const float L3 = 100;            // Tibia length
const int tibiaOffsetDeg = 40;   // Offset relative to CW femur in Z (top down)
const float PI = 3.141592;       // Pi constant


// Adjust based on your servos:
const int SERVO_MIN = 120;   // ~500us
const int SERVO_MAX = 500;   // ~2500us

const int servo1 = 13;  // Coxa
const int servo2 = 14;  // Femur
const int servo3 = 15;  // Tibia

const int steps = 25;         // Number of interpolation steps
const int delayPerStep = 10;  // Fixed time between points

Vec3 posA = {125, -125, -50};
Vec3 posB = {125,  125, -50};