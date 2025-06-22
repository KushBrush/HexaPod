
#pragma once

extern const float L1;             // Coxa length
extern const float L2;             // Femur length
extern const float L3;            // Tibia length
extern const int tibiaOffsetDeg;   // Offset relative to CW femur in Z (top down)
extern const int liftHeight;

struct Vec3 {
    float x, y, z;
};

extern Vec3 posA;
extern Vec3 posB;

// // Adjust based on your servos:
// extern const int SERVO_MIN;   // ~500us
// extern const int SERVO_MAX;   // ~2500us

// extern const int servo1;  // Coxa
// extern const int servo2;  // Femur
// extern const int servo3;  // Tibia

// extern const int steps;         // Number of interpolation steps
// extern const int delayPerStep;  // Fixed time between points

