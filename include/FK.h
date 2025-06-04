#pragma once

// Forward Kinematics (FK) function declaration
// This function calculates the end effector position (x3, y3, z3) based on the angles of the leg segments

void FK(float theta1_deg, float theta2_deg, float theta3_deg,
        float &x3, float &y3, float &z3);

// === Leg segment lengths in mm ===
//extern const float L1;                // L1    
//extern const float L2;                // L2    
//extern const float L3;               // L3   
//extern const int tibiaOffsetDeg;      //Ofset relative to CW femur in z (top down)
