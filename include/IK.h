#pragma once

// Inverse Kinematics (IK) function declaration
// This function calculates the angles of the leg segments based on the target position (x3, y3, z3)
// and returns true if the position is reachable, false otherwise.  


bool IK(float x, float y, float z, float &theta1_deg, float &theta2_deg, float &theta3_deg);

// === Leg segment lengths in mm ===
//extern const float L1;                // coxaLength    
//extern const float L2;                // femurLength    
//extern const float L3;               // tibiaLength   
//extern const int tibiaOffsetDeg;      //Ofset relative to CW femur in z (top down)
