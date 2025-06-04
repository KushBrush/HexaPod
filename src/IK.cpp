#include "IK.h"     
//#include "Globals.h"
#include <math.h>
#include <Arduino.h>


//////////////////////////////////////////////////////////////
//Inverse kinematics accepting x y z and outputting theta[3]
//////////////////////////////////////////////////////////////


const float L1 = 50.0f;
const float L2 = 75.0f;
const float L3 = 100.0f;
const int tibiaOffsetDeg = 40;


bool IK(float x, float y, float z, float &theta1_deg, float &theta2_deg, float &theta3_deg) {


  const float COXA_LENGTH = 50.0;
  const float FEMUR_LENGTH = 75.0;
  const float TIBIA_LENGTH = 100.0;

  // === Step 1.x: Coxa rotation in XY plane ===

  // 1.1: Compute theta1 in XY plane (rotation about Z axis)
  float theta1_rad = atan2(y, x); // CCW from X+ to Y+
  theta1_deg = (theta1_rad * 180.0 / PI) + 90;

  // 1.2: Rotate (x, y) into femur's local frame (along X axis)
  float cos1 = cos(theta1_rad);
  float sin1 = sin(theta1_rad);
  float x_local = cos1 * x + sin1 * y;
  float z_local = z;

  // 1.3: Subtract coxa length to get femur base position
  float x_femur = x_local - COXA_LENGTH;

  // === Step 2.x: Femur angle in its local XZ plane ===

  // 2.1: Compute straight-line distance from femur base to target
  float dist = sqrt(x_femur * x_femur + z_local * z_local);
  if (dist > (FEMUR_LENGTH + TIBIA_LENGTH)) return false; // Target is unreachable

  // 2.2: Compute base angle from femur to target in XZ plane
  float angleToTarget = atan2(z_local, x_femur);

  // 2.3: Apply law of cosines to get internal triangle angle at femur
  float angleFemurInner = acos(
    (FEMUR_LENGTH * FEMUR_LENGTH + dist * dist - TIBIA_LENGTH * TIBIA_LENGTH) /
    (2 * FEMUR_LENGTH * dist)
  );

  // 2.4: Combine to get femur's full rotation angle
  float theta2_rad = angleToTarget + angleFemurInner;

  // 2.5: Convert to degrees
  theta2_deg = (theta2_rad * 180.0 / PI) + 90;


  // === Step 3.x: Tibia angle at the knee joint ===

  // 3.1: Use law of cosines to compute angle at the knee
  float angleKneeInner = acos(
    (FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - dist * dist) /
    (2 * FEMUR_LENGTH * TIBIA_LENGTH)
  );

  // 3.2: Convert to servo-friendly tibia angle (180 = fully folded)
  float theta3_rad = PI - angleKneeInner;
  theta3_deg = (theta3_rad * 180.0 / PI) + 90 - tibiaOffsetDeg;



              theta1_deg = constrain(theta1_deg, 0, 180);
              theta2_deg = constrain(theta2_deg, 0, 180);
              theta3_deg = constrain(theta3_deg, 0, 180);


  return true;}