#include "FK.h"
#include <math.h>
//#include "Globals.h"


// Function to calculate the forward kinematics of a hexapod leg
// given the angles of the joints in degrees and return the end effector position

const float L1 = 50.0f;
const float L2 = 75.0f;
const float L3 = 100.0f;

void FK(float theta1_deg, float theta2_deg, float theta3_deg,
        float &x3, float &y3, float &z3)
{
    float theta1 = theta1_deg * M_PI / 180.0;
    float theta2 = theta2_deg * M_PI / 180.0;
    float theta3 = theta3_deg * M_PI / 180.0;

    float x1 = L1 * cos(theta1 - M_PI/2);
    float y1 = L1 * sin(theta1 - M_PI/2);
    float z1 = 0;

    float local_x_x = cos(theta1 - M_PI/2);
    float local_x_y = sin(theta1 - M_PI/2);
    float local_x_z = 0;

    float local_y_x = -sin(theta1 - M_PI/2);
    float local_y_y = cos(theta1 - M_PI/2);
    float local_y_z = 0;

    float local_z_x = 0;
    float local_z_y = 0;
    float local_z_z = 1;

    float femur_local_x = L2 * sin(theta2);
    float femur_local_y = 0;
    float femur_local_z = -L2 * cos(theta2);

    float femur_global_x = femur_local_x * local_x_x + femur_local_y * local_y_x + femur_local_z * local_z_x;
    float femur_global_y = femur_local_x * local_x_y + femur_local_y * local_y_y + femur_local_z * local_z_y;
    float femur_global_z = femur_local_x * local_x_z + femur_local_y * local_y_z + femur_local_z * local_z_z;

    float x2 = x1 + femur_global_x;
    float y2 = y1 + femur_global_y;
    float z2 = z1 + femur_global_z;

    float femur_length = sqrt(femur_global_x * femur_global_x +
                              femur_global_y * femur_global_y +
                              femur_global_z * femur_global_z);
    float tibia_local_x_x = femur_global_x / femur_length;
    float tibia_local_x_y = femur_global_y / femur_length;
    float tibia_local_x_z = femur_global_z / femur_length;

    float tibia_local_y_x = local_y_x;
    float tibia_local_y_y = local_y_y;
    float tibia_local_y_z = local_y_z;

    float tibia_local_z_x = tibia_local_x_y * tibia_local_y_z - tibia_local_x_z * tibia_local_y_y;
    float tibia_local_z_y = tibia_local_x_z * tibia_local_y_x - tibia_local_x_x * tibia_local_y_z;
    float tibia_local_z_z = tibia_local_x_x * tibia_local_y_y - tibia_local_x_y * tibia_local_y_x;

    float tibia_local_x = L3 * sin(theta3);
    float tibia_local_y = 0;
    float tibia_local_z = L3 * cos(theta3);

    float tibia_global_x = tibia_local_x * tibia_local_x_x + tibia_local_y * tibia_local_y_x + tibia_local_z * tibia_local_z_x;
    float tibia_global_y = tibia_local_x * tibia_local_x_y + tibia_local_y * tibia_local_y_y + tibia_local_z * tibia_local_z_y;
    float tibia_global_z = tibia_local_x * tibia_local_x_z + tibia_local_y * tibia_local_y_z + tibia_local_z * tibia_local_z_z;

    x3 = x2 + tibia_global_x;
    y3 = y2 + tibia_global_y;
    z3 = z2 + tibia_global_z;
}
