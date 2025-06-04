#include <Arduino.h>
#include "FK.h"
#include "IK.h"
#include <Globals.h>
#include "MotionControl.h"




void processSerialCommand();



void processSerialCommand() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.length() == 0) return;
  
  // Parse the command
  int firstSpace = command.indexOf(' ');
  if (firstSpace == -1) {
    Serial.println("Error: Invalid command format");
    return;
  }
  
  String cmd = command.substring(0, firstSpace);
  String params = command.substring(firstSpace + 1);
  
  // Parse parameters
  float param1, param2, param3;
  int count = sscanf(params.c_str(), "%f %f %f", &param1, &param2, &param3);
  
  if (count != 3) {
    Serial.println("Error: Need exactly 3 parameters");
    return;
  }
  
  if (cmd.equalsIgnoreCase("deg")) {
    // Forward kinematics: degrees to coordinates
    float x3, y3, z3;
    FK(param1, param2, param3, x3, y3, z3);
    

    Serial.print("Input angles: θ1=");
    Serial.print(param1, 2);
    Serial.print("° θ2=");
    Serial.print(param2, 2);
    Serial.print("° θ3=");
    Serial.print(param3, 2);
    Serial.println("°");
    
    Serial.print("Coordinates: x=");
    Serial.print(x3, 2);
    Serial.print(" y=");
    Serial.print(y3, 2);
    Serial.print(" z=");
    Serial.println(z3, 2);

    


    
  } else if (cmd.equalsIgnoreCase("pos")) {
    // Inverse kinematics: coordinates to degrees
    float theta1_deg, theta2_deg, theta3_deg;
    bool success = IK(param1, param2, param3, theta1_deg, theta2_deg, theta3_deg);
    
    Serial.print("Input coordinates: x=");
    Serial.print(param1, 2);
    Serial.print(" y=");
    Serial.print(param2, 2);
    Serial.print(" z=");
    Serial.println(param3, 2);
    
    if (success) {
      Serial.print("Angles: θ1=");
      Serial.print(theta1_deg, 2);
      Serial.print("° θ2=");
      Serial.print(theta2_deg, 2);
      Serial.print("° θ3=");
      Serial.print(theta3_deg, 2);
      Serial.println("°");
    } else {
      Serial.println("Error: Position not reachable or calculation failed");
    }
    
  } else {
    Serial.println("Error: Unknown command. Use 'deg' or 'pos'");
  }
  
  Serial.println(); // Empty line for readability
}