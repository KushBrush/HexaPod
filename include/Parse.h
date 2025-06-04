
#pragma once

// Reads a line from the serial port and executes the command.
// Supported commands:
//  - "deg a b c" converts servo angles to coordinates using FK
//  - "pos x y z" converts coordinates to servo angles using IK
void processSerialCommand();