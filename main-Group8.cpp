/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lizea                                            */
/*    Created:      Wed Jul 03 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotorGroup       motor_group   1, 2            
// RightMotorGroup      motor_group   3, 4            
// Motorarm             motor_group   5, 6            
// Motorhand            motor         11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include"cmath"

using namespace vex;
DistanceController pid1;//定义的pid指针

void Autocontrol()
{
  chassis c1(0, 0, 0);
  c1.go_to(5, 5);
  double pre_angle,error;
  double ini_angle=Motorarm.position(deg);
  double tar_angle=20.0+ini_angle;
  while(true)
  {
    pre_angle=Motorarm.position(deg);
    error=tar_angle-pre_angle;
    if (fabs(error) > 5.0)
      control_arms(tar_angle,pre_angle,UP,&pid1);
    else
    {
      Motorarm.stop(hold);
      break;
    }
  } 
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Autocontrol();
  // Usercontrol();

}
