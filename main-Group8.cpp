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
double distance_cal;
const double angle_one=54.714;
void Autocontrol()
{
  const double div = 23;
  static double ini_angle_max=Motorhand.position(deg);
  distance_hand();
  Motorhand.spin(reverse,35,pct);
  task::sleep(500);  
  control_arms(angle_one,UP,&pid1);
  distance_hand();
  chassis c1(3.5 * div, 0.5 * div, M_PI);
  c1.go_to(0.5 * div, 0.5 * div);


  task::sleep(500);   
  control_hands(OPEN,ini_angle_max);
  

  
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Autocontrol();
  // Usercontrol();

}
