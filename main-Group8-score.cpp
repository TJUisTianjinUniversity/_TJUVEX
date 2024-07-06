/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lizea                                            */
/*    Created:      Wed Jul 03 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include"cmath"

using namespace vex;
// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here
DistanceController pid1;//定义的pid指针
double distance_cal;
const double angle_one = 54.714;
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotorGroup       motor_group   19, 20            
// RightMotorGroup      motor_group   11, 17            
// Motorarm             motor_group   18, 13            
// Motorhand            motor         16              
// ---- END VEXCODE CONFIGURED DEVICES ----

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
void pre_auton(void) {

    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
    // ..........................................................................
    // Insert autonomous user code here.
    // ..........................................................................

    
    const double div = 23;
    static double ini_angle_max = Motorhand.position(deg);

    distance_hand();
    Motorhand.spin(reverse,35,pct);
    task::sleep(500); 

    control_arms(angle_one,UP,&pid1);
    Brain.Screen.printAt(10,80,"66666666");
    distance_hand();
    chassis c1(3.5 * div, 0.5 * div, M_PI);
    c1.go_to(0.5 * div, 0.5 * div);

    task::sleep(500);
    control_arms(70.0,UP,&pid1);   
    control_hands(OPEN,ini_angle_max);
    task::sleep(500);   

    c1.go_to(2 * div, 0.5 * div, 1);
    Motorhand.spin(reverse,35,pct);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
const double angle_max = 123.57;
const double angle_min = 1.0;
void VRUN(double l, double r)
{
    vexMotorVoltageSet(PORT19, l * 120); //PORT为电机对应的端口
    vexMotorVoltageSet(PORT1, r * 120);
    vexMotorVoltageSet(PORT18, l * 120);
    vexMotorVoltageSet(PORT14, r * 120);
}
void usercontrol(void) {
  // User control code here, inside the loop
  double ini_angle_control2 = Motorarm.position(deg);//init_arm
  double ini_angle_control = Motorhand.position(deg);//init_hand

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
     /***操纵摇杆控制底盘移动***/
        int fb, lf;
        /********************************************
        相应Axis对应(两个十字对应手柄左右两边摇杆)：
                Axis3
                =
        Axis4 =====
                =       Axis1
                        =
                Axis2 =====
                        =
        *********************************************/
        fb = Controller1.Axis3.value();
        lf = Controller1.Axis1.value();
        fb = abs(fb) > 15 ? fb : 0;
        lf = abs(lf) > 15 ? lf : 0;
        if (fb != 0 || lf != 0) VRUN((fb + lf) * 100.0 / 127.0, (fb - lf) * 100.0 / 127.0);
        else VRUN(0, 0);


        //按钮
        //arm
        double pre_angle_control2 = Motorarm.position(deg);
        // Brain.Screen.printAt(10,40, "init_angle = %f", ini_angle_control2);
        // Brain.Screen.printAt(10,80, "pre_angle = %f", pre_angle_control2);
        if (Controller1.ButtonR1.pressing() && fabs(pre_angle_control2 - ini_angle_control2) < angle_max * 7)
        {
            Motorarm.spin(forward, 50, pct);//up
        }
        else if (Controller1.ButtonR2.pressing() && fabs(pre_angle_control2 - ini_angle_control2) > angle_min * 7)
        {
            Motorarm.spin(reverse, 50, pct);//down
        }
        else
            Motorarm.stop(hold);

        double pre_angle_control = Motorhand.position(deg);
        //hand
        Brain.Screen.printAt(10, 80, "pre_angle = %f", pre_angle_control);
        if (Controller1.ButtonL2.pressing() && fabs(pre_angle_control - ini_angle_control)<50.0)
        {
           Motorhand.spin(forward, 30, pct);//open
        }
        else if (Controller1.ButtonL1.pressing())
        {
           Motorhand.spin(reverse, 30, pct);//close
        }
        else
            Motorhand.stop(hold);

        //微调位置
        if (Controller1.ButtonUp.pressing()) {
            LeftMotorGroup.spin(forward, 10, pct);
            RightMotorGroup.spin(forward, 10, pct);
        }
        else if (Controller1.ButtonDown.pressing()) {
            LeftMotorGroup.spin(reverse, 10, pct);
            RightMotorGroup.spin(reverse, 10, pct);
        }
        else if (Controller1.ButtonLeft.pressing()) {
            LeftMotorGroup.spin(reverse, 10, pct);
            RightMotorGroup.spin(reverse, 10, pct);
        }
        else if (Controller1.ButtonRight.pressing()) {
            LeftMotorGroup.spin(forward, 10, pct);
            RightMotorGroup.spin(forward, 10, pct);
        }

        task::sleep(8);//注意要sleep一小段时间防止过载
   }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
