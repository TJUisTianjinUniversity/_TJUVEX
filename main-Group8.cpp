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
//底盘移动，四个轮子对应电机
void VRUN(double l, double r)
{
    vexMotorVoltageSet(PORT1, l * 120); //PORT为电机对应的端口
    vexMotorVoltageSet(PORT3, r * 120);
    vexMotorVoltageSet(PORT2, l * 120);    
    vexMotorVoltageSet(PORT4, r * 120);
}
void Autocontrol()
{
  chassis c1(0, 0, 0);
  c1.go_to(5, 5);
  double ini_angle=Motorarm.position(rev);
  double tar_angle=M_PI/4+ini_angle;
  control_arms(tar_angle,ini_angle,UP,&pid1);
}
void Usercontrol()
{
  while (true)
{
    /***以下为手动部分***/
    int fb, lf;
    /********************************************
    相应Axis对应(两个十字对应手柄左右两边摇杆)：
            Axis3
            =
    Axis4 =====
            =       Axis2
                    =
            Axis1 =====
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
    if (Controller1.ButtonR1.pressing())
        Motorarm.spin(forward, 50, pct);//up
    else if (Controller1.ButtonR2.pressing())
        Motorarm.spin(reverse, 50, pct);//down
    else
        Motorarm.stop(hold);

    //hand
    if (Controller1.ButtonL1.pressing())
        Motorhand.spin(forward, 50, pct);//open
    else if (Controller1.ButtonL2.pressing())
        Motorhand.spin(reverse, 50, pct);//close
    else
        Motorhand.stop(hold);


    task::sleep(8);//注意要sleep一小段时间防止过载
  }    

}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Autocontrol();
  Usercontrol();

}
