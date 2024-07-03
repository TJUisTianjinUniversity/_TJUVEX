/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\闫茂尧                                              */
/*    Created:      Sun Jun 09 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Motor0               motor         8               
// Motor1               motor         9               
// Motor2               motor         12              
// Motor3               motor         11              
// Motorhand            motor         3               
// Motorarm             motor_group   1, 2            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

//底盘移动，四个轮子对应电机
void VRUN(double l,double r)
{
vexMotorVoltageSet(PORT8,l*120); //PORT为电机对应的端口
vexMotorVoltageSet(PORT9,l*120);
vexMotorVoltageSet(PORT10,r*120);
vexMotorVoltageSet(PORT11,r*120);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  while(1)
    {
    /***操纵摇杆控制底盘移动***/
    int fb,lf;
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
    fb=Controller1.Axis3.value();
    lf=Controller1.Axis1.value();
    fb=abs(fb)>15?fb:0;
    lf=abs(lf)>15?lf:0;
    if(fb!=0||lf!=0) VRUN((fb+lf)*100.0/127.0,(fb-lf)*100.0/127.0);
    else VRUN(0,0);


    //按钮
    //arm
    if(Controller1.ButtonR2.pressing())
      Motorarm.spin(forward,50,pct);//up
    else if(Controller1.ButtonR1.pressing())
      Motorarm.spin(reverse,50,pct);//down
    else   
      Motorarm.stop(hold);


    //hand
    if(Controller1.ButtonL1.pressing())
      Motorhand.spin(forward,50,pct);//open
    else if(Controller1.ButtonL2.pressing())
      Motorhand.spin(reverse,50,pct);//close
    else 
      Motorhand.stop(hold);


    task::sleep(8);//注意要sleep一小段时间防止过载
    }
}
