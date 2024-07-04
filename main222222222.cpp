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
// Motorhand            motor         7               
// Motorarm             motor_group   5, 6            
// LeftMotorGroup       motor_group   1, 2            
// RightMotorGroup      motor_group   3, 4            
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


void usercontrol()
{
  double ini_angle_control2=Motorarm.position(deg);//arm
  double ini_angle_control=Motorhand.position(deg);//hand

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
    
    if(Controller1.ButtonR2.pressing()||Controller1.ButtonR1.pressing())
    {
      int r2=0,r1=0;
      if(Controller1.ButtonR2.pressing())
      r2=1;
      else if(Controller1.ButtonR1.pressing())
      r1=1;

      double pre_angle_control2=Motorarm.position(deg);

      if(pre_angle_control2-ini_angle_control2>0.5){
        for(;pre_angle_control2-ini_angle_control2<130.0;pre_angle_control2=Motorarm.position(deg))
        {

          if(r2==1)
            Motorarm.spin(forward,50,pct);//up
          else if(r1==1)
            Motorarm.spin(reverse,50,pct);//down
          
        }   
      }   
      Motorarm.stop(hold);  
    }

    //hand
    
    if(Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing())
    {
      int l2=0,l1=0;
      if(Controller1.ButtonL2.pressing())
      l2=1;
      else if(Controller1.ButtonL1.pressing())
      l1=1;

      double pre_angle_control=Motorhand.position(deg);
      if(pre_angle_control-ini_angle_control>0.5){
        for(;pre_angle_control-ini_angle_control<60.0;pre_angle_control=Motorhand.position(deg))
        {

          if (l1==1)
              Motorhand.spin(forward, 50, pct);//open
          else if (l2==1)
              Motorhand.spin(reverse, 50, pct);//close 
        } 
      }

      Motorhand.stop(hold);  
    }
  
  //微调位置
  if(Controller1.ButtonUp.pressing()){
    LeftMotorGroup.spin(forward,10,pct);
    RightMotorGroup.spin(forward,10,pct);
  }
  else if(Controller1.ButtonDown.pressing()){
    LeftMotorGroup.spin(reverse,10,pct);
    RightMotorGroup.spin(reverse,10,pct);
  }
  else if(Controller1.ButtonLeft.pressing()){
    LeftMotorGroup.spin(reverse,10,pct);
    RightMotorGroup.spin(reverse,10,pct);
  }
  else if(Controller1.ButtonRight.pressing()){
     LeftMotorGroup.spin(forward,10,pct);
    RightMotorGroup.spin(forward,10,pct);
  }

    task::sleep(8);//注意要sleep一小段时间防止过载
    }
}
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  
}
