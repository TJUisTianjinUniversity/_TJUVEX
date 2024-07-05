#include "vex.h"
#include"cmath"

using namespace vex;
const double angle_max=123.57;
const double angle_min=1.0;
const double angle_max_hand=60;
//底盘移动，四个轮子对应电机
void VRUN(double l, double r)
{
    vexMotorVoltageSet(PORT19, l * 120); //PORT为电机对应的端口
    vexMotorVoltageSet(PORT11, r * 120);
    vexMotorVoltageSet(PORT20, l * 120);    
    vexMotorVoltageSet(PORT17, r * 120);
}
void Usercontrol()
{
  double ini_angle_control2=Motorarm.position(deg);//init_arm
  double ini_angle_control=Motorhand.position(deg);//init_hand

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
    double pre_angle_control2=Motorarm.position(deg);
    // Brain.Screen.printAt(10,40, "init_angle = %f", ini_angle_control2);
    // Brain.Screen.printAt(10,80, "pre_angle = %f", pre_angle_control2);
    if(Controller1.ButtonR1.pressing()&&fabs(pre_angle_control2-ini_angle_control2)<angle_max*7)
    {    
       Motorarm.spin(forward,50,pct);//up
    }
    else if(Controller1.ButtonR2.pressing()&&fabs(pre_angle_control2-ini_angle_control2)>angle_min*7)
    {
       Motorarm.spin(reverse,50,pct);//down
    }
    else   
      Motorarm.stop(hold);

    double pre_angle_control=Motorhand.position(deg);
    //hand
    Brain.Screen.printAt(10,80, "pre_angle = %f", pre_angle_control);
    if(Controller1.ButtonL2.pressing()&&fabs(pre_angle_control-ini_angle_control)<angle_max_hand)
    {
      if(pre_angle_control-ini_angle_control<50.0)
          Motorhand.spin(forward,30,pct);//open
    }
    else if(Controller1.ButtonL1.pressing())
    {
      if(pre_angle_control-ini_angle_control>5.0)
          Motorhand.spin(reverse,20,pct);//close
    }
    else 
      Motorhand.stop(hold);

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