#include "vex.h"
#include"cmath"

using namespace vex;
//底盘移动，四个轮子对应电机
void VRUN(double l, double r)
{
    vexMotorVoltageSet(PORT1, l * 120); //PORT为电机对应的端口
    vexMotorVoltageSet(PORT3, r * 120);
    vexMotorVoltageSet(PORT2, l * 120);    
    vexMotorVoltageSet(PORT4, r * 120);
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
    double ini_angle_control=Motorhand.position(deg);
    if(Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing())
    {
      double pre_angle_control=Motorhand.position(deg);
      if(pre_angle_control-ini_angle_control<90.0)
      {
        if (Controller1.ButtonL1.pressing())
            Motorhand.spin(forward, 50, pct);//open
        else if (Controller1.ButtonL2.pressing())
            Motorhand.spin(reverse, 50, pct);//close
        else
            Motorhand.stop(hold);
      }
      else
        Motorhand.stop(hold);  
    }
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