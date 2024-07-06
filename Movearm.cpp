#include "vex.h"
#include "cmath"


using namespace vex;
DistanceController spin_con;
extern double distance_cal;
// spin_con.limMin=0;
//初始化
void DistanceController_Init(DistanceController* pid) {

	pid->integrator = 0.0;
	pid->prevError = 0.0;//上一次误差
	pid->differentiator = 0.0;

}
double PIDController_Update(DistanceController* pid, double target_position, double current_position) {
  //误差标志
	double error = target_position - current_position;

	//比例误差
	double proportional = pid->Kp * error;

  //积分误差
	pid->integrator +=error;

	/* 限制积分项，防止饱和 */
	if (pid->integrator > pid->limMaxInt) {

		pid->integrator = pid->limMaxInt;
	}
	else if (pid->integrator < pid->limMinInt) {

		pid->integrator = pid->limMinInt;
	}


	//微分误差
	pid->differentiator = (error - pid->prevError)/dt;


	//判断是否达到预期结果


	pid->out = proportional + pid->Ki* pid->integrator + pid->Kd* pid->differentiator;

	if (pid->out > pid->limMax) {

		pid->out = pid->limMax;
	}
	else if (pid->out < pid->limMin) {

		pid->out = pid->limMin;
	}

  //更新误差
	pid->prevError = error;

	//返回输出值
  
	return pid->out;
}
double distance_hand()
{
  const double init_angle = atan(20 / 28);
  static double init_pos = Motorarm.position(rev);
  double now_pos = Motorarm.position(rev);
  double now_angle = (now_pos - init_pos) / 7 * 2 * M_PI;;
  distance_cal = sin(init_angle + now_angle) * 14 - 2.74;
  Brain.Screen.printAt(10,200, "distance_cal = %f", distance_cal);
  return 1;
}
//控制机械臂
void down_arms(double speed)
{
  Motorarm.spin(reverse,speed,pct);
}
void up_arms(double speed)
{
   Motorarm.spin(forward,speed,pct);
}
void hold_arms()
{
  Motorarm.stop(hold);
}
//控制爪子开合
void close_hands()
{
  Motorhand.spin(reverse,35,pct);
}
void open_hands()
{
  Motorhand.spin(forward,35,pct);
}
//控制爪子停止
void hold_hands()
{
  Motorhand.stop(hold);
}
//控制机械臂运动
//directon 转动方向
//angle_speed 角速度
/***************************************************************************
  函数名称：control_arms
  功    能：控制机械臂
  输入参数：改变角度、方向
  返 回 值：无
  说    明：调用该函数执行功能
***************************************************************************/
void control_arms(double change_angle,int direction,DistanceController* spin_con)//后续增加其他判断条件
{
  double pre_angle;
  double ini_angle=Motorarm.position(deg);
  double error, angle_speed;
  double tar_angle=change_angle*7+ini_angle;
  spin_con->limMaxInt=50;
  spin_con->limMax=40;
  DistanceController_Init(spin_con);
  while(true)
 {
  pre_angle=Motorarm.position(deg);
  Brain.Screen.printAt(10,40,"pre_angle=%f",pre_angle);
  error= tar_angle-pre_angle;
  // Brain.Screen.printAt(10,40,"error=%f",error);
  // Brain.Screen.printAt(10,80,"pre_angle=%f",pre_angle);
  if (fabs(error) > 0.05)
  {     
    angle_speed=PIDController_Update(spin_con, tar_angle, pre_angle);//名字要改
    if (direction == DOWN)
    {
      down_arms(angle_speed);
    }
    else if (direction == UP)
    {
      up_arms(angle_speed);
    } 
  }
  else
  {
    hold_arms();
    break;
  }
 }
}
//控制机械臂停止

//控制爪子运动
/***************************************************************************
  函数名称：control_hands
  功    能：控制爪子
  输入参数：目标角度、当前角度、角速度、方向
  返 回 值：无
  说    明：调用该函数执行功能
***************************************************************************/
void control_hands(int direction,double ini_angle_max)
{ 
  double pre_angle;
  // double ini_angle=Motorarm.position(deg);
  while(true)
  {
    pre_angle=Motorhand.position(deg);
    if(fabs(pre_angle-ini_angle_max)<2.5)
    {
      hold_hands();
      break;
    }
    else
    {
      if(direction==OPEN)
          open_hands();
      else if(direction==CLOSE)
      {        
          close_hands();
          vexMotorVoltageSet(PORT16,20);
          break;
      }
    }  
  }
}