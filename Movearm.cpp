#include "vex.h"
#include "cmath"


using namespace vex;
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

//控制机械臂
void up_arms(double speed)
{
  Motorarm.spin(forward,speed,pct);
}
void down_arms(double speed)
{
   Motorarm.spin(reverse,speed,pct);
}
//控制爪子开合
void open_hands(double speed)
{
  Motorhand.spin(forward,speed,pct);
}
void close_hands(double speed)
{
  Motorhand.spin(reverse,speed,pct);
}
//控制机械臂运动
//directon 转动方向
//angle_speed 角速度
/***************************************************************************
  函数名称：control_arms
  功    能：控制机械臂
  输入参数：目标角度、当前角度、角速度、方向
  返 回 值：无
  说    明：调用该函数执行功能
***************************************************************************/
void control_arms(double tar_angle,double pre_angle,int direction,DistanceController* pid)//后续增加其他判断条件
{
  double error, angle_speed;
  error= tar_angle-pre_angle;
  while(true)
  {
    angle_speed=PIDController_Update(pid, tar_angle, pre_angle);//名字要改
    if (error > 0.001)
    {
      if (direction == UP)
      {
        up_arms(angle_speed*80);
      }
      else if (direction == DOWN)
      {
        down_arms(angle_speed*80);
      }
    }
    else
    {
      break;
    }
  }
}
//控制机械臂停止
void hold_arms()
{
  Motorarm.stop(hold);
}
//控制爪子运动
/***************************************************************************
  函数名称：control_hands
  功    能：控制爪子
  输入参数：目标角度、当前角度、角速度、方向
  返 回 值：无
  说    明：调用该函数执行功能
***************************************************************************/
void control_hands(double tar_angle,double pre_angle,int direction,DistanceController* pid)
{
  double error,angle_speed;
  error= tar_angle-pre_angle;
  while(true)
  {  
      angle_speed=PIDController_Update(pid, tar_angle, pre_angle);//名字要改
      if(error>0.001)
    {
      if(direction==OPEN)
      {
        open_hands(angle_speed*50);
      }
      else if(direction==CLOSE)
      {
        close_hands(angle_speed*50);
      }
      else
      {
        break;
      }
    }
  }
}
//控制爪子停止
void hold_hands()
{
  Motorhand.stop(hold);
}