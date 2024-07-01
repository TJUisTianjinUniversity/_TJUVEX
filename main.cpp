/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\lizea                                            */
/*    Created:      Sun Jun 30 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Motorarm             motor_group   1, 2            
// Motorhand            motor         3               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

 typedef struct {

	/* 控制因子 */
	float Kp;
	float Ki;
	float Kd;

	/* 设置极限 */
	float limMin;
	float limMax;

	/* 积分上限 */
	float limMinInt;
	float limMaxInt;

	/* 各项变量 */
	float integrator;//积分
	float prevError;//比例
	float differentiator;//微分
	float prevMeasurement;//当前状态

  float out;


} PIDController;
const int FORWARD=1;
const int REVERSE=0;
const int UP=1;
const int DOWN=0;
const int OPEN=1;
const int CLOSE=0;
const double dt=0.001;

using namespace vex;
//初始化
void PIDController_Init(PIDController* pid) {

	pid->integrator = 0.0f;
	pid->prevError = 0.0f;//上一次误差
	pid->differentiator = 0.0f;

}

float PIDController_Update(PIDController* pid, float target_position, float current_position) {

  //误差标志
	float error = target_position - current_position;

	//比例误差
	float proportional = pid->Kp * error;

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
	pid->out = proportional +pid->Ki* pid->integrator + pid->Kd* pid->differentiator;

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



//directon 转动方向
//angle_speed 角速度
//motorname 电机名字

void run(motor* motorname, int direction, float speed)
{
    if (direction == FORWARD) {
        motorname->spin(directionType::fwd, speed, velocityUnits::pct);
    } else if (direction == REVERSE) {
        motorname->spin(directionType::rev, speed, velocityUnits::pct);
    }
}
void move_arms(double pre_angle,double tar_angle,double speed)
{
  while(true)
  {
    if(pre_angle<tar_angle)
    {
      Motorarm.spin(forward,speed,pct);
    }
    else if(pre_angle>tar_angle)
    {
       Motorarm.spin(reverse,speed,pct);
    }
    else
    {
      Motorarm.stop(hold);
      break;
    }
  }
}
void control_hands(double pre_angle,double tar_angle,double speed)
{
  while(true)
  {
    if(pre_angle<=tar_angle)
    {
      Motorhand.spin(forward,speed,pct);
    }
    else if(pre_angle>tar_angle)
    {
       Motorhand.spin(reverse,speed,pct);
    }
    else
    {
      Motorhand.stop(hold);
      break;
    }
  }
}





  

}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
