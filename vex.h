/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
   for (int iterator = 0; iterator < iterations; iterator++)
const int UP=1;
const int DOWN=0;
const int OPEN=1;
const int CLOSE=0;
const double dt=0.001;
struct DistanceController{
	/* 控制因子 */
	double Kp=2;
	double Ki=0;
	double Kd=0;

	/* 设置极限 */
	double limMin;
	double limMax;

	/* 积分上限 */
	double limMinInt;
	double limMaxInt;

	/* 各项变量 */
	double integrator;//积分
	double prevError;//比例
	double differentiator;//微分
	double prevMeasurement;//当前状态

  double out;


};
void control_arms(double tar_angle,double pre_angle,int direction,DistanceController* spin_con);//控制机械臂
struct coordinate
{
    /* data */
    double x;
    double y;
};

class chassis{

private:
  coordinate co;
  double toward;
  void run(double l, double r);
  void turn(double angle);
  void move(double distance);
  void record();
  
public:
  chassis(double x, double y, double toward);
  void go_to(double x, double y, int R = 0) ;
  void position(double &x, double &y, double &toward);
};

void Usercontrol();
void VRUN(double l, double r);