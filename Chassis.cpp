#include "vex.h"
#include <cmath>
using namespace vex;

const double r = 4 / 2;      //半径
const double propotion =  2 * M_PI * r; //  (m / rev)
const double max_v = 200.0 / 60 * propotion;     //(m / s)
const double T = 0.001;     //采样间隔
//const double wheelbase = 11.97;   //底盘轴距
const double tread = 12.13; //底盘轮距


/*************************************************************
*成员函数：run
*函数功能：控制两侧轮子转速
*函数参数：l-左侧线速度 r-右侧线速度
*注意事项：单位为米每秒
**************************************************************/
void chassis::run(double l, double r)
{
    l = l / propotion * 60;
    r = r / propotion * 60;
    LeftMotorGroup.spin(forward, l, rpm);
    RightMotorGroup.spin(forward, r, rpm);
}

/*******************************************************************
*函数名称：turn
*函数作用：转过指定的角度
*函数参量：angle-转过的角度
*注意事项：angle是一个[-Π,Π]的量
*********************************************************************/
void chassis::turn(double angle)
{
    const double T = 0.001;     //采样间隔

    double kp = 10, ki = 0.1, kd = 0.1;   //PID的三个常量，明天一定改好
    double chassis_angle = 0;     //初始位置定为0
    double error_angle = angle, just_error_angle;   //误差值，和前一个误差值
    double init_pos = LeftMotorGroup.position(rev);   //起始时的电机位置
    double v = 0;    //控制的速度

    double integrate_angle = 0;
    double diff_angle = 0;

    while (fabs(error_angle) > 10e-2 || fabs(v) > 10e-2){
        Brain.Screen.printAt(10, 40, "error_angle = %f", error_angle);
        Brain.Screen.printAt(10, 80, "chassis_angle = %f", chassis_angle);
        just_error_angle = error_angle;     //更新前一个误差值
        error_angle = angle - chassis_angle;    //更新误差值
        integrate_angle += error_angle * T;   //计算积分
        diff_angle = (error_angle - just_error_angle) / T;    //计算微分

        v = kp * error_angle + ki * integrate_angle + kd * diff_angle;  //PID算法计算速度
        
        //防止超速
        if (v > max_v){
            run(-max_v, max_v);
        }
        else{
            run(-v , v);
        }

        //更新位置
        chassis_angle = -propotion * (LeftMotorGroup.position(rev) - init_pos) / tread * 2;

        //间隔
        task::sleep(T * 1000);
    }
}

/*******************************************************************
*函数名称：move
*函数作用：向前行进指定的距离
*函数参量：distance-距离
*注意事项：distance是一个正数
*********************************************************************/
void chassis::move(double distance)
{

    double kp = 2, ki = 0.1, kd = 0.1;   //PID的三个常量，明天一定改好
    double chassis_route = 0;     //初始位置定为0
    double error_route = distance, just_error_route;   //误差值，和前一个误差值
    double init_pos = LeftMotorGroup.position(rev);   //起始时的电机位置
    double v = 0;    //控制的速度

    double integrate_route = 0;
    double diff_route = 0;

    while (fabs(error_route) > 10e-2 || fabs(v) > 10e-2){
        Brain.Screen.printAt(10, 40, "error_angle = %f", error_route);
        Brain.Screen.printAt(10, 80, "chassis_angle = %f", chassis_route);
        just_error_route = error_route;     //更新前一个误差值
        error_route = distance - chassis_route;    //更新误差值
        integrate_route += error_route * T;   //计算积分
        diff_route = (error_route - just_error_route) / T;    //计算微分

        v = kp * error_route + ki * integrate_route + kd * diff_route;  //PID算法计算速度
        
        //防止超速
        if (v > max_v){
            run(max_v, max_v);
        }
        else{
            run(v , v);
        }

        //更新位置
        chassis_route = propotion * (LeftMotorGroup.position(rev) - init_pos) / tread * 2;

        //间隔
        task::sleep(T * 1000);
    }
}

/*******************************************************************
*函数名称：record
*函数作用：记录此时的坐标和朝向
*函数参量：无
*注意事项：构造函数时就开始，开一个新线程，到自动结束停止
*********************************************************************/
// static int record(coordinate co, double toward)
// {
//     double left_motor[2] = {LeftMotorGroup.position(rev), LeftMotorGroup.position(rev)};
//     double right_motor[2] = {RightMotorGroup.position(rev), RightMotorGroup.position(rev)};
//     double v_left, v_right;

//     while(1){
//         left_motor[1] = left_motor[0];
//         left_motor[0] = LeftMotorGroup.position(rev);
//         v_left = propotion * (left_motor[0] - left_motor[1]);   //左侧线速度

//         right_motor[1] = right_motor[0];
//         right_motor[0] = RightMotorGroup.position(rev);
//         v_right = propotion * (right_motor[0] - right_motor[1]);  //右侧线速度

//         //计算速度和转动速度
//         co.x += sin(toward) * (v_left + v_right) / 2;
//         co.y += cos(toward) * (v_left + v_right) / 2;
//         toward += (-v_left + v_right) / 2 / tread / 2 / M_PI;

//         Brain.Screen.printAt(10, 20, "x = %f", co.x);
//         Brain.Screen.printAt(10, 50, "y = %f", co.y);

//         task::sleep(T * 1000);
//     }
//     return 0;
// }

/*******************************************************************
*函数名称：chassis，是本对象的构造函数
*函数作用：初始化，给定初始的坐标值和朝向
*函数参量：x-初始位置的横坐标，y-初始位置的纵坐标，toward-初始朝向
*********************************************************************/
chassis::chassis(double x, double y, double now_toward)
{
    co.x = x;
    co.y = y;
    toward = now_toward;
    // task t1(record);
}

/*******************************************************************
*函数名称：go_to
*函数作用：移动到指定坐标
*函数参量：x-目标位置的横坐标，y-目标位置的纵坐标，R为倒挡，默认为0，要倒车输入非0
*********************************************************************/
void chassis::go_to(double x, double y, int R) 
{
    
    if (R){
        toward += M_PI;
    }
    //转动
    double target_angle;    //目标角度
    //计算目标角度
    if (y > 0) {
        if (co.y == y){
            target_angle = M_PI / 2;
        }
        else{
            target_angle = atan((y - co.y) / (x - co.x)); 
        }
    }
    else {
        if (co.y == y){
            target_angle = M_PI * 3 / 2;
        }
        else{
            target_angle = atan((y - co.y) / (x - co.x)) + M_PI;; 
        }
    }
    //计算要转的角度，范围为[-Π,Π]
    double angle = target_angle - toward;   //要转的角度
    if (angle> M_PI) {
        angle -= 2 * M_PI;
    }
    else if(angle < -M_PI) {
        angle += 2 * M_PI;
    }
    turn(angle);    //调旋转函数
    toward = target_angle;
    Brain.Screen.printAt(0, 0, "angle = %f", angle);
    //前进
    double distance = sqrt(pow((x - co.x), 2) + pow((y - co.y), 2));    //前进的距离
    move(distance - distance_cal);
    co.x = x - distance_cal - cos(toward);
    co.y = y - distance_cal - sin(toward);
}

/*******************************************************************
*函数名称：position
*函数作用：获取当前坐标
*函数参量：x-当前位置的横坐标，y-当前位置的纵坐标，toward当前朝向
*********************************************************************/
void chassis::position(double &x, double &y, double &now_toward)
{
    x = co.x;
    y = co.y;
    now_toward = toward;
}

