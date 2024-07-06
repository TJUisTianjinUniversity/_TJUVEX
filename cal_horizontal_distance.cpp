/***************************************
函数名称：horizontal_length
输入参数：1.机械臂未启动时初始相位(角度）；2.机械臂长度 3.人为读取的初相位
返回值:爪子与机械臂连接端的水平距离
说明：
****************************************/
double horizontal_length(double ini_phi, double ini_deg)
{
	double length_of_leg = ;
	double current_deg = Motor0.position(rev) * 2 * M_PI;
	double horizontal_length = length_of_leg * cos(current_deg - ini_deg - ini_phi);
	return horizontal_length;
}