/***************************************
�������ƣ�horizontal_length
���������1.��е��δ����ʱ��ʼ��λ(�Ƕȣ���2.��е�۳��� 3.��Ϊ��ȡ�ĳ���λ
����ֵ:צ�����е�����Ӷ˵�ˮƽ����
˵����
****************************************/
double horizontal_length(double ini_phi, double ini_deg)
{
	double length_of_leg = ;
	double current_deg = Motor0.position(rev) * 2 * M_PI;
	double horizontal_length = length_of_leg * cos(current_deg - ini_deg - ini_phi);
	return horizontal_length;
}