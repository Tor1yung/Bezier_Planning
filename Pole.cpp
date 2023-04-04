#define _USE_MATH_DEFINES
#include<vector>
#include <opencv2/opencv.hpp>
#include <windows.h>
#include <cmath>
#include "Pole.h"
#include "Paint.h"
#include "PublicDefine.h"



using namespace cv;
using namespace std;



double map_radius = 10;									//�������ͼ����
double theta = 0;										//����
double radius = 0;										//����
double delta_theta = 1.5;									//���Ǳ仯��
double delta_radius = 0.05;								//�����仯��
double TargetPoint[][2] = { {1,1},{2,3},{3,1},{4,4} };	//Ŀ���
double CenterPoint[2] = { 0,0 };						//��ͼ���ĵ�
vector<double>MovingPoint(2,0);							//����(������)���ĵ�
int TargetPointNum = lenofarray(TargetPoint);			//Ŀ������
double PointSize = 4.5;									//��Ĵ�С

vector<vector<double>>TracePoint;				//�켣�㼯��һ��ʼֻ����ʼ��

double CurrentCenterPoint[2] = { 1,1 };			//��ǰ���������ĵ㣬��ʼ��Ϊ��ʼ��



//int main()
//{									
//	Pole_Control_Running();
//	
//}


void Pole_Control_Running(void)
{
	/* �ڼ���������Ͽ��ƶ��㵽��Ŀ��� */

	for (;;)
	{
		if ((GetAsyncKeyState(VK_UP) && 0x8000) && (radius < map_radius))	//����	���󼫾������Ϊ��ͼ����
		{
			radius += delta_radius;
			cout << "��ǰ������" << radius << endl;
		}
		if ((GetAsyncKeyState(VK_DOWN) && 0x8000) && (radius > 0))	//����	��С��������СΪ0
		{
			radius -= delta_radius;
			cout << "��ǰ������" << radius << endl;
		}
		if ((GetAsyncKeyState(VK_RIGHT) && 0x8000))	//����	���󼫽�
		{
			if (theta == 360) { theta = 0; }	//���Ǵﵽ360ʱ����Ϊ0
			theta += delta_theta;
			cout << "��ǰ���ǣ�" << theta << endl;
		}
		if ((GetAsyncKeyState(VK_LEFT) && 0x8000))	//����	��С����
		{
			if (theta == 0) { theta = 360; }	//���ǽ���0ʱ����Ϊ360
			theta -= delta_theta;
			cout << "��ǰ���ǣ�" << theta << endl;
		}

#if STATIC_CENTER

		MovingPoint[0] = radius * cos(theta / 180 * M_PI);
		MovingPoint[1] = radius * sin(theta / 180 * M_PI);

		//��������һ��Ŀ������������ԭ��
		for (int i = 0; i < TargetPointNum; i++)
		{
			//���������������Ŀ������о��η�Χ����Ϊ����
			if ((MovingPoint[0] > (TargetPoint[i][0] - PointSize / map_scale / 1.5)) && (MovingPoint[0] < (TargetPoint[i][0] + PointSize / map_scale / 1.5)) && (MovingPoint[1] > (TargetPoint[i][1] - PointSize / map_scale / 1.5)) && (MovingPoint[1] < (TargetPoint[i][1] + PointSize / map_scale / 1.5)))
			{
				//Ŀ���λ�ü�ȥ����λ�õõ����ԭ��λ��
				for (int ii = 0; ii < TargetPointNum; ii++)
				{
					if (ii != i)	//���˵����Ŀ��㱾��
					{
						TargetPoint[ii][0] -= MovingPoint[0];
						TargetPoint[ii][1] -= MovingPoint[1];
					}
				}
				//��������Ŀ���
				ArrayElementRemove(TargetPoint, i, TargetPointNum)


					//�������Ϊԭ��
					MovingPoint[0] = CenterPoint[0];
				MovingPoint[1] = CenterPoint[1];
				//���Ǽ�������
				theta = 0;
				radius = 0;
				break;
			}
		}

#else

		////////////////////////�������//////////////////////////

		//��ʼ��(��Ի�ͼ���ĵ�ƫ��)
		vector<double>StartPoint = {1,1};	//��ʼ��

		//������(������)
		double WayPointArray[][2] = { {2,2},{4,3},{6,2} };	//������ʵ����ֵ
		int WayPointNum = lenofarray(WayPointArray);			//���������
		for (int WayPointCount = 0; WayPointCount < WayPointNum; WayPointCount++)	//��Ӿ����㵽����
		{
			vector<double>Temp(WayPointArray[WayPointCount],WayPointArray[WayPointCount] + 2);
			WayPoint.push_back(Temp);
		}

		TracePoint.push_back(StartPoint);

		//double RealMapProfile[2] = {6.6,4.8};				//ʵ�ʵ�ͼ�ߴ�(����)
		//vector<vector<double>>MapContour;					//��ͼ��������
		//for (double i = 0; i < RealMapProfile[0]; i += 0.1)	//�����������
		//{
		//	for (double j = 0; j < RealMapProfile[1]; j += 0.1)
		//	{
		//		vector<double>Temp = {i,j};
		//		MapContour.push_back(Temp);
		//	}
		//}
		////////////////////////�������//////////////////////////


		//����������¼��㣺������Ե�ǰ������ԭ�������+��ǰ������ԭ������Ի�ͼ���ĵ�����õ�
		MovingPoint[0] = radius * cos(theta / 180 * M_PI) + CurrentCenterPoint[0];
		MovingPoint[1] = radius * sin(theta / 180 * M_PI) + CurrentCenterPoint[1];

		TracePoint.push_back(MovingPoint);	//��ӵ�ǰ����켣

		//�����µ�Ŀ�������²���
		for (int i = 0; i < WayPointNum; i++)
		{
			//���������������Ŀ������о��η�Χ����Ϊ����
			if ((MovingPoint[0] > (WayPoint[i][0] - PointSize / map_scale / 1.5)) && (MovingPoint[0] < (WayPoint[i][0] + PointSize / map_scale / 1.5)) && (MovingPoint[1] > (WayPoint[i][1] - PointSize / map_scale / 1.5)) && (MovingPoint[1] < (WayPoint[i][1] + PointSize / map_scale / 1.5)))
			{
				//����"��һ������ԭ��"
				CurrentCenterPoint[0] = MovingPoint[0];
				CurrentCenterPoint[1] = MovingPoint[1];

				//���Ǽ�������
				theta = 0;
				radius = 0;

				//��������Ŀ���
				VectorElementRemove(&WayPoint,i);
				break;
			}
		}
#endif

		//Paint_Pole();
		PaintIndoor();	//��ͼ

		Sleep(50);//ms
	}
}

