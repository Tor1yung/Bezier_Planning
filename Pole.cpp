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



double map_radius = 10;									//极坐标地图极径
double theta = 0;										//极角
double radius = 0;										//极径
double delta_theta = 1.5;									//极角变化量
double delta_radius = 0.05;								//极径变化量
double TargetPoint[][2] = { {1,1},{2,3},{3,1},{4,4} };	//目标点
double CenterPoint[2] = { 0,0 };						//绘图中心点
vector<double>MovingPoint(2,0);							//动点(极坐标)中心点
int TargetPointNum = lenofarray(TargetPoint);			//目标点个数
double PointSize = 4.5;									//点的大小

vector<vector<double>>TracePoint;				//轨迹点集，一开始只有起始点

double CurrentCenterPoint[2] = { 1,1 };			//当前极坐标中心点，初始化为起始点



//int main()
//{									
//	Pole_Control_Running();
//	
//}


void Pole_Control_Running(void)
{
	/* 在极坐标表盘上控制动点到达目标点 */

	for (;;)
	{
		if ((GetAsyncKeyState(VK_UP) && 0x8000) && (radius < map_radius))	//检测↑	增大极径，最大为地图极径
		{
			radius += delta_radius;
			cout << "当前极径：" << radius << endl;
		}
		if ((GetAsyncKeyState(VK_DOWN) && 0x8000) && (radius > 0))	//检测↓	缩小极径，最小为0
		{
			radius -= delta_radius;
			cout << "当前极径：" << radius << endl;
		}
		if ((GetAsyncKeyState(VK_RIGHT) && 0x8000))	//检测→	增大极角
		{
			if (theta == 360) { theta = 0; }	//极角达到360时重置为0
			theta += delta_theta;
			cout << "当前极角：" << theta << endl;
		}
		if ((GetAsyncKeyState(VK_LEFT) && 0x8000))	//检测←	减小极角
		{
			if (theta == 0) { theta = 360; }	//极角降到0时重置为360
			theta -= delta_theta;
			cout << "当前极角：" << theta << endl;
		}

#if STATIC_CENTER

		MovingPoint[0] = radius * cos(theta / 180 * M_PI);
		MovingPoint[1] = radius * sin(theta / 180 * M_PI);

		//到达任意一个目标点则更新坐标原点
		for (int i = 0; i < TargetPointNum; i++)
		{
			//动点中心坐标进入目标点外切矩形范围内则为到达
			if ((MovingPoint[0] > (TargetPoint[i][0] - PointSize / map_scale / 1.5)) && (MovingPoint[0] < (TargetPoint[i][0] + PointSize / map_scale / 1.5)) && (MovingPoint[1] > (TargetPoint[i][1] - PointSize / map_scale / 1.5)) && (MovingPoint[1] < (TargetPoint[i][1] + PointSize / map_scale / 1.5)))
			{
				//目标点位置减去动点位置得到相对原点位置
				for (int ii = 0; ii < TargetPointNum; ii++)
				{
					if (ii != i)	//除了到达的目标点本身
					{
						TargetPoint[ii][0] -= MovingPoint[0];
						TargetPoint[ii][1] -= MovingPoint[1];
					}
				}
				//清除到达的目标点
				ArrayElementRemove(TargetPoint, i, TargetPointNum)


					//动点更新为原点
					MovingPoint[0] = CenterPoint[0];
				MovingPoint[1] = CenterPoint[1];
				//极角极径重置
				theta = 0;
				radius = 0;
				break;
			}
		}

#else

		////////////////////////定义变量//////////////////////////

		//起始点(相对绘图中心的偏移)
		vector<double>StartPoint = {1,1};	//起始点

		//经过点(复数个)
		double WayPointArray[][2] = { {2,2},{4,3},{6,2} };	//经过点实际数值
		int WayPointNum = lenofarray(WayPointArray);			//经过点个数
		for (int WayPointCount = 0; WayPointCount < WayPointNum; WayPointCount++)	//添加经过点到容器
		{
			vector<double>Temp(WayPointArray[WayPointCount],WayPointArray[WayPointCount] + 2);
			WayPoint.push_back(Temp);
		}

		TracePoint.push_back(StartPoint);

		//double RealMapProfile[2] = {6.6,4.8};				//实际地图尺寸(长宽)
		//vector<vector<double>>MapContour;					//地图轮廓数据
		//for (double i = 0; i < RealMapProfile[0]; i += 0.1)	//填充轮廓数据
		//{
		//	for (double j = 0; j < RealMapProfile[1]; j += 0.1)
		//	{
		//		vector<double>Temp = {i,j};
		//		MapContour.push_back(Temp);
		//	}
		//}
		////////////////////////定义变量//////////////////////////


		//动点坐标更新计算：动点相对当前极坐标原点的坐标+当前极坐标原点在相对绘图中心的坐标得到
		MovingPoint[0] = radius * cos(theta / 180 * M_PI) + CurrentCenterPoint[0];
		MovingPoint[1] = radius * sin(theta / 180 * M_PI) + CurrentCenterPoint[1];

		TracePoint.push_back(MovingPoint);	//添加当前动点轨迹

		//到达新的目标点则更新参数
		for (int i = 0; i < WayPointNum; i++)
		{
			//动点中心坐标进入目标点外切矩形范围内则为到达
			if ((MovingPoint[0] > (WayPoint[i][0] - PointSize / map_scale / 1.5)) && (MovingPoint[0] < (WayPoint[i][0] + PointSize / map_scale / 1.5)) && (MovingPoint[1] > (WayPoint[i][1] - PointSize / map_scale / 1.5)) && (MovingPoint[1] < (WayPoint[i][1] + PointSize / map_scale / 1.5)))
			{
				//更新"上一极坐标原点"
				CurrentCenterPoint[0] = MovingPoint[0];
				CurrentCenterPoint[1] = MovingPoint[1];

				//极角极径重置
				theta = 0;
				radius = 0;

				//清除到达的目标点
				VectorElementRemove(&WayPoint,i);
				break;
			}
		}
#endif

		//Paint_Pole();
		PaintIndoor();	//画图

		Sleep(50);//ms
	}
}

