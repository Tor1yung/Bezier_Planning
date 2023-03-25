#define _USE_MATH_DEFINES
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include "Paint.h"
#include "Bezier.h"
#include "Pole.h"
#include "PublicDefine.h"

using namespace std;
using namespace cv;

/*����*/

string pic_path = "D:\work\robot\code\material\map.png";		//����ͼƬ·��

char canvas1_name[] = "canvas_1";	//������
char canvas2_name[] = "canvas_2";	//������
Mat canvas1(HEIGHT, WIDTH, CV_8UC3, Scalar(255,255,255));	//��ɫ����
Mat canvas2(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//��ɫ����
Mat canvas3(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//��ɫ����



double ControlPoint[][2] = { {30,30},{100,120},{200,170},{320,250},{400,150},{580,580} };		//���������߿��Ƶ㣬����Ӷ���
vector<vector<int>>ControlPoints;
int realmapHEIGHT = 7.5; 		    //��ʵ��ͼ�ߴ�(��)
int realmapWIDTH = 9.5;			    //��ʵ��ͼ�ߴ�(��)
int real2bitSCALE = 100;			//����ʵ��ͼ�ߴ��1:1ʱ��ʹ�ñ����ߵȱ����Ŵ�



#if STATIC_CENTER
int map_scale = HEIGHT / map_radius;	//���ü������ͼ��С�ͻ����任�߶�
#else	
int map_scale = 100;					//��ͼ�任�߶�
#endif // !STATIC_CENTER




/*����*/






void clear_canvas(Mat CANVAS)
{
	/* ��հ׻��� */

	Mat canvas0(CANVAS.rows, CANVAS.cols, CV_8UC3, Scalar(255, 255, 255));	//��ɫ��������ʼ����
	addWeighted(CANVAS, 0, canvas0, 1, 0, CANVAS, -1);
}

void fuse_canvas(Mat CANVAS1, Mat CANVAS2)
{
	/* �ں����Ż� */

	Mat canvas0(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//��ɫ��������ʼ����
	min(CANVAS1, CANVAS2, CANVAS2);
}


//��ͼ
void Paint_Bezier(void)
{
	/*�����������Ƶ�ֱ��*/

	clear_canvas(canvas3);	//��ʼ��Ϊ�հ׻���
	for (int i = 0; i < lenofarray(ControlPoint)-1; i++)
	{
		line(canvas1, Point(ControlPoint[i][0], HEIGHT - ControlPoint[i][1]), Point(ControlPoint[i+1][0], HEIGHT - ControlPoint[i+1][1]),Scalar(0,0,255),2, LINE_8);	//�����Ƶ����ɵ���
	}
	circle(canvas2, Point(Bezier_Point[0], HEIGHT - Bezier_Point[1]), 1, Scalar(0, 255, 0), -1, LINE_8);	//���켣��opencv��y������������������ҪHEIGHT������
	circle(canvas3, Point(Bezier_Point[0], HEIGHT - Bezier_Point[1]), 4, Scalar(255, 0, 255), -1, LINE_8);	//�����㣬opencv��y������������������ҪHEIGHT������
	fuse_canvas(canvas1,canvas2);	//�ں�����ͼ
	fuse_canvas(canvas2, canvas3);	//�ں�����ͼ

	imshow(canvas1_name, canvas3);
}


void Paint_Pole()
{
	/* ���Ƽ�������̼���Ϣ */

	clear_canvas(canvas1); //ÿ֡ͼ�������»���

	//�����������
		//ͬ��Բ
	for (int circle_radius = map_radius / 2; circle_radius > 0; circle_radius -= 1)
	{
		circle(canvas1, Point(map_radius / 2 * map_scale, map_radius / 2 * map_scale), circle_radius * map_scale, Scalar(180,180, 180), 1.5, LINE_8);
	}
	
		//������
	line(canvas1, Point(map_radius / 2 * map_scale, 0), Point(map_radius / 2 * map_scale, map_radius * map_scale), Scalar(180, 180, 180), 1.5);	//Y��
	line(canvas1, Point(0,map_radius / 2 * map_scale), Point( map_radius * map_scale, map_radius / 2 * map_scale), Scalar(180, 180, 180), 1.5);	//x��
	
	//Ŀ���
	for (int i = 0; i < TargetPointNum;i++)
	{
		circle(canvas1, Point((TargetPoint[i][0] + map_radius / 2)* map_scale, (TargetPoint[i][1] + map_radius / 2) * map_scale), PointSize, Scalar(0, 100, 0), -1, LINE_8);
	}
	//����
	line(canvas1, Point((MovingPoint[0] + map_radius / 2)* map_scale, (MovingPoint[1] + map_radius / 2) * map_scale), Point(map_radius / 2 * map_scale, map_radius / 2 * map_scale), Scalar(0, 0, 255), 2);	
	//����
	circle(canvas1, Point((MovingPoint[0] + map_radius / 2)* map_scale, (MovingPoint[1] + map_radius / 2) * map_scale), PointSize, Scalar(0, 255, 0), -1, LINE_8);

	imshow(canvas1_name, canvas1);
	waitKey(1);	
}


void PaintIndoor()
{
	/* ��������ͼ */

	clear_canvas(canvas1); //ÿ֡ͼ�������»���
	//����
		//�������ı�Ե����
		//ʵʱ�ı�Ե����
		double RealMapProfile[2] = { 6.6,4.8 };				//ʵ�ʵ�ͼ�ߴ�(����)
		vector<vector<double>>MapContour;					//��ͼ��������
		for (double i = 0; i < RealMapProfile[0]; i += 0.05)	//�����������
		{
			for (double j = 0; j < RealMapProfile[1]; j += 0.05)
			{
				if ((i == 0) || (i > RealMapProfile[0]-0.05))
				{
					vector<double>Temp = { i,j };
					MapContour.push_back(Temp);
					circle(canvas1, Point(Temp[0] * map_scale, Temp[1] * map_scale), 2, Scalar(255, 0, 0), -1, LINE_8);
				}
				else if(j == 0 || j > RealMapProfile[1]-0.05)
				{
					vector<double>Temp = { i,j };
					MapContour.push_back(Temp);
					circle(canvas1, Point(Temp[0] * map_scale, Temp[1] * map_scale), 2, Scalar(255, 0, 0), -1, LINE_8);
				}					
			}			
		}
	//������
		for (int i=0; i < WayPoint.size(); i++)
		{
			circle(canvas1, Point(WayPoint[i][0] * map_scale, WayPoint[i][1] * map_scale), PointSize, Scalar(150, 0, 150), -1, LINE_8);
		}
	//�켣
		//��ʼ����
		circle(canvas1, Point(TracePoint[0][0] * map_scale, TracePoint[0][1] * map_scale), PointSize, Scalar(150, 0, 150), -1, LINE_8);
		//��������ǰ���������ĵ㵽����
		line(canvas1, Point(CurrentCenterPoint[0] * map_scale, CurrentCenterPoint[1] * map_scale), Point(MovingPoint[0] * map_scale, MovingPoint[1] * map_scale), Scalar(0, 0, 255), 2);
		//����
		circle(canvas1, Point(MovingPoint[0]* map_scale, MovingPoint[1]* map_scale), PointSize, Scalar(0, 255, 0), -1, LINE_8);

		line(canvas1, Point(CurrentCenterPoint[0] * map_scale, CurrentCenterPoint[1] * map_scale), Point((CurrentCenterPoint[0] + 0.1*cos(theta / 180 * M_PI)) * map_scale, (CurrentCenterPoint[1] + 0.1*sin(theta / 180 * M_PI)) * map_scale), Scalar(255, 0, 0), 2);

	imshow(canvas1_name, canvas1);
	waitKey(1);
}



void OnMouseAction(int event, int x, int y, int flags, void *ustc)
{
	/* ���ص����ܺ��� */

	if (event == EVENT_LBUTTONDOWN)
	{
		vector<int>GetPoint = { x,y };
		//ControlPoints.push_back(GetPoint);
		cout<<"XY:"<<x<<y<<endl;
	}
	if (event == EVENT_RBUTTONDOWN)
	{
		vector<int>GetPoint = { x,y };
		Bezier_WayPoint.push_back(GetPoint);
	}
}


void Paint_Indoor_Bezier(Bezier B)
{
	Mat canvas1 = imread("D:/work/robot/material/map.png");
	resize(canvas1, canvas1, Size(950, 750));



#ifdef REAL_SCALE
	for(int i=0;i< B.QPSolution.size()/2;i++)					//�������Ƶ�
	{
		circle(canvas1, Point(B.QPSolution[i]*B.sj[i/4]*real2bitSCALE, (7 - B.QPSolution[i+B.CtrlPNum*B.SegNum] * B.sj[i / 4])*real2bitSCALE), PointSize, Scalar(0, 0, 0), -1, LINE_8);
	}
	for (int i = 0; i < B.SegNum; i++)		//����������
	{
		circle(canvas1, Point(B.Waypoints[i][0]*real2bitSCALE, (7 - B.Waypoints[i][1])*real2bitSCALE), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}
	for (int i = 0; i < BezierCurve_Point.size(); i++)
	{
		circle(canvas1, Point(BezierCurve_Point[i][0]*real2bitSCALE, (7 - BezierCurve_Point[i][1])*real2bitSCALE), PointSize/3, Scalar(0, 255, 0), -1, LINE_8);
	}

	imshow(canvas1_name, canvas1);	
	//setMouseCallback(canvas1_name, OnMouseAction, &canvas1);	//�����갴��



#else
	for (int i = 0; i < ControlPoints.size(); i++)					//�������Ƶ�
	{
		circle(canvas1, Point(ControlPoints[i][0], ControlPoints[i][1]), PointSize, Scalar(0, 0, 0), -1, LINE_8);
}
	for (int i = 0; i < Bezier_WayPoint.size(); i++)		//����������
	{
		circle(canvas1, Point(Bezier_WayPoint[i][0], Bezier_WayPoint[i][1]), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}

	for (int i = 0; i < BezierCurve_Point.size(); i++)
	{
		circle(canvas1, Point(BezierCurve_Point[i][0], BezierCurve_Point[i][1]), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}

	imshow(canvas1_name, canvas1);
	setMouseCallback(canvas1_name, OnMouseAction, &canvas1);	//�����갴��

#endif
}

//
//int main(void)
//{	
//	int num = lenofarray(ControlPoint);	//�����ϰ�����
//	for (double i = 0; i <= 1; i += 0.001)
//	{
//		Count_Bezier(ControlPoint,num,i);
//		Paint_Bezier();
//		waitKey(1);  //ms
//	}
//		
//}


int main(void)
{


	//canvas1 = imread("D:/work/robot/code/material/map.png");
	//
	//cout<<canvas1.rows<<endl;
	//clear_canvas(canvas1);	//��ʼ��Ϊ�հ׻���
	//imshow(canvas1_name, canvas1);
	//setMouseCallback(canvas1_name, OnMouseAction, reinterpret_cast<void*>(&canvas1));
	//waitKey(1);
	Bezier B;
	B.Bezier_Init();
	B.Bezier_Solve();

		
	for (double t=0;;t+=0.5)
	{
		B.Count_Bezier_Vector(t);
		Paint_Indoor_Bezier(B);
		waitKey(1);
		if (t >= B.sj[0]+ B.sj[1]+ B.sj[2]+ B.sj[3]+ B.sj[4]+ B.sj[5]+ B.sj[6]+B.sj[7])
		{
			system("pause");
			return 0;
		}
	}
}





