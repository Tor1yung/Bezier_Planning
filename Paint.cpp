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

/*定义*/

string pic_path = "D:\work\robot\code\material\map.png";		//加载图片路径

char canvas1_name[] = "canvas_1";	//画布名
char canvas2_name[] = "canvas_2";	//画布名
Mat canvas1(HEIGHT, WIDTH, CV_8UC3, Scalar(255,255,255));	//白色画布
Mat canvas2(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//白色画布
Mat canvas3(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//白色画布



double ControlPoint[][2] = { {30,30},{100,120},{200,170},{320,250},{400,150},{580,580} };		//贝塞尔曲线控制点，可添加多组
vector<vector<int>>ControlPoints;
int realmapHEIGHT = 7.5; 		    //真实地图尺寸(高)
int realmapWIDTH = 9.5;			    //真实地图尺寸(宽)
int real2bitSCALE = 100;			//当真实地图尺寸非1:1时，使用比例尺等比例放大



#if STATIC_CENTER
int map_scale = HEIGHT / map_radius;	//设置极坐标地图大小和画布变换尺度
#else	
int map_scale = 100;					//地图变换尺度
#endif // !STATIC_CENTER




/*定义*/






void clear_canvas(Mat CANVAS)
{
	/* 清空白画布 */

	Mat canvas0(CANVAS.rows, CANVAS.cols, CV_8UC3, Scalar(255, 255, 255));	//白色画布，初始化用
	addWeighted(CANVAS, 0, canvas0, 1, 0, CANVAS, -1);
}

void fuse_canvas(Mat CANVAS1, Mat CANVAS2)
{
	/* 融合两张画 */

	Mat canvas0(HEIGHT, WIDTH, CV_8UC3, Scalar(255, 255, 255));	//白色画布，初始化用
	min(CANVAS1, CANVAS2, CANVAS2);
}


//画图
void Paint_Bezier(void)
{
	/*画贝塞尔控制点直线*/

	clear_canvas(canvas3);	//初始化为空白画布
	for (int i = 0; i < lenofarray(ControlPoint)-1; i++)
	{
		line(canvas1, Point(ControlPoint[i][0], HEIGHT - ControlPoint[i][1]), Point(ControlPoint[i+1][0], HEIGHT - ControlPoint[i+1][1]),Scalar(0,0,255),2, LINE_8);	//画控制点连成的线
	}
	circle(canvas2, Point(Bezier_Point[0], HEIGHT - Bezier_Point[1]), 1, Scalar(0, 255, 0), -1, LINE_8);	//画轨迹，opencv的y轴向下生长，所以需要HEIGHT来修正
	circle(canvas3, Point(Bezier_Point[0], HEIGHT - Bezier_Point[1]), 4, Scalar(255, 0, 255), -1, LINE_8);	//画动点，opencv的y轴向下生长，所以需要HEIGHT来修正
	fuse_canvas(canvas1,canvas2);	//融合两个图
	fuse_canvas(canvas2, canvas3);	//融合两个图

	imshow(canvas1_name, canvas3);
}


void Paint_Pole()
{
	/* 绘制极坐标表盘及信息 */

	clear_canvas(canvas1); //每帧图都是重新绘制

	//画极坐标表盘
		//同心圆
	for (int circle_radius = map_radius / 2; circle_radius > 0; circle_radius -= 1)
	{
		circle(canvas1, Point(map_radius / 2 * map_scale, map_radius / 2 * map_scale), circle_radius * map_scale, Scalar(180,180, 180), 1.5, LINE_8);
	}
	
		//坐标轴
	line(canvas1, Point(map_radius / 2 * map_scale, 0), Point(map_radius / 2 * map_scale, map_radius * map_scale), Scalar(180, 180, 180), 1.5);	//Y轴
	line(canvas1, Point(0,map_radius / 2 * map_scale), Point( map_radius * map_scale, map_radius / 2 * map_scale), Scalar(180, 180, 180), 1.5);	//x轴
	
	//目标点
	for (int i = 0; i < TargetPointNum;i++)
	{
		circle(canvas1, Point((TargetPoint[i][0] + map_radius / 2)* map_scale, (TargetPoint[i][1] + map_radius / 2) * map_scale), PointSize, Scalar(0, 100, 0), -1, LINE_8);
	}
	//极径
	line(canvas1, Point((MovingPoint[0] + map_radius / 2)* map_scale, (MovingPoint[1] + map_radius / 2) * map_scale), Point(map_radius / 2 * map_scale, map_radius / 2 * map_scale), Scalar(0, 0, 255), 2);	
	//动点
	circle(canvas1, Point((MovingPoint[0] + map_radius / 2)* map_scale, (MovingPoint[1] + map_radius / 2) * map_scale), PointSize, Scalar(0, 255, 0), -1, LINE_8);

	imshow(canvas1_name, canvas1);
	waitKey(1);	
}


void PaintIndoor()
{
	/* 绘制室内图 */

	clear_canvas(canvas1); //每帧图都是重新绘制
	//框体
		//×完整的边缘数据
		//实时的边缘数据
		double RealMapProfile[2] = { 6.6,4.8 };				//实际地图尺寸(长宽)
		vector<vector<double>>MapContour;					//地图轮廓数据
		for (double i = 0; i < RealMapProfile[0]; i += 0.05)	//填充轮廓数据
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
	//经过点
		for (int i=0; i < WayPoint.size(); i++)
		{
			circle(canvas1, Point(WayPoint[i][0] * map_scale, WayPoint[i][1] * map_scale), PointSize, Scalar(150, 0, 150), -1, LINE_8);
		}
	//轨迹
		//起始点标点
		circle(canvas1, Point(TracePoint[0][0] * map_scale, TracePoint[0][1] * map_scale), PointSize, Scalar(150, 0, 150), -1, LINE_8);
		//极径：当前极坐标中心点到动点
		line(canvas1, Point(CurrentCenterPoint[0] * map_scale, CurrentCenterPoint[1] * map_scale), Point(MovingPoint[0] * map_scale, MovingPoint[1] * map_scale), Scalar(0, 0, 255), 2);
		//动点
		circle(canvas1, Point(MovingPoint[0]* map_scale, MovingPoint[1]* map_scale), PointSize, Scalar(0, 255, 0), -1, LINE_8);

		line(canvas1, Point(CurrentCenterPoint[0] * map_scale, CurrentCenterPoint[1] * map_scale), Point((CurrentCenterPoint[0] + 0.1*cos(theta / 180 * M_PI)) * map_scale, (CurrentCenterPoint[1] + 0.1*sin(theta / 180 * M_PI)) * map_scale), Scalar(255, 0, 0), 2);

	imshow(canvas1_name, canvas1);
	waitKey(1);
}



void OnMouseAction(int event, int x, int y, int flags, void *ustc)
{
	/* 鼠标回调功能函数 */

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
	for(int i=0;i< B.QPSolution.size()/2;i++)					//画出控制点
	{
		circle(canvas1, Point(B.QPSolution[i]*B.sj[i/4]*real2bitSCALE, (7 - B.QPSolution[i+B.CtrlPNum*B.SegNum] * B.sj[i / 4])*real2bitSCALE), PointSize, Scalar(0, 0, 0), -1, LINE_8);
	}
	for (int i = 0; i < B.SegNum; i++)		//画出经过点
	{
		circle(canvas1, Point(B.Waypoints[i][0]*real2bitSCALE, (7 - B.Waypoints[i][1])*real2bitSCALE), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}
	for (int i = 0; i < BezierCurve_Point.size(); i++)
	{
		circle(canvas1, Point(BezierCurve_Point[i][0]*real2bitSCALE, (7 - BezierCurve_Point[i][1])*real2bitSCALE), PointSize/3, Scalar(0, 255, 0), -1, LINE_8);
	}

	imshow(canvas1_name, canvas1);	
	//setMouseCallback(canvas1_name, OnMouseAction, &canvas1);	//检测鼠标按键



#else
	for (int i = 0; i < ControlPoints.size(); i++)					//画出控制点
	{
		circle(canvas1, Point(ControlPoints[i][0], ControlPoints[i][1]), PointSize, Scalar(0, 0, 0), -1, LINE_8);
}
	for (int i = 0; i < Bezier_WayPoint.size(); i++)		//画出经过点
	{
		circle(canvas1, Point(Bezier_WayPoint[i][0], Bezier_WayPoint[i][1]), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}

	for (int i = 0; i < BezierCurve_Point.size(); i++)
	{
		circle(canvas1, Point(BezierCurve_Point[i][0], BezierCurve_Point[i][1]), PointSize, Scalar(0, 255, 0), -1, LINE_8);
	}

	imshow(canvas1_name, canvas1);
	setMouseCallback(canvas1_name, OnMouseAction, &canvas1);	//检测鼠标按键

#endif
}

//
//int main(void)
//{	
//	int num = lenofarray(ControlPoint);	//计算障碍个数
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
	//clear_canvas(canvas1);	//初始化为空白画布
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





