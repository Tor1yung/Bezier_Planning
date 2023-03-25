
//#ifdef _DEBUG
//#pragma comment(lib,".\\OsqpEigend.lib")
//#else
//#pragma comment(lib,"..\\release\\LedCtrlBoard.lib")
//#endif

#include<cmath>
#include<iostream>
#include<Eigen/Dense>
#include <OsqpEigen/Data.hpp>
#include <OsqpEigen/Settings.hpp>
#include <OsqpEigen/Solver.hpp>
#include "Bezier.h"
#include "PublicDefine.h"

//using namespace std;
using std::vector;	

double Bezier_Point[2] = { 0,0 };			//计算出来的贝塞尔曲线点坐标
double TimeScale[10];					//每段贝塞尔曲线的时间变换尺度                                                                                                         






int Count_Factorial(int N)
{
	/* 计算阶乘N! */
	int T=1;

	if (N == 0)
	{
		return 1;
	}
	else
	{
		for (int n = 1; n <= N; n++)
		{
			T *= n;
		}
		return T;
	}
}


int Count_Bern_Coef(int n, int i)
{
	/* 计算伯恩斯坦系数n!/i!(n-i)! */

	int T = 1;

	if (i == 0)
	{
		return 1;
	}
	else
	{
		T = Count_Factorial(n) / (Count_Factorial(i)*Count_Factorial(n - i));
		return T;
	}
}


double Count_Bezier(double (*Obstacle_Point)[2],int Obstacle_num,double t)
{
	/* 计算某时刻贝塞尔曲线坐标点 */

	if (Obstacle_num >= 2)
	{
		Bezier_Point[0] = Bezier_Point[1] = 0;
		for (int count = 0; count < Obstacle_num; count++)
		{
			Bezier_Point[0] += *(*(Obstacle_Point + count)) * Count_Bern_Coef(Obstacle_num-1, count)*pow(t, count)*pow(1 - t, Obstacle_num -1 - count);	//x坐标
			Bezier_Point[1] += *(*(Obstacle_Point + count)+1) * Count_Bern_Coef(Obstacle_num-1, count)*pow(t, count)*pow(1 - t, Obstacle_num -1 - count);	//y坐标
		}
	}
	else
	{
		//给出提示：点不够
	}
	return 0;
}




void Bezier::Waypoints_Update()
{
	/* 经过点设置 */
	float WP[8][4];

	//    X轴				Y轴                速度          角度
	WP[0][0] = 2.5;   WP[0][1] = 1.5;	 WP[0][2] = 0.05;  WP[0][3] = -PI/4;           	//第个1点
	WP[1][0] = 1;   WP[1][1] = 3.5;		 WP[1][2] = 0.05;  WP[1][3] = PI/5;           //第个2点
	WP[2][0] = 4.25;   WP[2][1] = 6.25;	 WP[2][2] = 0.05;  WP[2][3] = 0;           	//第个3点
	WP[3][0] = 5.5;   WP[3][1] = 5;		 WP[3][2] = 0.05;  WP[3][3] = -110 / 180 * PI;           //第个4点
	WP[4][0] = 4.75;   WP[4][1] = 2.75;	 WP[4][2] = 0.05;  WP[4][3] = -PI/4;           //第个5点
	WP[5][0] = 7.75;   WP[5][1] = 0.25;	 WP[5][2] = 0.05;  WP[5][3] = 0;           	//第个6点
	WP[6][0] = 8.75;   WP[6][1] = 2.5;	 WP[6][2] = 0.05;  WP[6][3] = PI/2;           	//第个7点
	WP[7][0] = 7.5;   WP[7][1] = 4.25;	 WP[7][2] = 0.05;  WP[7][3] = PI;           	//第个8点


	for (size_t i = 0; i < 8; i++)
	{
		vector<float> points = {WP[i][0],WP[i][1],WP[i][2],WP[i][3] };
		Waypoints.push_back(points);		
	}



}


void Bezier::Bezier_Init()
{
	/* 初始化Bezier曲线优化参数 */

	StartPoint_x = 0;
	StartPoint_y = 0;
	StartPoint_angle = 0;
	StartPoint_speed = 0;

	Waypoints_Update();
	Count_Sj();							//sj初始化
	Hessian_Init();						//H矩阵初始化
	LinearMatrix_Init();				//A矩阵初始化
	BoundaryMatrix_Init();				//B矩阵初始化
}


void Bezier::Count_Sj()
{
	/* 计算时间刻度sj */
	sj.resize(SegNum);
	sj.fill(0);
	float speed;

	for (int i = 0; i < SegNum; i++)
	{
		speed = max(abs(Waypoints[i][2] * sin(Waypoints[i][3])), abs(Waypoints[i][2] * cos(Waypoints[i][3])));
		if (i == 0)				//第一段为第一个waypoint和起始点
		{
			
			sj[i] = abs((Waypoints[i][1] - StartPoint_y) / speed);
			//sj[i] = 1;
		}
		else					//其余的为waypoint之间的段
		{
			sj[i] = abs((Waypoints[i][1] - Waypoints[i-1][1]) / speed);
			//sj[i] = 1;
		}
	}
	cout<<"sj:"<<sj<<endl;
}

void Bezier::Hessian_Init()
{
	/* H矩阵初始化 */

	Eigen::MatrixXd F_j(CtrlPNum, CtrlPNum);
	Eigen::MatrixXd F = Eigen::MatrixXd::Zero(CtrlPNum*SegNum*2, CtrlPNum*SegNum*2);	//*2是因为同时考虑x和y
	Eigen::MatrixXd Q_j = Eigen::MatrixXd::Zero(CtrlPNum, CtrlPNum);
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(CtrlPNum*SegNum*2, CtrlPNum*SegNum*2);
	Eigen::MatrixXd Hessian_j;

	//计算F_j矩阵
	F_j(0, 0) = 1; F_j(0, 1) = 0; F_j(0, 2) = 0; F_j(0, 3) = 0;
	F_j(1, 0) = -3; F_j(1, 1) = 3; F_j(1, 2) = 0; F_j(1, 3) = 0;
	F_j(2, 0) = 3; F_j(2, 1) = -6; F_j(2, 2) = 3; F_j(2, 3) = 0;
	F_j(3, 0) = -1; F_j(3, 1) = 3; F_j(3, 2) = -3; F_j(3, 3) = 1;

	//F_j插入F矩阵，构建元素为子矩阵的广义对角矩阵
	for (int i = 0; i < SegNum*2; i++)
	{
		F.block<4, 4>(i * 4, i * 4) = F_j;
	}

	//计算Q矩阵
	for (int i = 0; i < SegNum*2; i++)
	{
		if (i < SegNum)
		{
			Q_j(3, 3) = 36 / pow(sj[i],3);			//计算Q_j矩阵
		}
		else
		{
			Q_j(3, 3) = 36 / pow(sj[i - SegNum],3);			//计算Q_j矩阵
		}
		//Q_j(3, 3) = 36;			//计算Q_j矩阵
		Q.block<4, 4>(i * 4, i * 4) = Q_j;
		//cout << Q_j << endl;
	}
	

	//计算H矩阵
	Hessian_j = F.transpose()*Q*F;			//计算乘积
	Hessian = Hessian_j.sparseView();		//转化为稀疏矩阵
	//cout<<Hessian_j<<endl;
	


	//cout <<Hessian<<endl;
	//system("pause");
}

void Bezier::LinearMatrix_Init()
{
	/* A矩阵/约束系数初始化 */

	int count = 0;										//记录更新位置
	int count_copy = count;								//临时记录更新位置
	LinearMatrix.resize(ConstrainNum, CtrlPNum*SegNum*2);	//设置大小(约束数*变量数)

	//经过点及安全约束
		//X(32)
	for (size_t i = 0; i < CtrlPNum*SegNum; i++)
	{
		LinearMatrix.insert(i, i) = 1;
		count++;
	}
		//Y(48)
	LinearMatrix.insert(count++, 0 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第0组
	LinearMatrix.insert(count++, 0 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;	
	LinearMatrix.insert(count, 0 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 0 * CtrlPNum + 1) = -1 / 1.5;
	LinearMatrix.insert(count++, 0 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 0 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 0 * CtrlPNum + 2) = -1 / 1.5;
	LinearMatrix.insert(count++, 0 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;		


	LinearMatrix.insert(count++, 1 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第1组		
	LinearMatrix.insert(count++, 1 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;	
	LinearMatrix.insert(count, 1 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 1 * CtrlPNum + 1) = -1 * (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0]);
	LinearMatrix.insert(count++, 1 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 1 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 1 * CtrlPNum + 2) = -1 * (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0]);
	LinearMatrix.insert(count++, 1 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	LinearMatrix.insert(count++, 2 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第2组		
	LinearMatrix.insert(count, 2 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;		
	LinearMatrix.insert(count++, 2 * CtrlPNum + 1) = -1 * (5 - Waypoints[1][1]) / (2 - Waypoints[1][0]);
	LinearMatrix.insert(count, 2 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 2 * CtrlPNum + 1) = -1 * (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0]);
	LinearMatrix.insert(count, 2 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 2 * CtrlPNum + 2) = -1 * (5 - Waypoints[1][1]) / (2 - Waypoints[1][0]);
	LinearMatrix.insert(count, 2 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 2 * CtrlPNum + 2) = -1 * (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0]);
	LinearMatrix.insert(count++, 2 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;	

	LinearMatrix.insert(count++, 3 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第3组		
	LinearMatrix.insert(count++, 3 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;	
	LinearMatrix.insert(count, 3 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 3 * CtrlPNum + 1) = -1 * (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0]);
	LinearMatrix.insert(count++, 3 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 3 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 3 * CtrlPNum + 2) = -1 * (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0]);
	LinearMatrix.insert(count++, 3 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	LinearMatrix.insert(count++, 4 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第4组		
	LinearMatrix.insert(count++, 4 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;	
	LinearMatrix.insert(count, 4 * CtrlPNum + 1) = -1 * (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0]);
	LinearMatrix.insert(count++, 4 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 4 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 4 * CtrlPNum + 2) = -1 * (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0]);
	LinearMatrix.insert(count++, 4 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 4 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	LinearMatrix.insert(count++, 5 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第5组		
	LinearMatrix.insert(count, 5 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;		
	LinearMatrix.insert(count++, 5 * CtrlPNum + 1) = -1 * (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0]);
	LinearMatrix.insert(count, 5 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 5 * CtrlPNum + 1) = -1 * (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0]);
	LinearMatrix.insert(count, 5 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 5 * CtrlPNum + 2) = -1 * (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0]);
	LinearMatrix.insert(count, 5 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 5 * CtrlPNum + 2) = -1 * (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0]);
	LinearMatrix.insert(count++, 5 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	LinearMatrix.insert(count++, 6 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第6组		
	LinearMatrix.insert(count++, 6 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 6 * CtrlPNum + 1) = -1 * (1 - Waypoints[5][1]) / (8 - Waypoints[5][0]);
	LinearMatrix.insert(count++, 6 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 6 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 6 * CtrlPNum + 2) = -1 * (1 - Waypoints[5][1]) / (8 - Waypoints[5][0]);
	LinearMatrix.insert(count++, 6 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 6 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	LinearMatrix.insert(count++, 7 * CtrlPNum + 0 + CtrlPNum * SegNum) = 1;		//第7组		
	LinearMatrix.insert(count++, 7 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 7 * CtrlPNum + 1) = -1 * (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0]);
	LinearMatrix.insert(count++, 7 * CtrlPNum + 1 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 7 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count, 7 * CtrlPNum + 2) = -1 * (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0]);
	LinearMatrix.insert(count++, 7 * CtrlPNum + 2 + CtrlPNum * SegNum) = 1;
	LinearMatrix.insert(count++, 7 * CtrlPNum + 3 + CtrlPNum * SegNum) = 1;

	//连续性约束
		//Xv(7)
	for (size_t i = 0; i < 7; i++)
	{
		LinearMatrix.insert(count, i * CtrlPNum + 2) = -1;		//第i-i+1段	
		LinearMatrix.insert(count, i * CtrlPNum + 3) = 1;		//第i-i+1段	
		LinearMatrix.insert(count, i * CtrlPNum + 4) = 1;		//第i-i+1段	
		LinearMatrix.insert(count++, i * CtrlPNum + 5) = -1;	//第i-i+1段	
	}
		//Yv(7)
	for (size_t i = 0; i < 7; i++)
	{
		LinearMatrix.insert(count, i * CtrlPNum + 2+ CtrlPNum * SegNum) = -1;		//第i-i+1段	
		LinearMatrix.insert(count, i * CtrlPNum + 3+ CtrlPNum * SegNum) = 1;		//第i-i+1段	
		LinearMatrix.insert(count, i * CtrlPNum + 4+ CtrlPNum * SegNum) = 1;		//第i-i+1段	
		LinearMatrix.insert(count++, i * CtrlPNum + 5 + CtrlPNum * SegNum) = -1;	//第i-i+1段	
	}


		


	////连续性约束
	//	//位置
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3) = sj[i];			//此处i为j，即前一段
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4) = -sj[i+1];		//此处i+1为j+1，即后一段
	//	count += 1;
	//}
	//count_copy = count;
	//	//速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 2) = -1;			//终点前一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3) = 1;			//终点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4) = 1;			//起点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 5) = -1;			//起点后一个点
	//	count += 1;
	//}
	//count_copy = count;
	//	//加速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 1) = 1 / sj[i];			//终点前两个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 2) = -2 / sj[i];		//终点前一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3) = 1 / sj[i];			//终点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4) = -1 / sj[i + 1];	//起点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 5) = 2 / sj[i + 1];		//起点后一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 6) = -1 / sj[i + 1];	//起点后两个点
	//	count += 1;
	//}
	//count_copy = count;


	////起始点/经过点约束(Y)
	//for (size_t i = 0; i < SegNum*2; i += 2)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 2 + CtrlPNum*SegNum) = 1;
	//	LinearMatrix.insert(i + 1 + count_copy, i * 2 + 3 + CtrlPNum * SegNum) = 1;
	//	count += 2;
	//}
	//count_copy = count;

	////连续性约束(Y)
	//	//位置
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3 + CtrlPNum * SegNum) = sj[i];			//此处i为j，即前一段
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4 + CtrlPNum * SegNum) = -sj[i + 1];		//此处i+1为j+1，即后一段
	//	count += 1;
	//}
	//count_copy = count;
	////速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 2 + CtrlPNum*SegNum) = -1;			//终点前一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3 + CtrlPNum*SegNum) = 1;			//终点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4 + CtrlPNum*SegNum) = 1;			//起点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 5 + CtrlPNum*SegNum) = -1;			//起点后一个点
	//	count += 1;
	//}
	//count_copy = count;
	////加速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	LinearMatrix.insert(i + count_copy, i * 4 + 1 + CtrlPNum*SegNum) = 1 / sj[i];			//终点前两个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 2 + CtrlPNum*SegNum) = -2 / sj[i];		//终点前一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 3 + CtrlPNum*SegNum) = 1 / sj[i];			//终点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 4 + CtrlPNum*SegNum) = -1 / sj[i + 1];	//起点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 5 + CtrlPNum*SegNum) = 2 / sj[i + 1];		//起点后一个点
	//	LinearMatrix.insert(i + count_copy, i * 4 + 6 + CtrlPNum*SegNum) = -1 / sj[i + 1];	//起点后两个点
	//	count += 1;
	//}
	//count_copy = count;

	cout<<LinearMatrix<<endl;
	system("pause");
}



void Bezier::BoundaryMatrix_Init()
{
	/* B矩阵/约束边界初始化 */

	int countU = 0;
	int countL = 0;
	BoundLower.resize(ConstrainNum);
	BoundUpper.resize(ConstrainNum);
	XBoundLower.resize(ConstrainNum_X);
	XBoundUpper.resize(ConstrainNum_X);
	YBoundLower.resize(ConstrainNum_Y);
	YBoundUpper.resize(ConstrainNum_Y);
	XBoundLower.fill(0);
	XBoundUpper.fill(0);
	YBoundLower.fill(0);
	YBoundUpper.fill(0);
	XBoundLower_v.resize(ConstrainNum_X_v);
	XBoundUpper_v.resize(ConstrainNum_X_v);
	YBoundLower_v.resize(ConstrainNum_Y_v);
	YBoundUpper_v.resize(ConstrainNum_Y_v);
	XBoundLower_v.fill(0);
	XBoundUpper_v.fill(0);
	YBoundLower_v.fill(0);
	YBoundUpper_v.fill(0);


	//安全约束
	//XBoundLower << 0, 0, 0, 0, Waypoints[1][0]/sj[2], Waypoints[1][0] / sj[2], Waypoints[2][0] / sj[3], Waypoints[2][0] / sj[3], Waypoints[3][0] / sj[4], Waypoints[3][0] / sj[4], Waypoints[4][0] / sj[5], Waypoints[4][0] / sj[5], Waypoints[5][0] / sj[6], Waypoints[5][0] / sj[6], Waypoints[6][0] / sj[7], Waypoints[6][0] / sj[7];
	//XBoundUpper << 3.5 / sj[0], 3.5 / sj[0], Waypoints[0][0] / sj[1], Waypoints[0][0] / sj[1], 3.5 / sj[2], 3.5 / sj[2], 6.5 / sj[3], 6.5 / sj[3], Waypoints[3][0] / sj[4], Waypoints[3][0] / sj[4], 7.5 / sj[5], 7.5 / sj[5], 9.5 / sj[6], 9.5 / sj[6], 9.5 / sj[7], 9.5 / sj[7];

	//XBoundLower[countL++] = 0; XBoundLower[countL++] = 0; XBoundLower[countL++] = 0; XBoundLower[countL++] = 0; XBoundLower[countL++] = Waypoints[1][0] / sj[2]; XBoundLower[countL++] = Waypoints[1][0] / sj[2]; XBoundLower[countL++] = Waypoints[2][0] / sj[3]; XBoundLower[countL++] = Waypoints[2][0] / sj[3]; XBoundLower[countL++] = Waypoints[3][0] / sj[4]; XBoundLower[countL++] = Waypoints[3][0] / sj[4]; XBoundLower[countL++] = Waypoints[4][0] / sj[5]; XBoundLower[countL++] = Waypoints[4][0] / sj[5]; XBoundLower[countL++] = Waypoints[5][0] / sj[6]; XBoundLower[countL++] = Waypoints[5][0] / sj[6]; XBoundLower[countL++] = Waypoints[6][0] / sj[7]; XBoundLower[countL++] = Waypoints[6][0] / sj[7];
	//XBoundUpper[countU++] = 3.5 / sj[0]; XBoundUpper[countU++] =  3.5 / sj[0]; XBoundUpper[countU++] = Waypoints[0][0] / sj[1]; XBoundUpper[countU++] = Waypoints[0][0] / sj[1]; XBoundUpper[countU++] = 3.5 / sj[2]; XBoundUpper[countU++] = 3.5 / sj[2]; XBoundUpper[countU++] = 6.5 / sj[3]; XBoundUpper[countU++] = 6.5 / sj[3]; XBoundUpper[countU++] = Waypoints[3][0] / sj[4]; XBoundUpper[countU++] = Waypoints[3][0] / sj[4]; XBoundUpper[countU++] = 7.5 / sj[5]; XBoundUpper[countU++] = 7.5 / sj[5]; XBoundUpper[countU++] = 9.5 / sj[6]; XBoundUpper[countU++] = 9.5 / sj[6]; XBoundUpper[countU++] = 9.5 / sj[7]; XBoundUpper[countU++] = 9.5 / sj[7];

	XBoundLower <<          0,0,0, Waypoints[0][0] / sj[0],
							Waypoints[0][0] / sj[1],0,0, Waypoints[1][0] / sj[1],
							Waypoints[1][0] / sj[2], Waypoints[1][0] / sj[2], Waypoints[1][0] / sj[2], Waypoints[2][0] / sj[2], 
							Waypoints[2][0] / sj[3], Waypoints[2][0] / sj[3],Waypoints[2][0] / sj[3], Waypoints[3][0] / sj[3], 
							Waypoints[3][0] / sj[4], 4 / sj[4],4 / sj[4], Waypoints[4][0] / sj[4],
							Waypoints[4][0] / sj[5], Waypoints[4][0] / sj[5], Waypoints[4][0] / sj[5], Waypoints[5][0] / sj[5],
							Waypoints[5][0] / sj[6], Waypoints[5][0] / sj[6], Waypoints[5][0] / sj[6], Waypoints[6][0] / sj[6],
							Waypoints[6][0] / sj[7], Waypoints[7][0] / sj[7], Waypoints[7][0] / sj[7], Waypoints[7][0] / sj[7]
		;
	XBoundUpper <<          0, 3.5 / sj[0], 3.5 / sj[0], Waypoints[0][0] / sj[0],
							Waypoints[0][0] / sj[1], Waypoints[0][0] / sj[1], Waypoints[0][0] / sj[1], Waypoints[1][0] / sj[1],
							Waypoints[1][0] / sj[2], 3.5 / sj[2], 3.5 / sj[2], Waypoints[2][0] / sj[2],
							Waypoints[2][0] / sj[3], 6.5 / sj[3],6.5 / sj[3], Waypoints[3][0] / sj[3],
							Waypoints[3][0] / sj[4], Waypoints[3][0] / sj[4], Waypoints[3][0] / sj[4], Waypoints[4][0] / sj[4],
							Waypoints[4][0] / sj[5],7.5 / sj[5],7.5 / sj[5], Waypoints[5][0] / sj[5],
							Waypoints[5][0] / sj[6], 9.5/ sj[6], 9.5/ sj[6], Waypoints[6][0] / sj[6],
							Waypoints[6][0] / sj[7], 9.5/ sj[7], 9.5/ sj[7], Waypoints[7][0] / sj[7]
		;
	XBoundLower_v <<        0,0,0,0,0,0,0;
	XBoundUpper_v <<		0,0,0,0,0,0,0;

	////起始点/经过点约束
	//for (size_t i = 0; i < SegNum; i++)
	//{
	//	for (size_t j = 0; j < 2; j++)
	//	{
	//		if (i == 0)	//第一段
	//		{
	//			//XBoundLower << StartPoint_x/sj[i];
	//			//XBoundUpper << StartPoint_x / sj[i];
	//			//XBoundLower << Waypoints[0][0]/sj[i];
	//			//XBoundUpper << Waypoints[0][0]/sj[i];
	//			XBoundLower[countL++] = StartPoint_x / sj[i];
	//			XBoundUpper[countU++] = StartPoint_x / sj[i];
	//			XBoundLower[countL++] = Waypoints[0][0] / sj[i];
	//			XBoundUpper[countU++] = Waypoints[0][0] / sj[i];
	//			break;
	//		}
	//		XBoundLower[countL++] = Waypoints[i + j - 1][0] / sj[i];
	//		XBoundUpper[countU++] = Waypoints[i + j - 1][0]/sj[i];
	//	}
	//}

	////连续性约束
	//	//位置
	//for (size_t i = 0; i < SegNum-1; i++)
	//{
	//	XBoundLower[countL++] = 0;
	//	XBoundUpper[countU++] = 0;
	//}
	//	//速度
	//for (size_t i = 0; i < SegNum-1; i++)
	//{
	//	XBoundLower[countL++] = 0;
	//	XBoundUpper[countU++] = 0;
	//}
	//	//加速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	XBoundLower[countL++] = 0;
	//	XBoundUpper[countU++] = 0;
	//}

	countU = 0;
	countL = 0;

	//安全性约束(Y)
	//YBoundLower << -0.5, -OsqpEigen::INFTY,- 0.5, -OsqpEigen::INFTY;	//第0组
	//YBoundUpper << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0;
	YBoundLower[countL++] = 0; YBoundLower[countL++] = -0.5 / sj[0]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = -0.5 / sj[0]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = Waypoints[0][1] / sj[0];	//第0组
	YBoundUpper[countU++] = 0; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = 0; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = 0; YBoundUpper[countU++] = Waypoints[0][1]/ sj[0];

	//YBoundLower << Waypoints[0][1], -OsqpEigen::INFTY, Waypoints[0][1], -OsqpEigen::INFTY;	//第1组
	//YBoundUpper << OsqpEigen::INFTY, Waypoints[0][1] - (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0]), OsqpEigen::INFTY, Waypoints[0][1] - (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0]);
	YBoundLower[countL++] = Waypoints[0][1]/ sj[1]; YBoundLower[countL++] = Waypoints[0][1] / sj[1]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = Waypoints[0][1] / sj[1]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = Waypoints[1][1] / sj[1];	//第1组
	YBoundUpper[countU++] = Waypoints[0][1] / sj[1]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = (Waypoints[0][1] - (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0])*Waypoints[0][0]) / sj[1]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = (Waypoints[0][1] - (3 - Waypoints[0][1]) / (1.5 - Waypoints[0][0])*Waypoints[0][0]) / sj[1]; YBoundUpper[countU++] = Waypoints[1][1] / sj[1];

	//YBoundLower << Waypoints[1][1]-(5- Waypoints[1][1])/(2-Waypoints[1][0])*Waypoints[1][0], -OsqpEigen::INFTY, Waypoints[1][1] - (5 - Waypoints[1][1]) / (2 - Waypoints[1][0])*Waypoints[1][0], -OsqpEigen::INFTY;	//第2组
	//YBoundUpper << OsqpEigen::INFTY, Waypoints[1][1] - (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0])*Waypoints[1][0], OsqpEigen::INFTY, Waypoints[1][1] - (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0])*Waypoints[1][0];
	YBoundLower[countL++] = Waypoints[1][1] / sj[2]; YBoundLower[countL++] = -OsqpEigen::INFTY;  YBoundLower[countL++] = (Waypoints[1][1] - (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0])*Waypoints[1][0]) / sj[2];  YBoundLower[countL++] = -OsqpEigen::INFTY;  YBoundLower[countL++] = (Waypoints[1][1] - (5.5 - Waypoints[1][1]) / (3.5 - Waypoints[1][0])*Waypoints[1][0]) / sj[2];   YBoundLower[countL++] = Waypoints[2][1] / sj[2];   //第2组
	YBoundUpper[countU++] = Waypoints[1][1] / sj[2]; YBoundUpper[countU++] = (Waypoints[1][1] - (5 - Waypoints[1][1]) / (2 - Waypoints[1][0])*Waypoints[1][0]) / sj[2]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = (Waypoints[1][1] - (5 - Waypoints[1][1]) / (2 - Waypoints[1][0])*Waypoints[1][0]) / sj[2]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = Waypoints[2][1]/ sj[2];

	//YBoundLower <<  -OsqpEigen::INFTY, Waypoints[2][1] - (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0])*Waypoints[2][0], -OsqpEigen::INFTY, Waypoints[2][1] - (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0])*Waypoints[2][0];	//第3组
	//YBoundUpper << Waypoints[2][1], OsqpEigen::INFTY, Waypoints[2][1], OsqpEigen::INFTY;
	YBoundLower[countL++] = Waypoints[2][1] / sj[3]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[2][1] - (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0])*Waypoints[2][0]) / sj[2]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[2][1] - (5.5 - Waypoints[2][1]) / (5 - Waypoints[2][0])*Waypoints[2][0]) / sj[3]; YBoundLower[countL++] = Waypoints[3][1] / sj[3];	//第3组
	YBoundUpper[countU++] = Waypoints[2][1] / sj[3]; YBoundUpper[countU++] = Waypoints[2][1] / sj[3]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = Waypoints[2][1] / sj[3]; YBoundUpper[countU++] = OsqpEigen::INFTY;YBoundUpper[countU++] = Waypoints[3][1] / sj[3];

	//YBoundLower << Waypoints[3][1] - (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0])*Waypoints[3][0], -OsqpEigen::INFTY, Waypoints[3][1] - (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0])*Waypoints[3][0], -OsqpEigen::INFTY;	//第4组
	//YBoundUpper << OsqpEigen::INFTY, Waypoints[3][1], OsqpEigen::INFTY, Waypoints[3][1];
	YBoundLower[countL++] = Waypoints[3][1] / sj[4]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[3][1] - (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0])*Waypoints[3][0]) / sj[4]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[3][1] - (3.5 - Waypoints[3][1]) / (5 - Waypoints[3][0])*Waypoints[3][0])/ sj[4]; 	YBoundLower[countL++] = Waypoints[4][1] / sj[4];//第4组
	YBoundUpper[countU++] = Waypoints[3][1] / sj[4]; YBoundUpper[countU++] = Waypoints[3][1] / sj[4]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = Waypoints[3][1] / sj[4]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = Waypoints[4][1] / sj[4];

	//YBoundLower << Waypoints[4][1] - (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0])*Waypoints[4][0], -OsqpEigen::INFTY, Waypoints[4][1] - (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0])*Waypoints[4][0], -OsqpEigen::INFTY;	//第5组
	//YBoundUpper << OsqpEigen::INFTY, Waypoints[4][1] - (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0])*Waypoints[4][0], OsqpEigen::INFTY, Waypoints[4][1] - (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0])*Waypoints[4][0];
	YBoundLower[countL++] = Waypoints[4][1] / sj[5]; YBoundLower[countL++] = (Waypoints[4][1] - (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0])*Waypoints[4][0]) / sj[5]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[4][1] - (1.5 - Waypoints[4][1]) / (6 - Waypoints[4][0])*Waypoints[4][0]) / sj[5]; YBoundLower[countL++] = -OsqpEigen::INFTY;	YBoundLower[countL++] = Waypoints[5][1] / sj[5];//第5组
	YBoundUpper[countU++] = Waypoints[4][1] / sj[5]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = (Waypoints[4][1] - (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0])*Waypoints[4][0]) / sj[5]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] =(Waypoints[4][1] - (1 - Waypoints[4][1]) / (7.5 - Waypoints[4][0])*Waypoints[4][0]) / sj[5];		YBoundUpper[countU++] = Waypoints[5][1] / sj[5];
	
	//YBoundLower << Waypoints[5][1], -OsqpEigen::INFTY, Waypoints[5][1], -OsqpEigen::INFTY;	//第6组
	//YBoundUpper << OsqpEigen::INFTY, Waypoints[5][1] - (1 - Waypoints[5][1]) / (8 - Waypoints[5][0])*Waypoints[5][0], OsqpEigen::INFTY, Waypoints[5][1] - (1 - Waypoints[5][1]) / (8 - Waypoints[5][0])*Waypoints[5][0];
	YBoundLower[countL++] = Waypoints[5][1] / sj[6]; YBoundLower[countL++] = Waypoints[5][1] / sj[6]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = Waypoints[5][1] / sj[6]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = Waypoints[6][1] / sj[6];	//第6组																													  
	YBoundUpper[countU++] = Waypoints[5][1] / sj[6]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = (Waypoints[5][1] - (1 - Waypoints[5][1]) / (8 - Waypoints[5][0])*Waypoints[5][0])/ sj[6]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] =(Waypoints[5][1] - (1 - Waypoints[5][1]) / (8 - Waypoints[5][0])*Waypoints[5][0]) / sj[6];	 YBoundUpper[countU++] = Waypoints[6][1] / sj[6];

	//YBoundLower << Waypoints[6][1] - (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0])*Waypoints[6][0], -OsqpEigen::INFTY, Waypoints[6][1] - (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0])*Waypoints[6][0], -OsqpEigen::INFTY;	//第7组
	//YBoundUpper << OsqpEigen::INFTY, 5, OsqpEigen::INFTY, 5;
	YBoundLower[countL++] = Waypoints[6][1] / sj[7]; YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[6][1] - (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0])*Waypoints[6][0]) / sj[7];  YBoundLower[countL++] = -OsqpEigen::INFTY; YBoundLower[countL++] = (Waypoints[6][1] - (3.5 - Waypoints[6][1]) / (8 - Waypoints[6][0])*Waypoints[6][0]) / sj[7];	YBoundLower[countL++] = Waypoints[7][1] / sj[7]; //第7组
	YBoundUpper[countU++] = Waypoints[6][1] / sj[7];  YBoundUpper[countU++] = 5 / sj[7]; YBoundUpper[countU++] = OsqpEigen::INFTY; YBoundUpper[countU++] = 5 / sj[7];	 YBoundUpper[countU++] = OsqpEigen::INFTY;	YBoundUpper[countU++] = Waypoints[7][1] / sj[7];

	countU = 0;
	countL = 0;
	  
	YBoundLower_v[countL++] = 0, 0, 0, 0, 0, 0, 0 ;
	YBoundUpper_v[countU++] = 0, 0, 0, 0, 0, 0, 0 ;

	//起始点/经过点约束(Y)
	//for (size_t i = 0; i < SegNum; i++)
	//{
	//	for (size_t j = 0; j < 2; j++)
	//	{
	//		if (i == 0)	//第一段
	//		{
	//			YBoundLower[countL++] = StartPoint_y / sj[i];
	//			YBoundUpper[countU++] = StartPoint_y / sj[i];
	//			YBoundLower[countL++] = Waypoints[0][1] / sj[i];
	//			YBoundUpper[countU++] = Waypoints[0][1] / sj[i];
	//			break;
	//		}
	//		YBoundLower[countL++] = Waypoints[i + j - 1][1] / sj[i];
	//		YBoundUpper[countU++] = Waypoints[i + j - 1][1] / sj[i];
	//	}
	//}

	////连续性约束(Y)
	//	//位置
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	YBoundLower[countL++] = 0;
	//	YBoundUpper[countU++] = 0;
	//}
	////速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	YBoundLower[countL++] = 0;
	//	YBoundUpper[countU++] = 0;
	//}
	////加速度
	//for (size_t i = 0; i < SegNum - 1; i++)
	//{
	//	YBoundLower[countL++] = 0;
	//	YBoundUpper[countU++] = 0;
	//}

	//合并XY轴边界
	BoundLower << XBoundLower, YBoundLower, XBoundLower_v, YBoundLower_v;
	BoundUpper << XBoundUpper, YBoundUpper, XBoundUpper_v, YBoundUpper_v;
	//BoundLower << 0, 0, 0, 2.5, 2.5, 0, 0, 1, 1, 1, 1, 4.25, 4.25, 4.25, 4.25, 5.5, 5.5, 4, 4, 4.75, 4.75, 4.75, 4.75, 7.75, 7.75, 7.75, 7.75, 8.75, 8.75, 7.5, 7.5, 7.5, 0, -0.5, -OsqpEigen::INFTY, -0.5, -OsqpEigen::INFTY, 1.5, 1.5, 1.5, -OsqpEigen::INFTY, 1.5, -OsqpEigen::INFTY, 3.5, 3.5, -OsqpEigen::INFTY, 2.7, -OsqpEigen::INFTY, 2.7, 6.25, 6.25, -OsqpEigen::INFTY, 10.5, -OsqpEigen::INFTY, 10.5, 5, 5, -OsqpEigen::INFTY, -11.5, -OsqpEigen::INFTY, -11.5, 2.75, 2.75, 7.5, -OsqpEigen::INFTY, 7.5, -OsqpEigen::INFTY, 0.25, 0.25, 0.25, -OsqpEigen::INFTY, 0.25, -OsqpEigen::INFTY, 2.5, 2.5, -OsqpEigen::INFTY, 16.64, -OsqpEigen::INFTY, 16.64, 4.25
	//	, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;    //速度连续约束
	//BoundUpper << 0, 3.5, 3.5, 2.5, 2.5, 2.5, 2.5, 1, 1, 3.5, 3.5, 4.25, 4.25, 6.5, 6.5, 5.5, 5.5, 5.5, 5.5, 4.75, 4.75, 7.5, 7.5, 7.75, 7.75, 9.5, 9.5, 8.75, 8.75, 9.5, 9.5, 7.5, 0, OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, 1.5, 1.5, OsqpEigen::INFTY, 5.25 , OsqpEigen::INFTY, 5.25, 3.5, 3.5, 2, OsqpEigen::INFTY, 2, OsqpEigen::INFTY, 6.25, 6.25, 6.25, OsqpEigen::INFTY, 6.25, OsqpEigen::INFTY, 5, 5, 5, OsqpEigen::INFTY, 5, OsqpEigen::INFTY, 2.75, 2.75, OsqpEigen::INFTY, 5.8, OsqpEigen::INFTY, 5.8, 0.25, 0.25, OsqpEigen::INFTY, -23, OsqpEigen::INFTY, -23, 2.5, 2.5, 5, OsqpEigen::INFTY, 5, OsqpEigen::INFTY, 4.25
	//	, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;        //速度连续约束
	cout << "Bound" << BoundLower << endl;
	system("pause");
}

bool Bezier::Bezier_Solve()
{
	OsqpEigen::Solver BezierSolver;
	vector<float> a;
	BezierSolver.settings()->setWarmStart(true);


	BezierSolver.data()->setNumberOfVariables(CtrlPNum*SegNum*2);													           //A矩阵的列数，即控制点个数
	BezierSolver.data()->setNumberOfConstraints(ConstrainNum);														           //A矩阵的行数，即约束个数
	if (!BezierSolver.data()->setHessianMatrix(Hessian)) { cout << "H矩阵初始化失败" << endl; system("pause"); }				   //H矩阵
	if(!BezierSolver.data()->setGradient(Gradient)){ cout << "f矩阵初始化失败" << endl; system("pause"); }						 //f矩阵
	if(!BezierSolver.data()->setLinearConstraintsMatrix(LinearMatrix) ){ cout << "A矩阵初始化失败" << endl; system("pause"); }	 //A矩阵
	if(!BezierSolver.data()->setLowerBound(BoundLower)) { cout << "LB矩阵初始化失败" << endl; system("pause"); }				   //设置下边界
	if(!BezierSolver.data()->setUpperBound(BoundUpper)) { cout << "UB矩阵初始化失败" << endl; system("pause"); }				   //设置上边界


	if(!BezierSolver.initSolver()) { cout << "初始化失败" << endl; system("pause"); }										   //初始化

	if (BezierSolver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)		
	{
		cout<<"无解"<<endl;
		system("pause");
		return 0;
	}
	else
	{
		//cout << "完成" << endl;
		//system("pause");
	}

	QPSolution = BezierSolver.getSolution();	//取求解结果
	cout << QPSolution << endl;
	system("pause");



}

void Bezier::Count_Bezier_Vector(double t)
{
	/* 计算某时刻贝塞尔曲线坐标点(vectorXd情况) */
	
	int t_pre = 0;		//记录上一个waypoint的时间点
	int seg = 0;		//表示当前在第几段
	if (t >= 0 && t < sj[0])
	{
		seg = 0;
		t_pre = 0;
	}
	else if (t >= sj[0] && t < sj[0] + sj[1])
	{
		seg = 1;
		t_pre = sj[0];
	}
	else if (t >= sj[0]+sj[1] && t < sj[0]+ sj[1]+sj[2])
	{
		seg = 2;
		t_pre = sj[0] + sj[1];
	}
	else if (t >= sj[0]+ sj[1]+sj[2] && t < sj[0]+ sj[1]+ sj[2]+sj[3])
	{
		seg = 3;
		t_pre = sj[0] + sj[1] + sj[2];
	}
	else if (t >= sj[0]+ sj[1]+ sj[2]+sj[3] && t < sj[0]+ sj[1]+ sj[2]+ sj[3]+sj[4])
	{
		seg = 4;
		t_pre = sj[0] + sj[1] + sj[2] + sj[3];
	}
	else if (t >= sj[0]+ sj[1]+ sj[2]+ sj[3]+sj[4] && t < sj[0]+ sj[1]+ sj[2]+ sj[3]+ sj[4]+sj[5])
	{
		seg = 5;
		t_pre = sj[0] + sj[1] + sj[2] + sj[3] + sj[4];
	}
	else if (t >= sj[0]+ sj[1]+ sj[2]+ sj[3]+ sj[4]+sj[5] && t < sj[0] + sj[1] + sj[2] + sj[3] + sj[4] + sj[5]+sj[6])
	{
		seg = 6;
		t_pre = sj[0] + sj[1] + sj[2] + sj[3] + sj[4] + sj[5];
	}
	else
	{
		seg = 7;
		t_pre = sj[0] + sj[1] + sj[2] + sj[3] + sj[4] + sj[5] + sj[6];
	}


	cout << "t:" << t << endl;
	cout << "t:" << (t - t_pre) / sj[seg]<< endl;
	Bezier_Point[0] = Bezier_Point[1] = 0;
	for (size_t j = 0; j < CtrlPNum; j++)
	{
		Bezier_Point[0] += QPSolution[j + seg * CtrlPNum]*sj[seg] * Count_Bern_Coef(CtrlPNum - 1, j)*pow((t-t_pre)/sj[seg] , j)*pow(1 - (t-t_pre)/sj[seg] , CtrlPNum - 1 - j);	//x坐标
		Bezier_Point[1] += QPSolution[j + seg * CtrlPNum + CtrlPNum*SegNum] * sj[seg] * Count_Bern_Coef(CtrlPNum - 1, j)*pow((t-t_pre)/sj[seg] , j)*pow(1 - (t-t_pre)/sj[seg] , CtrlPNum - 1 - j);	//y坐标
		cout <<  "QPSolution[j + seg * CtrlPNum + CtrlPNum*SegNum] = " << QPSolution[j + seg * CtrlPNum + CtrlPNum * SegNum] * sj[seg] << endl;
	}
	vector<double>BP = {Bezier_Point[0] ,Bezier_Point[1]};
	BezierCurve_Point.push_back(BP);	
	cout << "BCpointX = "<< Bezier_Point[0]<< "BCpointY = " << Bezier_Point[1] <<endl;
}

//
//int main()
//{
//	Bezier B;
//	B.Bezier_Init();
//	B.Bezier_Solve();
//	B.Count_Bezier_Vector();
//}
//
