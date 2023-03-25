#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <Eigen/Eigen>



using std::vector;
/*****宏定义*****/





/*****宏定义*****/



/*****变量声明*****/

extern double Bezier_Point[2];
extern double TimeScale[10];




/*****变量声明*****/




/*****函数声明*****/



//void Count_Bezier_Vector(double t);											//计算某时刻贝塞尔曲线坐标点(vector情况)
double Count_Bezier(double(*Obstacle_Point)[2], int Obstacle_num,double t);		//计算某时刻贝塞尔曲线坐标点
/*****函数声明*****/



/*****类声明*****/

class Bezier
{
public:

	int order = 3;				 //贝塞尔曲线阶数
	int CtrlPNum = order + 1;	 //贝塞尔曲线控制点数
	int SegNum = 8;				 //贝塞尔曲线段数
	int ConstrainNum_X = 32;	 //贝塞尔曲线X约束个数
	int ConstrainNum_Y = 48;	 //贝塞尔曲线Y约束个数
	int ConstrainNum_X_v = 7;	 //贝塞尔曲线X速度约束个数
	int ConstrainNum_Y_v = 7;	 //贝塞尔曲线Y速度约束个数
	int ConstrainNum = ConstrainNum_Y + ConstrainNum_X + 14;	 //贝塞尔曲线约束总个数

	
	Eigen::VectorXd sj;								//时间放缩刻度sj
	Eigen::SparseMatrix<double> Hessian;			//H矩阵
	Eigen::VectorXd Gradient = Eigen::VectorXd::Zero(CtrlPNum*SegNum*2);	//f矩阵
	Eigen::SparseMatrix<double> LinearMatrix;		//A矩阵
	Eigen::VectorXd XBoundLower;					//X轴下边界
	Eigen::VectorXd XBoundUpper;					//X轴上边界
	Eigen::VectorXd YBoundLower;					//Y轴下边界
	Eigen::VectorXd YBoundUpper;					//Y轴上边界
	Eigen::VectorXd XBoundLower_v;					//X轴速度下边界
	Eigen::VectorXd XBoundUpper_v;					//X轴速度上边界
	Eigen::VectorXd YBoundLower_v;					//Y轴速度下边界
	Eigen::VectorXd YBoundUpper_v;					//Y轴速度上边界
	Eigen::VectorXd BoundLower;						//下边界
	Eigen::VectorXd BoundUpper;						//上边界
	Eigen::VectorXd QPSolution;						//优化解

	//起始点各参数
	float StartPoint_x;
	float StartPoint_y;
	float StartPoint_angle;
	float StartPoint_speed;
	//float StartPoint_angle;
	//float StartPoint_speed;

	////终点各参数
	//float EndPoint_x;
	//float EndPoint_y;
	float EndPoint_speed;
	float EndPoint_angle;

	//经过点
	vector<vector<float>>Waypoints;

public:
	void Waypoints_Update();
	void Bezier_Init();						//贝塞尔规划初始化
	bool Bezier_Solve();					//贝塞尔优化求解
	void Count_Sj();						//计算最小时间刻度sj
	void Hessian_Init();					//初始化H矩阵
	void LinearMatrix_Init();				//初始化A矩阵
	void BoundaryMatrix_Init();				//初始化B矩阵
	void Count_Bezier_Vector(double t);		//计算贝塞尔曲线
};


/*****类声明*****/






#endif // !__BEZIER_H__