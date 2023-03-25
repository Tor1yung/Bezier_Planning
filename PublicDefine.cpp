#include "PublicDefine.h"

using namespace std;


void VectorElementRemove(vector<vector<double>>*Vector, int i)
{
	/* 二维vector删除行元素(单个点坐标) */
	Vector->erase(Vector->begin()+i);
}




/*****变量定义*****/
	
vector<vector<double>>WayPoint;						//经过点
vector<vector<int>>Bezier_WayPoint;					//经过点
vector<vector<double>>BezierCurve_Point;			//计算出来的贝塞尔曲线


/*****变量定义*****/