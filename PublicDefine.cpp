#include "PublicDefine.h"

using namespace std;


void VectorElementRemove(vector<vector<double>>*Vector, int i)
{
	/* ��άvectorɾ����Ԫ��(����������) */
	Vector->erase(Vector->begin()+i);
}




/*****��������*****/
	
vector<vector<double>>WayPoint;						//������
vector<vector<int>>Bezier_WayPoint;					//������
vector<vector<double>>BezierCurve_Point;			//��������ı���������


/*****��������*****/