#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <Eigen/Eigen>



using std::vector;
/*****�궨��*****/





/*****�궨��*****/



/*****��������*****/

extern double Bezier_Point[2];
extern double TimeScale[10];




/*****��������*****/




/*****��������*****/



//void Count_Bezier_Vector(double t);											//����ĳʱ�̱��������������(vector���)
double Count_Bezier(double(*Obstacle_Point)[2], int Obstacle_num,double t);		//����ĳʱ�̱��������������
/*****��������*****/



/*****������*****/

class Bezier
{
public:

	int order = 3;				 //���������߽���
	int CtrlPNum = order + 1;	 //���������߿��Ƶ���
	int SegNum = 8;				 //���������߶���
	int ConstrainNum_X = 32;	 //����������XԼ������
	int ConstrainNum_Y = 48;	 //����������YԼ������
	int ConstrainNum_X_v = 7;	 //����������X�ٶ�Լ������
	int ConstrainNum_Y_v = 7;	 //����������Y�ٶ�Լ������
	int ConstrainNum = ConstrainNum_Y + ConstrainNum_X + 14;	 //����������Լ���ܸ���

	
	Eigen::VectorXd sj;								//ʱ������̶�sj
	Eigen::SparseMatrix<double> Hessian;			//H����
	Eigen::VectorXd Gradient = Eigen::VectorXd::Zero(CtrlPNum*SegNum*2);	//f����
	Eigen::SparseMatrix<double> LinearMatrix;		//A����
	Eigen::VectorXd XBoundLower;					//X���±߽�
	Eigen::VectorXd XBoundUpper;					//X���ϱ߽�
	Eigen::VectorXd YBoundLower;					//Y���±߽�
	Eigen::VectorXd YBoundUpper;					//Y���ϱ߽�
	Eigen::VectorXd XBoundLower_v;					//X���ٶ��±߽�
	Eigen::VectorXd XBoundUpper_v;					//X���ٶ��ϱ߽�
	Eigen::VectorXd YBoundLower_v;					//Y���ٶ��±߽�
	Eigen::VectorXd YBoundUpper_v;					//Y���ٶ��ϱ߽�
	Eigen::VectorXd BoundLower;						//�±߽�
	Eigen::VectorXd BoundUpper;						//�ϱ߽�
	Eigen::VectorXd QPSolution;						//�Ż���

	//��ʼ�������
	float StartPoint_x;
	float StartPoint_y;
	float StartPoint_angle;
	float StartPoint_speed;
	//float StartPoint_angle;
	//float StartPoint_speed;

	////�յ������
	//float EndPoint_x;
	//float EndPoint_y;
	float EndPoint_speed;
	float EndPoint_angle;

	//������
	vector<vector<float>>Waypoints;

public:
	void Waypoints_Update();
	void Bezier_Init();						//�������滮��ʼ��
	bool Bezier_Solve();					//�������Ż����
	void Count_Sj();						//������Сʱ��̶�sj
	void Hessian_Init();					//��ʼ��H����
	void LinearMatrix_Init();				//��ʼ��A����
	void BoundaryMatrix_Init();				//��ʼ��B����
	void Count_Bezier_Vector(double t);		//���㱴��������
};


/*****������*****/






#endif // !__BEZIER_H__