#include<vector>


using namespace std;


/*****�궨��*****/

#define PI	3.1415

//��������Ԫ�ظ���
#define lenofarray(a)	(sizeof(a) / sizeof(a[0][0]) / 2)	

//ɾ������ĳ��λ�õ�Ԫ��
#define ArrayElementRemove(Array, index, NumOfArray)			 \
{														  		 \
	for (int indexx = 0; indexx < NumOfArray; indexx++)			 \
	{													  		 \
		if (indexx == index)	/*�ж��Ƿ��Ϊλ��*/			 \
		{														 \
			/*��λ�ú���Ԫ����ǰ�ƶ�һλ*/						 \
			for (int ii = i; ii < NumOfArray - 1; ii++)	  	     \
			{											  		 \
				Array[ii][0] = Array[ii + 1][0];				 \
				Array[ii][1] = Array[ii + 1][1];				 \
			}											  		 \
			NumOfArray = NumOfArray -1;	/*����Ԫ�ظ�����һ*/	 \
			break;												 \
		}												  		 \
	}													  		 \
}	

/*****�궨��*****/




/*****��������*****/

extern vector<vector<double>>WayPoint;
extern vector<vector<int>>Bezier_WayPoint;					
extern vector<vector<double>>BezierCurve_Point;
/*****��������*****/




/*****��������*****/

void VectorElementRemove(vector<vector<double>>*Vector, int i);

/*****��������*****/
