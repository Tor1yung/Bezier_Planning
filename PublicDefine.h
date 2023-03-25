#include<vector>


using namespace std;


/*****宏定义*****/

#define PI	3.1415

//计算数组元素个数
#define lenofarray(a)	(sizeof(a) / sizeof(a[0][0]) / 2)	

//删除数组某个位置的元素
#define ArrayElementRemove(Array, index, NumOfArray)			 \
{														  		 \
	for (int indexx = 0; indexx < NumOfArray; indexx++)			 \
	{													  		 \
		if (indexx == index)	/*判断是否该为位置*/			 \
		{														 \
			/*该位置后面元素向前移动一位*/						 \
			for (int ii = i; ii < NumOfArray - 1; ii++)	  	     \
			{											  		 \
				Array[ii][0] = Array[ii + 1][0];				 \
				Array[ii][1] = Array[ii + 1][1];				 \
			}											  		 \
			NumOfArray = NumOfArray -1;	/*数组元素个数减一*/	 \
			break;												 \
		}												  		 \
	}													  		 \
}	

/*****宏定义*****/




/*****变量声明*****/

extern vector<vector<double>>WayPoint;
extern vector<vector<int>>Bezier_WayPoint;					
extern vector<vector<double>>BezierCurve_Point;
/*****变量声明*****/




/*****函数声明*****/

void VectorElementRemove(vector<vector<double>>*Vector, int i);

/*****函数声明*****/
