#ifndef __PAINT_H__
#define __PAINT_H__

#include <vector>

using namespace std;


#define REAL_SCALE		True		//ʹ��ʵ�ʵ�ͼ����



constexpr int HEIGHT = 800;			//������
constexpr int WIDTH = 800;			//������	



void Paint_Bezier();	//���Ʊ���������
void Paint_Pole();		//���Ƽ������˶��켣
void PaintIndoor();		//�������ڵ�ͼ�켣

extern int map_scale;
extern int realmapHEIGHT;
extern int realmapWIDTH;
extern int real2bitSCALE;










#endif // !__PAINT_H__