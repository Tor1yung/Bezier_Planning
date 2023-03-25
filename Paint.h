#ifndef __PAINT_H__
#define __PAINT_H__

#include <vector>

using namespace std;


#define REAL_SCALE		True		//使用实际地图比例



constexpr int HEIGHT = 800;			//画布高
constexpr int WIDTH = 800;			//画布宽	



void Paint_Bezier();	//绘制贝塞尔曲线
void Paint_Pole();		//绘制极坐标运动轨迹
void PaintIndoor();		//绘制室内地图轨迹

extern int map_scale;
extern int realmapHEIGHT;
extern int realmapWIDTH;
extern int real2bitSCALE;










#endif // !__PAINT_H__