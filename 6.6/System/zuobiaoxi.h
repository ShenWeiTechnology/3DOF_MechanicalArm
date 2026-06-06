#ifndef __ZUOBIAOXI_H
#define __ZUOBIAOXI_H

// 角度转弧度（math.h 里的三角函数都用弧度）
#define DEG2RAD(x) ((x) * 3.1415926535f / 180.0f)

typedef struct {
    float x;
    float y;
} Point2D;

typedef struct {
    float x;
    float y;
	float z;
} Point3D;




void calc_coordinates(float theta0, float theta1,float theta2,
					  Point2D *pointC,Point3D *pointD);

					  
void hand_to_base(float theta0, float theta1, float theta2,
                  const Point3D *hand_point, Point3D *base_point);
#endif

