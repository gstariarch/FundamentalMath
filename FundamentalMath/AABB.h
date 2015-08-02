#ifndef _AABB3_H_
#define _AABB3_H_

#include "vector3.h"

class Matrix4x3;

class AABB3
{
public:
	Vector3 min;
	Vector3 max;

public:

	Vector3 size()
	{
		return max - min;
	}
	float xSize()
	{
		return max.x - min.x;
	}
	float ySize()
	{
		return max.y - min.y;
	}
	float zSize()
	{
		return max.z - min.z;
	}
	Vector3 center() const
	{
		return (min + max)*0.5f;
	}
	
	//提取8个定点中的一个
	Vector3 corner(int i) const;

	//empty the AABB
	void empty();

	//Add the point into AABB3
	void add(const Vector3& p);
	
	//向矩形边界框中添加AABB
	void add(const AABB3& box);
	
	//变换矩形边界框，计算新的AABB
	void setToTransformedBox(const AABB3& box, const Matrix4x3& m);

	//包含、相交性测试
	bool isEmpty() const;
	
	//判断矩形边界框是否包含此点
	bool contains(const Vector3& p) const;

	//返回矩形边界框上的最近点
	Vector3 closestPointTo(const Vector3& p) const;

	//判断是否和球相交
	bool intersectsSphere(const Vector3& center, float radius) const;

	//和参数射线的相交性测试，如果不相交则返回值大于1
	float rayIntersect(const Vector3& rayOrg, const Vector3& rayDelta, Vector3* returnNormal = 0) const;

	//判断矩形边界框在平面的哪一面
	int classfyPlane(const Vector3& n, float d) const;

	//和平面的动态相交性测试
	float intersectPlane(const Vector3& n, float planeD, const Vector3& dir) const;
};

//检测两个AABB的相交性，返回相交部分的AABB
bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect = 0);

//返回运动AABB和静止AABB相交时的参数点，如果不相交则返回值大于1
float intersectMovingAABB(
	const AABB3& stationaryBox,
	const AABB3& movingBox,
	const Vector3& d);

#endif