#include "AABB.h"
#include <assert.h>
#include <stdlib.h>
#include "Matrix4x3.h"
#include "CommonStuff.h"
#include "vector3.h"


Vector3 AABB3::corner(int i) const
{
	assert(i >= 0);
	assert(i <= 7);

	return Vector3((i & 1) ? max.x : min.x,
				   (i & 2) ? max.y : min.y,
		           (i & 4) ? max.z : min.z);

}

void AABB3::empty()
{
	const float kBigNumber = 1e37f;
	min.x = min.y = min.z = kBigNumber;
	max.x = max.y = max.z = -kBigNumber;

}

void AABB3::add(const Vector3& p)
{
	if (p.x < min.x) min.x = p.x;
	if (p.y < min.y) min.y = p.y;
	if (p.z < min.z) min.z = p.z;

	if (p.x > max.x) max.x = p.x;
	if (p.y > max.y) max.y = p.y;
	if (p.z > max.z) max.z = p.z;
}

void AABB3::add(const AABB3& box)
{
	if (box.min.x < min.x) min.x = box.min.x;
	if (box.min.x > max.x) max.x = box.min.x;

	if (box.min.y < min.y) min.y = box.min.y;
	if (box.min.y > max.y) max.y = box.min.y;

	if (box.min.z < min.z) min.z = box.min.z;
	if (box.min.z > max.z) max.z = box.min.z;
}

void AABB3::setToTransformedBox(const AABB3& box, const Matrix4x3& m)
{
	if (isEmpty()){
		empty();
		return;
	}

	min = max = getTranslation(m);

	if (m.m11 > 0.0f){
		min.x += m.m11*box.min.x; max.x += m.m11*box.max.x;
	}
	else {
		min.x += m.m11*box.max.x; max.x += m.m11*box.min.x;
	}

	if (m.m12 > 0.0f)
	{
		min.y += m.m12*box.min.x; max.y += m.m12*box.max.x;
	}
	else {
		min.y += m.m12*box.max.x; max.y += m.m12*box.min.x;
	}

	if (m.m13 > 0.0f)
	{
		min.z += m.m13*box.min.x; max.z += m.m13*box.max.x;
	}
	else {
		min.z += m.m13*box.max.x; max.z += m.m13*box.min.x;
	}

	if (m.m21 > 0.0f)
	{
		min.x += m.m21*box.min.y; max.x += m.m21*box.max.y;
	}
	else {
		min.x += m.m21*box.max.y; max.x += m.m21*box.min.y;
	}
	if (m.m22 > 0.0f)
	{
		min.y += m.m22*box.min.y; max.y += m.m22*box.max.y;
	}
	else {
		min.y += m.m22*box.max.y; max.y += m.m22*box.min.y;
	}

	if (m.m23 > 0.0f)
	{
		min.z += m.m23*box.min.y; max.z += m.m23*box.max.y;
	}
	else{
		min.z += m.m23*box.max.y; max.z += m.m23*box.min.y;
	}
	
	if (m.m31 > 0.0f)
	{
		min.x += m.m31*box.min.z; max.x += m.m31*box.max.z;
	}
	else{
		min.x += m.m31*box.max.z; max.x += m.m31*box.min.z;
	}

	if (m.m32 > 0.0f)
	{
		min.y += m.m32*box.min.z; max.y += m.m32*box.max.z;
	}
	else{
		min.y += m.m32*box.max.z; max.y += m.m32*box.min.z;
	}

	if (m.m33 > 0.0f)
	{
		min.z += m.m33*box.min.z; max.z += m.m33*box.max.z;
	}
	else{
		min.z += m.m33*box.max.z; max.z += m.m33*box.min.z;
	}

}

bool AABB3::isEmpty() const
{
	return (min.x > max.x) || (min.y > max.y) || (min.z > max.z);
}

bool AABB3::contains(const Vector3& p) const
{
	return 
		(p.x >= min.x) && (p.x <= max.x) &&
		(p.y >= min.y) && (p.y <= max.y) &&
		(p.z >= min.z) && (p.z <= max.z);
}

Vector3 AABB3::closestPointTo(const Vector3& p) const
{
	Vector3 r;
	if (p.x < min.x)
	{
		r.x = min.x;
	}
	else if (p.x>max.x)
	{
		r.x = max.x;
	}
	else{
		r.x = p.x;
	}

	if (p.y < min.y)
	{
		r.y = min.y;
	}
	else if (p.y >max.y)
	{
		r.y = max.y;
	}
	else {
		r.y = p.y;
	}

	if (p.z < min.z)
	{
		r.z = min.z;
	}
	else if (p.z>max.z)
	{
		r.z = max.z;
	}
	else {
		r.z = p.z;
	}

	return r;

}

//bool AABB3::intersectsSphere(const Vector3& center, float radius) const 
//{
//	Vector3 closestPoint = closestPointTo(center);
//	return distanceSquared(center, closestPoint) < radius*radius;
//}

float AABB3::rayIntersect
(const Vector3& rayOrg, const Vector3& rayDelta, Vector3* returnNormal /* = 0 */) const 
{
	const float kNoIntersection = 1e30f;
	bool inside = true;
	float xt, xn;
	if (rayOrg.x < min.x)
	{
		xt = min.x - rayOrg.x;
		if (xt > rayDelta.x)
			return kNoIntersection;
		xt /= rayDelta.x;
		inside = false;
		xn = -1.0f;
	}
	else if (rayOrg.x > max.x)
	{
		xt = max.x - rayOrg.x;
		if (xt < rayDelta.x) return kNoIntersection;
		xt /= rayDelta.x;
		inside = false;
		xn = 1.0f;
	}
	else{
		xt = -1.0f;
	}

	float yt, yn;
	if (rayOrg.y < min.y)
	{
		yt = min.y - rayOrg.y;
		if (yt > rayDelta.y) return kNoIntersection;
		yt /= rayDelta.y;
		inside = false;
		yn = -1.0f;
	}
	else if (rayOrg.y>max.y)
	{
		yt = max.y - rayOrg.y;
		if (yt < rayDelta.y) return kNoIntersection;
		yt /= rayDelta.y;
		inside = false;
		yn = 1.0f;
	}
	else {
		yt = -1.0f;
	}

	float zt, zn;
	if (rayOrg.z < min.z)
	{
		zt = min.z - rayOrg.z;
		if (zt > rayDelta.z) return kNoIntersection;
		zt /= rayDelta.z;
		inside = false;
		zn = -1.0f;
	}
	else if (rayOrg.z>max.z)
	{
		zt = max.z - rayOrg.z;
		if (zt < rayDelta.z) return kNoIntersection;
		zt /= rayDelta.z;
		inside = false;
		zn = 1.0f;
	}
	else{
		zt = -1.0f;
	}
	if (inside)
	{
		if (returnNormal != nullptr)
		{
			*returnNormal = -rayDelta;
			returnNormal->normalize();
		}
		return 0.0f;
	}

	int which = 0;
	float t = xt;
	if (yt > t){
		which = 1;
		t = yt;
	}
	if (zt>t)
	{
		which = 2;
		t = zt;
	}
	switch (which)
	{
	case 0:
	{
		float y = rayOrg.y + rayDelta.y*t;
		if (y<min.y || y>max.y) return kNoIntersection;
		float z = rayOrg.z + rayDelta.z*t;
		if (z<min.z || z>max.z)return kNoIntersection;
		if (returnNormal != nullptr)
		{
			returnNormal->x = xn;
			returnNormal->y = 0.0f;
			returnNormal->z = 0.0f;
		}
		break;
	}
	case 1:
	{
		float x = rayOrg.x + rayDelta.x*t; 
		if (x<min.x || x>max.x) return kNoIntersection;
		float z = rayOrg.z + rayDelta.z*t;
		if (z<min.z || z>max.z) return kNoIntersection;
		if (returnNormal != nullptr)
		{
			returnNormal->x = 0.0f;
			returnNormal->y = yn;
			returnNormal->z = 0.0f;
		}
		break;
	}
	case 2:
	{
		float x = rayOrg.x + rayDelta.x*t;
		if (x<min.x || x>max.x) return kNoIntersection;
		float y = rayOrg.y + rayDelta.y*t;
		if (y<min.y || y>max.y) return kNoIntersection;
		if (returnNormal != nullptr)
		{
			returnNormal->x = 0.0f;
			returnNormal->y = 0.0f;
			returnNormal->z = zn;
		}
		break;
	}
	}
	return t;
}


/*动态AABB与平面相交性检测
 *dir是AABB移动的方向
 *假设平面是静止的，
 *返回交点的参数值--相交时AABB移动的距离，如果未相交则返回一个大数
 *只检测和平面正面的相交*/
float AABB3::intersectPlane(const Vector3& n, float planeD, const Vector3& dir) const
{
	assert(fabs(n*n - 1.0) < 0.01);
	assert(fabs(dir*dir - 1.0) < 0.01);

	const float kNoIntersection = 1e30f;

	//计算夹角，确保我们在向平面的正面移动
	float dot = n*dir;
	if (dot >= 0.0f)
	{
		return kNoIntersection;
	}

	float minD, maxD;
	if (n.x > 0.0f)
	{
		minD = n.x*min.x; maxD = n.x*max.x;
	}
	else{
		minD = n.x*max.x; maxD = n.x*min.x;
	}

	if (n.y > 0.0f)
	{
		minD += n.y*min.y; maxD += n.y*max.y;
	}
	else {
		minD += n.y*max.y; maxD += n.y*min.y;
	}

	if (n.z > 0.0f)
	{
		minD += n.z*min.z; maxD += n.z*max.z;
	}
	else {
		minD += n.z*max.z; maxD += n.z*min.z;
	}

	if (maxD <= planeD)
		return kNoIntersection;

	float t = (planeD - minD) / dot;
	if (t < 0.0f)
		return 0.0f;

	return t;

}


bool intersectAABBs(const AABB3& box1, const AABB3& box2, AABB3* boxIntersect /* = 0 */)
{
	if (box1.min.x>box2.max.x) return false;
	if (box1.max.x < box2.min.x)return false;

	if (box1.min.y > box2.max.y) return false;
	if (box1.max.y < box2.min.y) return false;

	if (box1.min.z > box2.max.z) return false;
	if (box1.max.z < box2.min.z) return false;

	if (boxIntersect != nullptr)
	{
		boxIntersect->min.x = max(box1.min.x, box2.min.x);
		boxIntersect->max.x = min(box1.max.x, box2.max.x);
		boxIntersect->min.y = max(box1.min.y, box2.min.y);
		boxIntersect->max.y = min(box1.max.y, box2.max.y);
		boxIntersect->min.z = max(box1.min.z, box2.min.z);
		boxIntersect->max.z = min(box1.max.z, box2.max.z);
	}
	return true;
}

//动态AABB的相交性测试，如果返回值>1则未相交
float intersectMovingAABB(const AABB3& stationaryBox, const AABB3& movingBox, const Vector3& d)
{
	const float kNoIntersection = 1e30f;
	//初始化时间区间，以包含需要考虑的全部时间段
	float tEnter = 0.0f; 
	float tLeave = 1.0f;

	/*计算每一维上的重叠部分，再将这个重叠部分和前面的重叠部分作相交
	 *如果有一维尚重叠部分为零则返回（不会相交）
	 *每一维上必须当心零重叠*/

	if (d.x == 0.0f)
	{
		if ((stationaryBox.min.x >= movingBox.max.x) ||
			(stationaryBox.max.x <= movingBox.min.x))
			return kNoIntersection;
		else{
			float oneOverD = 1.0f / d.x;
			float xEnter = (stationaryBox.min.x - movingBox.max.x)*oneOverD;
			float xLeave = (stationaryBox.max.x - movingBox.min.x)*oneOverD;

			if (xEnter > xLeave)
				swap(xEnter, xLeave);

			if (xEnter > tEnter) tEnter = xEnter;
			if (xLeave < tLeave) tLeave = xLeave;

			if (tEnter > tLeave) return kNoIntersection;
		}
	}

	if (d.y == 0.0f)
	{
		if ((stationaryBox.min.y >= movingBox.max.y)
			|| (stationaryBox.max.y <= movingBox.min.y))
			return kNoIntersection;
		else{
			float oneOverD = 1.0f / d.y;

			float yEnter = (stationaryBox.min.y - movingBox.max.y)*oneOverD;
			float yLeave = (stationaryBox.max.y - movingBox.min.y)*oneOverD;

			if (yEnter > yLeave)
				swap(yEnter, yLeave);

			if (yEnter > tEnter) tEnter = yEnter;
			if (yLeave < tLeave) tLeave = yLeave;

			if (tEnter > tLeave)return kNoIntersection;
		}
	}

	if (d.z == 0.0f)
	{
		if ((stationaryBox.min.z >= movingBox.max.z)
			|| (stationaryBox.max.z <= movingBox.min.z))
			return kNoIntersection;
		else{
			float oneOverD = 1.0f / d.z;

			float zEnter = (stationaryBox.min.z - movingBox.max.z)*oneOverD;
			float zLeave = (stationaryBox.max.z - movingBox.min.z)*oneOverD;

			if (zEnter > zLeave)
				swap(zEnter, zLeave);

			if (zEnter > tEnter) tEnter = zEnter;
			if (zLeave < tLeave) tLeave = zLeave;

			if (tEnter > tLeave)
				return kNoIntersection;
		}
	}

	return tEnter;
}