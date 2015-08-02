#ifndef  _ROTATIONMATRIX_H_
#define _ROTATIONMATRIX_H_

class Vector3;
class EulerAngles;

class Quaternion;

class RotationMatrix
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	
	//置为单位矩阵
	void identity(); 

	//根据欧拉角指定的方位构造矩阵
	void setup(const EulerAngles& orientation);

	//根据四元数构造矩阵，假设该四元数参数代表指定方向的变换
	void fromInertialToObjectQuaternion(const Quaternion& q);
	void fromObjectToInertialQuaternion(const Quaternion& q);

	//执行旋转

	Vector3 inertialToObject(const Vector3& v) const;
	Vector3 ObjectToinertial(const Vector3& v) const;
};

#endif