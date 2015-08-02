#ifndef _QUATERNION_H_
#define _QUATERNION_H_

class Vector3;
class EulerAngles;

class Quaternion
{
public:
	float w, x, y, z;

	void identity()
	{
		w = 1.0f;
		x = y = z = 0.0f;
	}

	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3& axis, float theta);

	void setToRotateObjectToInertial(const EulerAngles& orientation);
	void setToRotateInertialToObject(const EulerAngles& orientation);

	//叉乘
	Quaternion operator*(const Quaternion& rhs) const;
	Quaternion& operator*=(const Quaternion& rhs);

	//正则化
	void normalize();

	//提取旋转角和旋转轴
	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
	
};


//全局单位四元数
extern const Quaternion kQuaternionIdentity;

//四元数点乘
extern float dotProduct(const Quaternion& lhs, const Quaternion& rhs);

//球面线性插值
extern Quaternion slerp(const Quaternion& f, const Quaternion& l, float t); 

//四元数共轭
extern Quaternion conjugate(const Quaternion& q);

//四元数幂
extern Quaternion pow(const Quaternion& q, float exponent);

#endif