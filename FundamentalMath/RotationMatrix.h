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
	
	//��Ϊ��λ����
	void identity(); 

	//����ŷ����ָ���ķ�λ�������
	void setup(const EulerAngles& orientation);

	//������Ԫ��������󣬼������Ԫ����������ָ������ı任
	void fromInertialToObjectQuaternion(const Quaternion& q);
	void fromObjectToInertialQuaternion(const Quaternion& q);

	//ִ����ת

	Vector3 inertialToObject(const Vector3& v) const;
	Vector3 ObjectToinertial(const Vector3& v) const;
};

#endif