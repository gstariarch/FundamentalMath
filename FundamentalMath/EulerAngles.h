#ifndef _EULERANGLES_H_
#define _EULERANGLES_H_

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngles
{
public:
	float heading, pitch, bank;

	EulerAngles()
	{

	}
	EulerAngles(float h, float p, float b) :
		heading(h), pitch(p), bank(b)
	{

	}
	//Set zero
	void identity()
	{
		pitch = bank = heading = 0.0f;
	}
	//变换为“限制集”欧拉角
	void canonize();

	//From Quaternion to EulerAngles
	void fromObjectToInertialQuaternion(const Quaternion & q);
	void fromInertialToObjectQuaternion(const Quaternion& q);

	//From matrix to EulerAngles
	void fromObjectToWorldMatrix(const Matrix4x3& m);
	void fromWorldToObjectMatrix(const Matrix4x3& m);

	//From RotationMatrix to EulerAngles.
	void fromRotationMatrixToEuler(const RotationMatrix& m);
};


extern const EulerAngles kZeroEulerAnglesIdentity;



#endif