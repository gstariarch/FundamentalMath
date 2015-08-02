#ifndef _MATRIX4X3_H_
#define _MATRIX4X3_H_

class Vector3;
class EulerAngles;
class RotationMatrix;
class Quaternion;


class Matrix4x3
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	float tx, ty, tz;

	void identity();

	//直接访问平移部分
	void zeroTranslation();

	//平移部分赋值
	void setTranslation(const Vector3& d);

	//线性变换部分置为单位矩阵，平移部分赋值
	void setupTranslation(const Vector3& d);
	
	/*创建一个矩阵能将点从局部坐标空间变换到父空间坐标。
	 *首先从物体空间变换到惯性空间，接着变换到世界空间*/
	void setupLocalToParent(const Vector3& pos, const EulerAngles& orient);
	void setupLocalToParent(const Vector3& pos, const RotationMatrix& orient);

	/*构造从父空间到局部空间的矩阵，首先从世界空间转换到惯性空间，接着转换到物体空间*/
	void setupParentToLocal(const Vector3& pos, const EulerAngles& orient);
	void setupParentToLocal(const Vector3& pos, const RotationMatrix& orient);

	//构造绕坐标轴旋转的矩阵

	void setupRotate(int axis, float theta);
	void setupRotate(const Vector3& axis, float theta);
	
	//构造旋转矩阵，角位移由四元数形式给出
	void fromQuaternion(const Quaternion& q); 

	//构造沿坐标轴缩放的矩阵
	void setupScale(const Vector3& s);

	// 构造沿任意轴缩放的矩阵
	void setupScaleAlongAxis(const Vector3& axis, float k);

	//构造切变矩阵
	void setupShear(int axis, float s, float t);

	//构造投影矩阵，投影平面过原点
	void setupProject(const Vector3& n);

	//构造反射矩阵
	void setupReflect(int axis, float k = 0.0f);

	//构造沿任意平面反射的矩阵

	void setupReflect(const Vector3& n);

};

Vector3 operator*(const Vector3& p, const Matrix4x3& m);
Matrix4x3 operator*(const Matrix4x3& a, const Matrix4x3& b);

Vector3&   operator*=(Vector3& p, const Matrix4x3& m);
Matrix4x3& operator*=(const Matrix4x3& a, const Matrix4x3& m);

//计算3x3部分的行列式
float determinant(const Matrix4x3& m);

//计算矩阵的逆
Matrix4x3 inverse(const Matrix4x3& m);

//提取矩阵的平移部分

Vector3 getTranslation(const Matrix4x3& m);

//从局部矩阵->父矩阵 或 父矩阵->局部矩阵 取位置/方位
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3& m);
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3& m);



#endif