#ifndef _VECTOR3_H_
#define _VECTOR3_H_


#include <cmath>
class Vector3
{
public:
	float x, y, z;
public:
	
	friend Vector3 operator* (float k, const Vector3& v);
	//构造函数和拷贝赋值操作
	Vector3()
	{
	}
	Vector3(float x, float y, float z) :x(x), y(y), z(z)
	{

	}
	Vector3(const Vector3& v) :x(v.x), y(v.y), z(v.z)
	{

	}
	Vector3& operator=(const Vector3& rhs)
	{
		this->x = rhs.x;
		this->y = rhs.x;
		this->z = rhs.x;
		return *this;
	}
	
	//判断运算符重载，用于判断两个向量
	bool operator==(const Vector3& rhs)
	{
		return ((x == rhs.x) && (y == rhs.y) && (z == rhs.z));
	}
	bool operator!=(const Vector3& rhs)
	{
		return *this == rhs;
	}
	
	//向量运算
	void zero()
	{
		this->x = this->y = this->z = 0.0f;
	}
	//重载一元“-”运算符
	Vector3 operator-()
	{
		return Vector3(-x, -y, -z);
	}
	Vector3 operator-() const{
		return Vector3(-x,-y,-z);
	}
	//重载二元加减运算符
	Vector3 operator+(const Vector3& rhs) const
	{
		return Vector3(x + rhs.x, y + rhs.y, z + rhs.z); 
	}
	Vector3 operator - (const Vector3& rhs) const
	{
		return Vector3(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	Vector3 operator *(float k) const{
		return Vector3(k*x, k*y, k*z);
	}
	Vector3 operator /(float k) const
	{
		float temp = 1.0f / k;
		return Vector3(x*temp, y*temp, z*temp);
	}

	Vector3& operator+=(const Vector3& rhs)
	{
		this->x += rhs.x;
		this->y += rhs.y;
		this->z += rhs.z;
		return *this;
	}
	Vector3& operator-=(const Vector3& rhs)
	{
		this->x -= rhs.x;
		this->y -= rhs.y;
		this->z -= rhs.z;
		return *this;
	}
	Vector3& operator*=(const Vector3& rhs)
	{
		this->x *= rhs.x;
		this->y *= rhs.y;
		this->z *= rhs.z;
		return *this;
	}
	Vector3& operator/=(float k)
	{
		float t = 1.0f / k;
		this->x *= t;
		this->y *= t;
		this->z *= t;
		return *this;
	}

	void normalize()
	{
		float mag = sqrt(x*x + y*y + z*z);
		if (mag > 0.0f)
		{
			float k = 1.0f / mag;
			x *= k;
			y *= k;
			z *= k;
		}

	}

	//向量点乘
	float operator*(const Vector3& rhs) const
	{
		return (this->x*rhs.x + this->y*rhs.y + this->z*rhs.z);
	}

	
	inline Vector3 crossProduct(const Vector3& lhs, const Vector3& rhs)
	{
		return Vector3(
			lhs.y*rhs.x - lhs.z*rhs.y,
			lhs.z*rhs.x - lhs.x*rhs.z,
			lhs.x*rhs.y - lhs.y*rhs.x
			);
	}
	
	inline float distance(const Vector3& a, const Vector3& b)
	{
		float dx = b.x - a.x;
		float dy = b.y - a.y;
		float dz = b.z - a.z;
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

}; 

inline float vectorMag(const Vector3& v)
{
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
//声明为友元函数
inline Vector3 operator* (float k, const Vector3& v)
{
	return Vector3(k*v.x, k*v.y, k*v.z);
}

//全局变量，用来传递零向量。
extern const Vector3 kZeroVector;


#endif