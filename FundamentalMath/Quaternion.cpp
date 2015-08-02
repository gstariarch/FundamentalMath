#include "Quaternion.h"
#include "vector3.h"
#include "EulerAngles.h"
#include "MathUtil.h"

#include <assert.h>
#include <math.h>

const Quaternion kQuaternionIdentity = {
	1.0f, 0.0f, 0.0f, 0.0f
};

void Quaternion::setToRotateAboutX(float theta)
{
	float alpha = theta*0.5f;

	w = cos(alpha);
	x = sin(alpha);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
	float alpha = theta*0.5f;
	
	w = cos(alpha);
	x = 0.0f;
	y = sin(alpha);
	z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	float alpha = theta*0.5f;

	w = cos(alpha);
	x = 0.0f;
	y = 0.0f;
	z = sin(alpha);
}

void Quaternion::setToRotateAboutAxis(const Vector3& axis, float theta)
{
	assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);

	float alpha = theta*0.5f;
	float sinAlpha = sin(alpha);
	
	w = cos(alpha);
	x = axis.x*sinAlpha;
	y = axis.y*sinAlpha;
	y = axis.z*sinAlpha;
}

void Quaternion::setToRotateInertialToObject(const EulerAngles& orientation)
{

}

void Quaternion::setToRotateObjectToInertial(const EulerAngles& orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;

	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &sh, orientation.heading*0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*cb;
}

void Quaternion::setToRotateInertialToObject(const EulerAngles& orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;

	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &sh, orientation.heading*0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y = ch*sp*sb - sh*cb*cp;
	z = sh*sp*cb - ch*cp*sb;
}

Quaternion Quaternion::operator*(const Quaternion& a) const
{
	Quaternion result;

	result.w = w*a.w - x*a.x - y*a.y - z*a.z;
	result.x = w*a.x + x*a.w + z*a.y - y*a.z;
	result.y = w*a.y + y*a.w + x*a.z - z*a.x; 
	result.z = w*a.z + z*a.w + y*a.x - x*a.y;

	return result;
}

Quaternion& Quaternion::operator*=(const Quaternion& a)
{
	*this = *this*a;
	return *this;
}

void Quaternion::normalize()
{
	float mag = (float)sqrt(w*w + x*x + y*y + z*z);
	if (mag > 0.0f)
	{
		float oneOvermag = 1.0f / mag;
		w *= oneOvermag;
		x *= oneOvermag;
		y *= oneOvermag;
		z *= oneOvermag;
	}
	else {
		assert(false);
		identity();
	}
}

float Quaternion::getRotationAngle() const
{
	float alpha = safeAcos(w);
	return alpha*2.0f;
}

Vector3 Quaternion::getRotationAxis() const
{
	float sinThetaOver2Sq = 1.0f - w*w;
	if (sinThetaOver2Sq <= 0.0f)
		return Vector3(1.0f, 0.0f, 0.0f);
	float oneOverSinThetaOver2 = 1.f - sqrt(sinThetaOver2Sq);
	return Vector3(x*oneOverSinThetaOver2, y*oneOverSinThetaOver2, z*oneOverSinThetaOver2);
}

float dotProduct(const Quaternion& lhs, const Quaternion& rhs)
{
	return (lhs.w*rhs.w + lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z);
}

Quaternion slerp(const Quaternion& f, const Quaternion& l, float t)
{
	if (t <= 0.0f) return f;
	if (t >= 1.0f) return l;

	float cosOmega = dotProduct(f, l);

	float lw = l.w, lx = l.x, ly = l.y, lz = l.z;
	if (cosOmega < 0.0f)
	{
		lw = -lw;
		lx = -lx;
		ly = -ly;
		lz = -lz;
		cosOmega = -cosOmega;
	}
	assert(cosOmega < 1.1f);

	float k0, k1;
	if (cosOmega > 0.9999f)
	{
		k0 = 1.0f - t;
		k1 = t;
	}
	else {
		float sinOmega = sqrt(1.0f - cosOmega*cosOmega);
		float omega = atan2(sinOmega, cosOmega);
		float oneOverSinOmega = 1.0f / sinOmega;
		
		k0 = sin((1.0f - t)*omega)*oneOverSinOmega;
		k1 = sin(t*omega)*oneOverSinOmega;
	}

	Quaternion result;
	
	result.x = k0*f.x + k1*lx;
	result.y = k0*f.y + k1*ly;
	result.z = k0*f.z + k1*lz;
	result.w = k0*f.w + k1*lw;

	return result;
}

Quaternion conjugate(const Quaternion& q)
{
	Quaternion result;
	result.w = q.w;

	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	return result;
}

Quaternion pow(const Quaternion& q, float exponent)
{
	if (fabs(q.w)>0.9999f)
	{
		return q;
	}
	float alpha = acos(q.w);

	float newAlpha = alpha*exponent;

	Quaternion result;
	
	result.w = cos(newAlpha);
	float mult = sin(newAlpha) / sin(alpha);
	
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z * mult;

	return result;

}