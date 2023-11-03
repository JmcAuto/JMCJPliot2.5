#ifndef __VECTOR_H
#define __VECTOR_H
#include <float.h>
#include <math.h>
/*
     VECTOR.H

     Vector3f class

     OpenGL Game Programming
     by Kevin Hawkins and Dave Astle

     Some operators of the Vector3f class based on
     operators of the Vector3f class by Bas Kuenen.
     Copyright (c) 2000 Bas Kuenen. All Rights Reserved.
     homepage: baskuenen.cfxweb.net
*/
struct Vector4f
{
	float x;
	float y;
	float z;
	float w;
	Vector4f() :
		x(0.0),
		y(0.0),
		z(0.0),
		w(0.0)
	{};
	Vector4f(float _x, float _y, float _z, float _w) :
		x(_x),
		y(_y),
		z(_z),
		w(_w)
	{};
	Vector4f(const Vector4f &other) :
		x(other.x),
		y(other.y),
		z(other.z),
		w(other.w)
	{};
	inline const float Length() const
	{
		return (float)sqrt((double)(x*x + y*y + z*z + w*w));
	}

	inline void Normalize()
	{
		*this /= Length();
	}

	inline Vector4f Normalized()
	{
		return *this / Length();
	}

	inline float &operator[](const long idx)
	{
		return *((&x) + idx);
	}
	inline const Vector4f operator-(const Vector4f& vec) const
	{
		return Vector4f(x - vec.x, y - vec.y, z - vec.z, w - vec.w);
	}
	inline const Vector4f operator+(const Vector4f& vec) const
	{
		return Vector4f(x + vec.x, y + vec.y, z + vec.z, w + vec.w);
	}

	const Vector4f& operator+=(const Vector4f& vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		w += vec.w;
		return *this;
	}

	const Vector4f operator/(float s) const
	{
		s = 1 / s;
		return Vector4f(s*x, s*y, s*z, s*w);
	}
	const Vector4f operator /=(float s)
	{
		*this = *this / s;
		return *this;
	}
	inline const Vector4f operator*(const float &s) const
	{
		return Vector4f(x*s, y*s, z*s, w*s);
	}


};
struct Vector4uc
{
	unsigned char x;
	unsigned char y;
	unsigned char z;
	unsigned char w;
	Vector4uc()
	{
		x = y = z = w = 0;
	};
	Vector4uc(unsigned char _x, unsigned char _y, unsigned char _z, unsigned char _w) :
		x(_x), y(_y), z(_z), w(_w)
	{}
	inline unsigned char &operator[](const long idx)
	{
		return *((&x) + idx);
	}
};

class Vector3f
{
public:
	float x;
	float y;
	float z;    // x,y,z coordinates

public:
	Vector3f(float a = 0, float b = 0, float c = 0) : x(a), y(b), z(c) {}
	Vector3f(const Vector3f &vec) : x(vec.x), y(vec.y), z(vec.z) {}
	Vector3f(const Vector4f &vec) : x(vec.x), y(vec.y), z(vec.z)
	{
		if (fabs(vec.w) > FLT_EPSILON)
		{
			*this /= vec.w;
		}
	}

	inline void SetZero()
	{
		x = y = z = 0.0f;
	}

	// vector index
	inline float &operator[](const long idx)
	{
		return *((&x)+idx);
	}

	// vector assignment
	inline const Vector3f &operator=(const Vector3f &vec)
	{
		x = vec.x;
		y = vec.y;
		z = vec.z;

		return *this;
	}

	// vecector equality
	inline const bool operator==(const Vector3f &vec) const
     {
          return ((x == vec.x) && (y == vec.y) && (z == vec.z));
     }

	// vecector inequality
	inline const bool operator!=(const Vector3f &vec) const
     {
          return !(*this == vec);
     }

	//vector add
	inline const Vector3f operator+(const Vector3f &vec) const
	{
	  return Vector3f(x + vec.x, y + vec.y, z + vec.z);
	}

	// vector add (opposite of negation)
	const Vector3f operator+() const
     {    
          return Vector3f(*this);
     }

	// vector increment
	const Vector3f& operator+=(const Vector3f& vec)
	{
		x += vec.x;
		y += vec.y;
		z += vec.z;
		return *this;
	}

	// vector subtraction
	inline const Vector3f operator-(const Vector3f& vec) const
     {    
          return Vector3f(x - vec.x, y - vec.y, z - vec.z);
     }
	// vector negation
	inline const Vector3f operator-() const
     {    
          return Vector3f(-x, -y, -z);
     }

	// vector decrement
	const Vector3f &operator-=(const Vector3f& vec)
     {
          x -= vec.x;
          y -= vec.y;
          z -= vec.z;

          return *this;
     }

	// scalar self-multiply
	const Vector3f &operator*=(const float &s)
     {
          x *= s;
          y *= s;
          z *= s;
          
          return *this;
     }

	// scalar self-divecide
	const Vector3f &operator/=(const float &s)
     {
          const float recip = 1/s; // for speed, one divecision

          x *= recip;
          y *= recip;
          z *= recip;

          return *this;
     }

	// post multiply by scalar
	inline const Vector3f operator*(const float &s) const
	{
		return Vector3f(x*s, y*s, z*s);
	}

	// pre multiply by scalar
	friend inline const Vector3f operator*(const float &s, const Vector3f &vec)
	{
		return vec*s;
	}

	inline const Vector3f operator*(const Vector3f& vec) const
	{
		return Vector3f(x*vec.x, y*vec.y, z*vec.z);
	}

	// post multiply by scalar
	/*friend inline const Vector3f operator*(const Vector3f &vec, const float &s)
	{
		return Vector3f(vec.x*s, vec.y*s, vec.z*s);
	}*/

	// divide by scalar
	const Vector3f operator/(float s) const
     {
          s = 1/s;

          return Vector3f(s*x, s*y, s*z);
     }

	// get the scalar between two vector
	const float operator/(const Vector3f& vec) const
	{
		return Length() / vec.Length() * this->Normalized().DotProduct(vec.Normalized());
	}


	// cross product
	inline const Vector3f CrossProduct(const Vector3f &vec) const
	{
		return *this ^ vec;
	}

	// cross product
	inline const Vector3f operator^(const Vector3f &vec) const
	{
		return Vector3f(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
	}

	// dot product
	inline const float DotProduct(const Vector3f &vec) const
	{
		return *this % vec;
	}

	// dot product
	const float operator%(const Vector3f &vec) const
	{
		return x*vec.x + y*vec.y + z*vec.z;
	}

	// length of vector
	inline const float Length() const
     {
          return (float)sqrt((double)(x*x + y*y + z*z));
     }

	// return the unit vector
	const Vector3f Normalized() const
     {
          return (*this) / Length();
     }

	// normalize this vector
	void Normalize()
     {
          (*this) /= Length();
     }

	const float operator!() const
     {
          return sqrtf(x*x + y*y + z*z);
     }

	// return vector with specified length
	const Vector3f operator | (const float length) const
     {
          return *this * (length / !(*this));
     }

	// set length of vector equal to length
	const Vector3f& operator |= (const float length)
     {
          return *this = *this | length;
     }

	// return angle between two vectors
	const float inline Angle(const Vector3f& normal) const
     {
		Vector3f a(*this);
		Vector3f b(normal);
		a.Normalize();
		b.Normalize();
		float dotVal = a % b;
		if (dotVal > 1.0)
		{
			dotVal = 1.0;
		}
		else if (dotVal < -1.0)
		{
			dotVal = -1.0;
		}
		return acosf(dotVal);
     }

	// reflect this vector off surface with normal vector
	const Vector3f inline Reflection(const Vector3f& normal) const
     {    
          const Vector3f vec(*this | 1);     // normalize this vector
          return (vec - normal * 2.0 * (vec % normal)) * !*this;
     }

	//DEPRECATED: USE Matrix4x4 Instead
	// rotate angle radians around a normal
	const Vector3f inline Rotate(const float angle, const Vector3f& normal) const
	{	
		//const float cosine = cos(angle);
		//const float sine = sin(angle);

		//return Vector3f(*this * cosine + ((normal * *this) * (1.0f - cosine)) *
		//	          normal + (*this ^ normal) * sine);
		return Vector3f();
	}

	inline const Vector3f GetPointOnLine(const Vector3f &lineA, const Vector3f &lineB) const
	{
		Vector3f lineVec = (lineB - lineA).Normalized();
		return lineA + lineVec * (*this - lineA).DotProduct(lineVec);
	}

	inline float GetLuminance()
	{
		return *this % Vector3f(0.299, 0.587, 0.114);
	}

	inline const float DistanceToLine(const Vector3f &lineA, const Vector3f &lineB)
	{
		return (*this - GetPointOnLine(lineA, lineB)).Length();
	}
	static Vector3f GetLinePosInPlane(const Vector3f &planePoint, const Vector3f &planeNormal, const Vector3f &linePoint, const Vector3f &lineDir)
	{
		Vector3f result(0.0, 0.0, 0.0);
		float vpt = planeNormal.DotProduct(lineDir);
		if (vpt != 0.0f)
		{
			float t = ((planePoint.x - linePoint.x) * planeNormal.x + (planePoint.y - linePoint.y) * planeNormal.y + (planePoint.z - linePoint.z) * planeNormal.z) / vpt;
			result = linePoint + lineDir * t;
		}
		return result;
	}

	static bool GetShortestBridge(const Vector3f &lineAPoint1, const Vector3f &lineAPoint2, const Vector3f &lineBPoint1, const Vector3f &lineBPoint2, Vector3f &bridgePointA, Vector3f &bridgePointB)
	{
		Vector3f lineADir = lineAPoint2 - lineAPoint1;
		Vector3f lineBDir = lineBPoint2 - lineBPoint1;
		lineADir.Normalize();
		lineBDir.Normalize();
		double dotResult = lineADir.DotProduct(lineBDir);

		if(dotResult == 0.0)
		{
			return false;
		}
		Vector3f abNormal = lineADir.CrossProduct(lineBDir);
		abNormal.Normalize();
		Vector3f aPlaneNormal = abNormal.CrossProduct(lineADir);
		bridgePointB =  GetLinePosInPlane(lineAPoint1, aPlaneNormal, lineBPoint1, lineBDir);
		Vector3f bPlaneNormal = abNormal.CrossProduct(lineBDir);
		bridgePointA =  GetLinePosInPlane(lineBPoint1, bPlaneNormal, lineAPoint1, lineADir);
		return true;
	}
};

struct Vector2f
{
	float x;
	float y;
	Vector2f() :
		x(0.0),
		y(0.0)
	{};
	Vector2f(float _x, float _y) :
		x(_x),
		y(_y)
	{};
	Vector2f(const Vector2f &other) :
		x(other.x),
		y(other.y)
	{};
	Vector2f(const Vector3f &other) :
		x(other.x),
		y(other.y)
	{};
	inline const Vector2f operator+(const Vector2f& vec) const
	{
		return Vector2f(x + vec.x, y + vec.y);
	}
	inline const Vector2f operator+=(const Vector2f& vec)
	{
		x += vec.x;
		y += vec.y;
		return *this;
	}
	inline const Vector2f operator-(const Vector2f& vec) const
	{
		return Vector2f(x - vec.x, y - vec.y);
	}
	inline const Vector2f &operator-=(const Vector2f& vec)
	{
		x -= vec.x;
		y -= vec.y;
		return *this;
	}
	inline const Vector2f operator*(const float &s) const
	{
		return Vector2f(x * s, y * s);
	}
	inline const Vector2f &operator*=(const float &s)
	{
		x *= s;
		y *= s;
		return *this;
	}
	inline const Vector2f operator/(const float &s) const
	{
		return *this * (1.0 / s);
	}
	// get the scalar between two vector
	const float operator/(const Vector2f& vec) const
	{
		return Length() / vec.Length() * this->Normalized().DotProduct(vec.Normalized());
	}
	inline const Vector2f &operator/=(const float &s)
	{
		const float recip = 1 / s; // for speed, one divecision
		x *= recip;
		y *= recip;
		return *this;
	}
	inline float &operator[](const long idx)
	{
		return *((&x) + idx);
	}

	inline void Normalize()
	{
		*this /= Length();
	}
	inline Vector2f Normalized() const
	{
		return *this / Length();
	}
	inline const Vector2f GetPointOnLine(const Vector2f &lineA, const Vector2f &lineB)
	{
		Vector2f lineVec = (lineB - lineA).Normalized();
		return lineA + lineVec * (*this - lineA).DotProduct(lineVec);
	}
	inline const float DistanceToLine(const Vector2f &lineA, const Vector2f &lineB)
	{
		return (*this - GetPointOnLine(lineA, lineB)).Length();
	}
	inline const float Length() const
	{
		return (float)sqrt((double)(x*x + y*y));
	}
	inline const float DotProduct(const Vector2f& vec) const
	{
		return x * vec.x + y * vec.y;
	}
	inline const float Angle(Vector2f vec) const
	{
		Vector2f nt = *this;
		vec.Normalize();
		nt.Normalize();
		return acosf(nt.DotProduct(vec));
	}
	inline const float CrossProduct(const Vector2f& vec) const
	{
		return x * vec.y - y * vec.x;
	}
	inline static float TriangleArea(const Vector2f &a, const Vector2f &b, const Vector2f &c)
	{
		double lenAB = (a - b).Length();
		double lenBC = (b - c).Length();
		double lenCA = (c - a).Length();
		double p = (lenCA + lenBC + lenAB) / 2.0;
		return sqrt(p * (p - lenCA) * (p - lenBC) * (p - lenAB));
	}

	inline static bool GetLineCross(Vector2f &result, const Vector2f &lineA1, const Vector2f &lineA2, const Vector2f &lineB1, const Vector2f &lineB2)
	{
		double dotPro = (lineA2 - lineA1).CrossProduct(lineB2 - lineB1);
		if (fabs(dotPro) < FLT_EPSILON)
		{
			return false;
		}
		result = lineA1 + (lineA2 - lineA1) * ((lineB1 - lineA1).CrossProduct(lineB2 - lineB1) / dotPro);
		return true;
	}
};
#endif
