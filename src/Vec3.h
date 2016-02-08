#ifndef VEC3_H_
#define VEC3_H_

#include <cmath>

typedef __attribute__ ((vector_size(4 * sizeof(float)))) float floatVector4;

struct Vec3
{
	union {
		struct {
			float x, y, z;
		};
		float coords[3];
		floatVector4 vec;
	};

	Vec3()
	{
		x = y = z = 0.f;
	}

	Vec3(float _x, float _y, float _z):
		x(_x), y(_y), z(_z) {}

	Vec3 operator+(const Vec3& rhs) const
	{
		return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
	}

	Vec3 operator-(const Vec3& rhs) const
	{
		return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
	}

	float operator*(const Vec3& rhs) const
	{
		return x * rhs.x + y * rhs.y  + z * rhs.z;
	}

	Vec3& operator*=(const float multiplier)
	{
		x *= multiplier;
		y *= multiplier;
		z *= multiplier;
		return *this;
	}

	float length() const
	{
		return sqrt(x * x + y * y + z * z);
	}

	float& operator[](const int index)
	{
		return coords[index];
	}

	const float& operator[](const int index) const
	{
		return coords[index];
	}
};

inline Vec3 operator*(const Vec3& vec, const float multiplier)
{
	return Vec3(vec.x * multiplier, vec.y * multiplier, vec.z * multiplier);
}

inline Vec3 operator*(const float multiplier, const Vec3& vec)
{
	return Vec3(vec.x * multiplier, vec.y * multiplier, vec.z * multiplier);
}

inline Vec3 normalize(const Vec3& vec)
{
	float length = vec.length();
	return vec * (1.f / length);
}

#endif /* VEC3_H_ */
