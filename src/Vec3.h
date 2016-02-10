#ifndef VEC3_H_
#define VEC3_H_

#include <cmath>

typedef __attribute__ ((vector_size(4 * sizeof(float)))) float Vec4Float;

struct Vec3
{
	union {
		struct {
			float x, y, z;
		};
		float coords[3];
		Vec4Float vec;
	};

	Vec3()
	{
		x = y = z = 0.f;
	}

	Vec3(float _x, float _y, float _z):
		x(_x), y(_y), z(_z) {}

	Vec3(const Vec4Float& vec)
	{
		this->vec = vec;
	}

	Vec3 operator+(const Vec3& rhs) const
	{
		return Vec3(vec + rhs.vec);
	}

	Vec3 operator-(const Vec3& rhs) const
	{
		return Vec3(vec - rhs.vec);
	}

	float operator*(const Vec3& rhs) const
	{
		Vec4Float copy = vec * rhs.vec;
		return copy[0] + copy[1] + copy[2];
	}

	Vec3& operator*=(const float multiplier)
	{
		vec = vec * multiplier;
		return *this;
	}

	float length() const
	{
		Vec4Float copy = vec * vec;
		return sqrt(copy[0] + copy[1] + copy[2]);
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

inline Vec3 operator*(const Vec3& v, const float multiplier)
{
	return Vec3(v.vec * multiplier);
}

inline Vec3 operator*(const float multiplier, const Vec3& v)
{
	return Vec3(v.vec * multiplier);
}

inline Vec3 normalize(const Vec3& v)
{
	return Vec3(v.vec * (1.f / v.length()));
}

#endif /* VEC3_H_ */
