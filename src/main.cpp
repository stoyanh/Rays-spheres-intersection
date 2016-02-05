#include <iostream>
#include "Vec3.h"
#include <ctime>
#include "xmmintrin.h"

struct Ray
{
	Vec3 origin;
	Vec3 direction;
};

struct Sphere
{
	Vec3 center;
	float radius;
};

struct IntersectionData
{
	bool intersect;
	Vec3 closestPoint;
};

IntersectionData intersect(const Ray& ray, const Sphere& sphere)
{
	IntersectionData data;
	Vec3 v = sphere.center - ray.origin;

	float vLength = v.length();
	bool originInside = vLength < sphere.radius;

	float ta = v * ray.direction;
	if(ta < 0 && !originInside)
	{
		data.intersect = false;
		return data;
	}

	float R = sphere.radius;
	float thSquare = R * R  - vLength * vLength + ta * ta;

	if(thSquare < 0)
	{
		data.intersect = false;
		return data;
	}

	data.intersect = true;
	float th = sqrt(thSquare);
	if(!originInside)
	{
		data.closestPoint = ray.origin + (ta - th) * ray.direction;
	}
	else
	{
		data.closestPoint = ray.origin + (ta + th) * ray.direction;
	}

	return data;
}

IntersectionData intersectAlg(const Ray& ray, const Sphere& sphere)
{
	IntersectionData data;
	float A = 1.f, B = 0.f, C = 0.f;

	for(int i = 0; i < 3; ++i)
	{
		B += (ray.direction.coords[i] * ray.origin.coords[i] - ray.direction.coords[i] * sphere.center.coords[i]);
		C += (ray.origin.coords[i] * ray.origin.coords[i] -
			  2.f * ray.origin.coords[i] * sphere.center.coords[i] +
			  sphere.center.coords[i] * sphere.center.coords[i]);
	}
	C -= sphere.radius * sphere.radius;
	float discriminant = B * B - 4.f * C;
	if(discriminant < 0)
	{
		data.intersect = false;
		return data;
	}

	data.intersect = true;
	data.closestPoint = ray.origin + ((-B - sqrt(discriminant)) / 2.f) * ray.direction;
	return data;
}

IntersectionData intersectIntr(const Ray& ray, const Sphere& sphere)
{
	IntersectionData data;

	__m128 dirCoords = _mm_set_ps(ray.direction.x, ray.direction.y, ray.direction.z, 0.f);
	__m128 originCoords = _mm_set_ps(ray.origin.x, ray.origin.y, ray.origin.z, 0.f);
	__m128 minusDirCoords = _mm_set_ps(-ray.direction.x, -ray.direction.y, -ray.direction.z, 0.f);
	__m128 centerCoords  = _mm_set_ps(sphere.center.x, sphere.center.y, sphere.center.z, 0.f);

	__m128 mult1 = _mm_mul_ps(dirCoords, originCoords);
	__m128 mult2 = _mm_mul_ps(minusDirCoords, centerCoords);

	__m128 added = _mm_add_ps(mult1, mult2);

	__m128 arr1 = _mm_set_ps(ray.origin.x - sphere.center.x, ray.origin.y - sphere.center.y,
			ray.origin.z - sphere.center.z, -sphere.radius);

	__m128 arr2 = _mm_set_ps(ray.origin.x - sphere.center.x, ray.origin.y - sphere.center.y,
				ray.origin.z - sphere.center.z, sphere.radius);

	__m128 multiplied = _mm_mul_ps(arr1, arr2);

	float b[4], c[4];
	_mm_store_ps(b, added);
	_mm_store_ps(c, multiplied);
	float B = 0.f, C = 0.f;
	for(int i = 0; i < 4; ++i)
	{
		B += b[i];
		C += c[i];
	}

	float discriminant = B * B - 4.f * C;
	if(discriminant < 0)
	{
		data.intersect = false;
		return data;
	}

	data.intersect = true;
	data.closestPoint = ray.origin + ((-B - sqrt(discriminant)) / 2.f) * ray.direction;

	return data;
}
int main()
{
	Ray ray;
	ray.origin = Vec3(0, 0, 0);
	ray.direction = normalize(Vec3(1, 1, 1));

	Sphere sphere;
	sphere.center = Vec3(0.5f, 0.5f, 0.5f);
	sphere.radius = 5.f;

	const int tries = 100000000;
	const clock_t begin = clock();

	for(int i = 0; i < tries; ++i)
	{
		intersectAlg(ray, sphere);
	}
	std::cout << static_cast<float>(clock() - begin) / static_cast<float>(CLOCKS_PER_SEC) << std::endl;
	return 0;
}
