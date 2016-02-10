#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include "Vec3.h"

struct Ray
{
	Ray(){}
	Ray(const Vec3& origin, const Vec3& direction)
	{
		this->origin = origin;
		this->direction = direction;
	}
	Vec3 origin;
	Vec3 direction;
};

struct Sphere
{
	Vec3 center;
	float radius;
};

struct Rays
{
	std::vector<Ray> rays;
};

struct Spheres
{
	std::vector<float> centerCoords[3];
	std::vector<float> radiuses;
	int count;
};

struct IntersectionData
{
	bool intersection;
	float tIntersection;
};

#endif /* COMMON_H_ */
