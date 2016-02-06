#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include "Vec3.h"

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

struct Rays
{
	std::vector<float> originX, originY, originZ;
	std::vector<float> directionX, directionY, directionZ;
};

struct Spheres
{
	std::vector<float> centerX, centerY, centerZ;
	std::vector<float> radiuses;
};

#endif /* COMMON_H_ */
