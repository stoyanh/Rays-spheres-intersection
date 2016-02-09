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
	std::vector<float> originCoords[3];
	std::vector<float> directionCoords[3];
	int count;
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
