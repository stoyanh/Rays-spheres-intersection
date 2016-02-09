#ifndef UTILS_H_
#define UTILS_H_

#include "Common.h"
#include <vector>

using std::vector;

namespace Intersection
{

	IntersectionData intersectRaySpheres(const Ray& ray, const vector<int>& spheresIndices,
			const Spheres& spheres);

	IntersectionData intersectSingleSphere(const Ray& ray, const Sphere& sphere);
}



#endif /* UTILS_H_ */
