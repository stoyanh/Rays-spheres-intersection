#ifndef RAYSPHEREINTERSECT_H_
#define RAYSPHEREINTERSECT_H_

#include <vector>
#include "Common.h"

void intersectRaySpheres(const Ray& ray, const Spheres& spheres, IntersectionData& data);

void intersectRaysSpheres(const Rays& rays, const Spheres& spheres, std::vector<IntersectionData>& intersections);



#endif /* RAYSPHEREINTERSECT_H_ */
