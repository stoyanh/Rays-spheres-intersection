#include "RaySphereIntersect.h"
#include "KDTree.h"

void intersectRaySpheres(const Ray& ray, const Spheres& spheres, IntersectionData& data)
{
	KDTree tree;
	tree.build(spheres);

	data = tree.intersectRay(ray);
}

void intersectRaysSpheres(const Rays& rays, const Spheres& spheres, std::vector<IntersectionData>& intersections)
{


}
