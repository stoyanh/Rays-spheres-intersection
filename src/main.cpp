#include <iostream>
#include "KDTree.h"
#include <xmmintrin.h>
#include "Vec3.h"
#include <cstdlib>


int main()
{
	Spheres spheres;
	const int size = 1000000;
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < size; ++j)
		{
			spheres.centerCoords[i].push_back(j + 10);
		}
	}

	for(int i = 0; i < size; ++i)
	{
		spheres.radiuses.push_back(2.f);
	}

	spheres.count = size;

	KDTree tree;
	tree.build(spheres);
	std::cout << "Tree built" << std::endl;

	for(int i = 0; i < 100000; ++i)
	{
		Ray ray;
		ray.origin = Vec3(0, 0, 0);
		ray.direction = Vec3(1, 1, 1);

		ray.direction = normalize(ray.direction);

		IntersectionData data = tree.intersectRay(ray);

		Vec3 intersectionPoint = ray.origin + data.tIntersection * ray.direction;

		//std::cout << data.intersection << std::endl;
//		if(data.intersection)
//		{
//			for(int i = 0; i < 3; ++i)
//			{
//				std::cout << intersectionPoint.coords[i] << std::endl;
//			}
//		}
	}

	return 0;
}
