#include <iostream>
#include "KDtree.h"
#include <xmmintrin.h>
#include "Vec3.h"


int main()
{
	Spheres spheres;
	const int size = 1000;
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

	Ray ray;
	ray.direction = Vec3(1, 1, 1);

	IntersectionData data = tree.intersectRay(ray);


	return 0;
}
