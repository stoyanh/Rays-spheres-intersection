#include <iostream>
#include "KDtree.h"
#include <xmmintrin.h>
#include "Vec3.h"


int main()
{
	Spheres spheres;
	const int size = 100000000;
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < size; ++j)
		{
			spheres.centerCoords[i].push_back(j);
		}
	}
	for(int i = 0; i < size; ++i)
	{
		spheres.radiuses.push_back(2.f);
	}

	spheres.count = size;

	KDTree tree;
	tree.build(spheres);
	std::cout << tree.getSize();
	return 0;
}
