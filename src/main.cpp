#include <iostream>
#include "KDTree.h"
#include <xmmintrin.h>
#include "Vec3.h"
#include <cstdlib>
#include "RaySphereIntersect.h"

inline float Rand()
{
	return static_cast<float>(rand()) / 1000.f;
}

int main()
{
	Spheres spheres;
	const int size = 100000;
	for(int i = 0; i < 3; ++i)
	{
		for(int j = 0; j < size; ++j)
		{
			spheres.centerCoords[i].push_back(j);//j + 10);
		}
	}

	for(int i = 0; i < size; ++i)
	{
		spheres.radiuses.push_back(5.f);//2.f * i + 1.f);
	}

	spheres.count = size;

	//KDTree tree;
	//tree.build(spheres);
	//std::cout << "Tree built" << std::endl;

	Rays rays;
	//for(int i = 0; i < 10000; ++i)
	//{
		IntersectionData data;
		Ray ray(Vec3(100, 100, 100), Vec3(0.5f, 0.5f, 0.5f));
		intersectRaySpheres(ray, spheres, data);
		//rays.rays.push_back(Ray(Vec3(Rand(), Rand(), Rand()), Vec3(Rand(), Rand(), Rand())));
		//rays.rays.push_back(Ray(Vec3(100, 100, 100), Vec3(0.5f, 0.5f, 0.5f)));
	//}

	//vector<IntersectionData> data;
	//intersectRaysSpheres(rays, spheres, data);

//	int count = 0;
//	for(int i = 0; i < data.size(); ++i)
//	{
//		if(data[i].intersection) ++count;
//	}
//
//	std::cout << count << std::endl;
	return 0;
}
