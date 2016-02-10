#include "RaySphereIntersect.h"
#include "KDTree.h"
#include <thread>
#include <iostream>

using std::thread;

const int predefinedCount = 10000;

void intersectSpheres(const Rays& rays, const KDTree& tree, int from, int count, vector<IntersectionData>& result)
{
	int end = from + count;
	for(int i = from; i < end; ++i)
	{
		result.push_back(tree.intersectRay(rays.rays[i]));
	}
}

void intersectRaySpheres(const Ray& ray, const Spheres& spheres, IntersectionData& data)
{
	KDTree tree;
	tree.build(spheres);

	data = tree.intersectRay(ray);
}

void intersectRaysSpheres(const Rays& rays, const Spheres& spheres, std::vector<IntersectionData>& intersections)
{
	KDTree tree;
	tree.build(spheres);
	std::cout << "Tree built\n";

	thread t[3];

	int raysCount = rays.rays.size();
	int remaining = raysCount;
	int start = 0;

	vector<IntersectionData> results[3];
	for(int i = 0; i < 2; ++i)
	{
		int execute = remaining / (2 - i);
		t[i] = thread(intersectSpheres, rays, std::cref(tree), start, execute, std::ref(results[i]));
		remaining -= execute;
		start += execute;
	}

	intersectSpheres(rays, tree, start, remaining, results[2]);
	t[0].join();
	t[1].join();
	//t[2].join();

	int count[3];
	for(int i = 0; i < 3; ++i)
	{
		count[i] = 0;
		for(int j = 0; j < results[i].size(); ++j)
		{
			if(results[i][j].intersection) ++count[i];
		}

		std::cout << "thread " << i + 1 << " " << count[i] << " intersections" << std::endl;
	}

}
