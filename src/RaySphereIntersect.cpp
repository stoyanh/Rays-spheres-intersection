#include "RaySphereIntersect.h"
#include "KDTree.h"
#include <thread>

using std::thread;

const int predefinedNoThreads = 100000;
const int predefined1Thread = 1000000;


void intersectSpheres(const Rays& rays, const KDTree& tree, int from, int count, vector<IntersectionData>& result)
{
	int end = from + count;
	for(int i = from; i < end; ++i)
	{
		result[i] = tree.intersectRay(rays.rays[i]);
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

	int raysCount = rays.rays.size();
	intersections.resize(raysCount);

	if(raysCount <= predefinedNoThreads)
	{
		intersectSpheres(rays, tree, 0, raysCount, intersections);
	}
	else if(raysCount <= predefined1Thread)
	{
		int half = raysCount / 2;
		thread th(intersectSpheres, rays, std::cref(tree), 0, half, std::ref(intersections));
		intersectSpheres(rays, tree, half, raysCount - half, intersections);
		th.join();
	}
	else
	{
		int third = raysCount / 3;
		thread t1, t2;
		t1 = thread(intersectSpheres, rays, std::cref(tree), 0, third, std::ref(intersections));
		t2 = thread(intersectSpheres, rays, std::cref(tree), third, third, std::ref(intersections));
		intersectSpheres(rays, tree, 2 * third, raysCount - 2 * third, intersections);
		t1.join();
		t2.join();
	}


}
