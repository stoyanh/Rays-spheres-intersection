#include "KDtree.h"
#include <stack>
#include <xmmintrin.h>

unsigned KDTree::leftChild(const unsigned index) const
{
	return index + offset(nodes[index]) / sizeof(KDNode);
}

BoundingBox KDTree::createBoundingBox(const Spheres& spheres) const
{
	BoundingBox bbox;
	float minXIndex = 0;
	float minYIndex = 0;
	float minZIndex = 0;

	float maxXIndex = minXIndex, maxYIndex = minYIndex, maxZIndex = minZIndex;

	for(auto i = 1; i < spheres.centerX.size(); ++i)
	{
		minXIndex = spheres.centerX[i] < spheres.centerX[minXIndex] ? i : minXIndex;
		minYIndex = spheres.centerY[i] < spheres.centerY[minYIndex] ? i : minYIndex;
		minZIndex = spheres.centerZ[i] < spheres.centerZ[minZIndex] ? i : minZIndex;

		maxXIndex = spheres.centerX[i] > spheres.centerX[maxXIndex] ? i : maxXIndex;
		maxYIndex = spheres.centerY[i] > spheres.centerY[maxYIndex] ? i : maxYIndex;
		maxZIndex = spheres.centerZ[i] > spheres.centerZ[maxZIndex] ? i : maxZIndex;
	}
	bbox.vmin = Vec3(spheres.centerX[minXIndex] - spheres.radiuses[minXIndex],
					 spheres.centerY[minYIndex] - spheres.radiuses[minYIndex],
					 spheres.centerZ[minZIndex] - spheres.radiuses[minZIndex]);

	bbox.vmax = Vec3(spheres.centerX[maxXIndex] + spheres.radiuses[maxXIndex],
			         spheres.centerY[maxYIndex] + spheres.radiuses[maxYIndex],
					 spheres.centerZ[maxZIndex] + spheres.radiuses[maxZIndex]);
	return bbox;
}

BoundingBox KDTree::createBoundingBox(const vector<Sphere>& spheres) const
{
	BoundingBox bbox;

	Vec3 min, max;

	for(int i = 0; i < 3; ++i)
	{
		min[i] = spheres[0].center[i];
		max[i] = min[i];
	}

	for(auto i = 0; i < spheres.size(); ++i)
	{
		for(int j = 0; j < 3; ++j)
		{
			min[j] = spheres[i].center[j] < min[j] ? spheres[i].center[j] : min[j];
			max[j] = spheres[i].center[j] > max[j] ? spheres[i].center[j] : max[j];
		}
	}

	bbox.vmin = min;
	bbox.vmax = max;

	return bbox;
}

float KDTree::surface(const BoundingBox& box) const
{
	float x = box.vmax.x - box.vmin.x;
	float y = box.vmax.y - box.vmin.y;
	float z = box.vmax.z - box.vmin.z;

	return x * y + x * z + y * z;
}

float KDTree::surfaceAreaHeuristic(const BoundingBox& bbox, Axis axis, float splitPoint, int spheresLeft, int spheresRight) const
{
	BoundingBox left, right;
	bbox.split(axis, splitPoint, left, right);

	float surfaceLeft = surface(left);
	float surfaceRight = surface(right);
	float surfaceWhole = surface(bbox);

	float costLeft = spheresLeft * (surfaceLeft / surfaceWhole);
	float costRight = spheresRight * (surfaceRight / surfaceWhole);

	return costLeft + costRight;
}

int KDTree::spheresCount(const vector<Sphere>& spheres, Axis axis, const float from, const float to) const
{
	int count = 0;
	for(auto i = 0; i < spheres.size(); ++i)
	{
		float radius = spheres[i].radius;
		if(spheres[i].center[axis] -  radius >= from && spheres[i].center[axis] + radius <= to)
		{
			++count;
		}
	}

	return count;
}

SAHCost KDTree::chooseSplittingAxis(const vector<Sphere>& spheres, const BoundingBox& bbox) const
{
	SAHCost sahCost;
	sahCost.cost = std::numeric_limits<float>::max();

	for(int i = 0; i < 3; ++i)
	{
		Axis axis = static_cast<Axis>(i);
		for(auto j = 0; j < spheres.size(); ++j)
		{
			float leftPlane = spheres[j].center[i] - spheres[j].radius;
			float rightPlane = spheres[j].center[i] + spheres[j].radius;
			if(!bbox.inBoundingBox(leftPlane, axis))
			{
				continue;
			}

			BoundingBox left, right;
			bbox.split(axis, leftPlane, left, right);

			int spheresLeft = spheresCount(spheres, axis, bbox.vmin[axis], leftPlane);
			int spheresRight = spheres.size() - spheresLeft;

			float sah = surfaceAreaHeuristic(bbox, axis, leftPlane, spheresLeft, spheresRight);
			if(sah < sahCost.cost)
			{
				sahCost.cost = sah;
				sahCost.splitAxis = axis;
				sahCost.splitPos = leftPlane;
			}

			if(!bbox.inBoundingBox(rightPlane, axis))
			{
				continue;
			}

			bbox.split(axis, rightPlane, left, right);
			spheresLeft = spheresCount(spheres, axis, bbox.vmin[axis], rightPlane);
			spheresRight = spheres.size() - spheresLeft;

			sah = surfaceAreaHeuristic(bbox, axis, rightPlane, spheresLeft, spheresRight);
			if(sah < sahCost.cost)
			{
				sahCost.cost = sah;
				sahCost.splitAxis = axis;
				sahCost.splitPos = rightPlane;
			}
		}
	}

	return sahCost;
}

void KDTree::build(const vector<Sphere>& spheres)
{
	BoundingBox bbox = createBoundingBox(spheres);
	SAHCost sahCost = chooseSplittingAxis(spheres, bbox);

}
