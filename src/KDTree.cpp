#include "KDtree.h"
#include <stack>
#include <xmmintrin.h>

unsigned KDTree::leftChild(unsigned index) const
{
	return index + offset(nodes[index]) / sizeof(KDNode);
}

BoundingBox KDTree::createBoundingBox(const Spheres& spheres) const
{
	BoundingBox bbox;
	float minXIndex = 0;
	float minYIndex = 0;
	float minZIndex = 0;

	__m128 min = _mm_set_ps(spheres.centerX[0], spheres.centerY[0], spheres.centerZ[0], 0.f);
	__m128 max = min;

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

Axis KDTree::chooseSplittingAxis(const Spheres& spheres, float& where) const
{
	Axis axis;
	return axis;
}

void KDTree::build(const Spheres& spheres)
{
	BoundingBox bbox = createBoundingBox(spheres);

}
