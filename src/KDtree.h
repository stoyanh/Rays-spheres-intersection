#ifndef KDTREE_H_
#define KDTREE_H_

#include "Common.h"
#include <limits>

using std::vector;

struct KDLeaf
{
	unsigned flagAndOffset;
	/**
	 * bits 0..30 offset
	 * bit 31 flag whether the node is leaf
	 * */
};

struct KDInner
{
	unsigned flagDimAndOffset;
	float splitCoord;
	/**
	 * bits 0..1 splitting dim
	 * bits 2..30 offset
	 * bit 31 flag whether the node is leaf
	 * */
};

struct KDChildrenPointer
{

};

union KDNode
{
	KDInner inner;
	KDLeaf leaf;
};

enum Axis
{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,
	AXIS_NONE,
};

enum SpherePosition
{
	LEFT,
	RIGHT,
	INTERSECT,
};

struct SAHCost
{
	float cost;
	Axis splitAxis;
	float splitPos;
};

struct StackNode
{

};

struct BoundingBox
{
	BoundingBox()
	{
		float min = std::numeric_limits<float>::lowest();
		float max = std::numeric_limits<float>::max();
		vmin = Vec3(min, min, min);
		vmax = Vec3(max, max, max);
	}

	void split(Axis axis, float where, BoundingBox& left, BoundingBox& right) const
	{
		left = *this;
		right = *this;
		left.vmax[axis] = where;
		right.vmin[axis] = where;

	}

	bool inBoundingBox(float coord, Axis axis) const
	{
		return vmin[axis] <= coord && vmax[axis] >= coord;
	}
	Vec3 vmin;
	Vec3 vmax;
};

class KDTree
{
public:
	void build(const vector<Sphere>& spheres);

private:
	bool isLeaf(const KDNode& node) const
	{
		return node.inner.flagDimAndOffset & static_cast<unsigned>(1 << 31);
	}

	unsigned offset(const KDNode& node) const
	{
		return node.inner.flagDimAndOffset & 0x7FFFFFFC;
	}

	int splittingDimension(const KDNode& node) const
	{
		return node.inner.flagDimAndOffset & 0x3;
	}

	float surface(const BoundingBox& box) const;

	float surfaceAreaHeuristic(const BoundingBox& bbox, Axis axis, float spiltPoint,
			int spheresLeft, int spheresRight) const;

	unsigned leftChild(const unsigned index) const;

	BoundingBox createBoundingBox(const Spheres& spheres) const;
	BoundingBox createBoundingBox(const vector<Sphere>& spheres) const;
	SAHCost chooseSplittingAxis(const vector<Sphere>& spheres, const BoundingBox& bbox) const;
	int spheresCount(const vector<Sphere>&spheres, Axis axis, const float from, const float to) const;

	static const int maxSpheresInBox = 8;
	std::vector<KDNode> nodes;
};



#endif /* KDTREE_H_ */
