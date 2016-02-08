#ifndef KDTREE_H_
#define KDTREE_H_

#include "Common.h"
#include <limits>

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
	Vec3 vmin;
	Vec3 vmax;
};

class KDTree
{
public:
	void build(const Spheres& spheres);

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

	unsigned leftChild(unsigned index) const;

	BoundingBox createBoundingBox(const Spheres& spheres) const;
	Axis chooseSplittingAxis(const Spheres& spheres, float& where) const;

	static const int maxSpheresInBox = 8;
	std::vector<KDNode> nodes;
};



#endif /* KDTREE_H_ */
