#ifndef KDTREE_H_
#define KDTREE_H_

#include "Common.h"
#include <limits>
#include <memory>

using std::vector;
using std::unique_ptr;

struct KDLeaf
{
	unsigned flagAndOffset;
	/**
	 * bits 0..30 Children index in leavesChildren
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

struct StackNode
{
	int nodeIdx;
	BoundingBox bbox;
	vector<int> sphereIndices;
};

class KDTree
{
public:
	void build(const Spheres& spheres);
	int getSize()const { return nodes.size(); }
	int getLeaves()const { return leaves; }
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
	SAHCost chooseSplittingAxis(const Spheres& spheres, const BoundingBox& bbox) const;
	int spheresCount(const Spheres &spheres, Axis axis, const float from, const float to) const;
	void minSAHCost(const Spheres& spheres, const BoundingBox& bbox, Axis axis, SAHCost& sahCost) const;
	void initInnerNode(unsigned nodeIdx, Axis axis, float splitPos, unsigned firstChiledIdx);
	void initLeafNode(unsigned nodeIdx, unsigned dataIdx);

	static const int maxSpheresInLeaf = 12;
	static const int maxNodes = 6000000;

	vector<KDNode> nodes;
	vector<vector<int>> leavesChildren;
	Spheres spheres;
	int leaves;
};



#endif /* KDTREE_H_ */
