#include "KDtree.h"
#include <stack>
#include <thread>
#include <algorithm>
#include <numeric>

using std::stack;
using std::thread;
using std::sort;
using std::iota;

unsigned KDTree::leftChild(const unsigned index) const
{
	return index + offset(nodes[index]) / sizeof(KDNode);
}

BoundingBox KDTree::createBoundingBox(const Spheres& spheres) const
{
	BoundingBox bbox;
	float minCoordIdx[3], maxCoordIdx[3];
	for(int i = 0; i < 3; ++i)
	{
		minCoordIdx[i] = 0;
		maxCoordIdx[i] = minCoordIdx[i];
	}

	for(int i = 0; i < 3; ++i)
	{
		for(int j = 1; j < spheres.centerCoords[i].size(); ++j)
		{
			minCoordIdx[i] = spheres.centerCoords[i][j] < spheres.centerCoords[i][minCoordIdx[i]] ? j : minCoordIdx[i];
			maxCoordIdx[i] = spheres.centerCoords[i][j] > spheres.centerCoords[i][maxCoordIdx[i]] ? j : maxCoordIdx[i];
		}
	}

	float minRadiuses[3], maxRadiuses[3], minCoord[3], maxCoord[3];
	for(int i = 0; i < 3; ++i)
	{
		minRadiuses[i] = spheres.radiuses[minCoordIdx[i]];
		maxRadiuses[i] = spheres.radiuses[maxCoordIdx[i]];

		minCoord[i] = spheres.centerCoords[i][minCoordIdx[i]] - minRadiuses[i];
		maxCoord[i] = spheres.centerCoords[i][maxCoordIdx[i]] + maxRadiuses[i];
	}

	bbox.vmin = Vec3(minCoord[0], minCoord[1], minCoord[2]);
	bbox.vmax = Vec3(maxCoord[0], maxCoord[1], maxCoord[2]);

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

int KDTree::spheresCount(const Spheres& spheres, Axis axis, const float from, const float to) const
{
	int count = 0;
	for(auto i = 0; i < spheres.count; ++i)
	{
		float radius = spheres.radiuses[i];
		if(spheres.centerCoords[axis][i] -  radius >= from && spheres.centerCoords[axis][i] + radius <= to)
		{
			++count;
		}
	}

	return count;
}

void KDTree::minSAHCost(const Spheres& spheres, const BoundingBox& bbox, Axis axis, SAHCost& sahCost) const
{
	for(auto i = 0; i < spheres.count; ++i)
	{
		float leftPlane = spheres.centerCoords[axis][i] - spheres.radiuses[i];
		float rightPlane = spheres.centerCoords[axis][i] + spheres.radiuses[i];
		if(!bbox.inBoundingBox(leftPlane, axis))
		{
			continue;
		}

		BoundingBox left, right;
		bbox.split(axis, leftPlane, left, right);

		int spheresLeft = spheresCount(spheres, axis, bbox.vmin[axis], leftPlane);
		int spheresRight = spheresCount(spheres, axis, leftPlane, bbox.vmax[axis]);

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
		spheresRight = spheresCount(spheres, axis, rightPlane, bbox.vmax[axis]);

		sah = surfaceAreaHeuristic(bbox, axis, rightPlane, spheresLeft, spheresRight);
		if(sah < sahCost.cost)
		{
			sahCost.cost = sah;
			sahCost.splitAxis = axis;
			sahCost.splitPos = rightPlane;
		}
	}
}

SAHCost KDTree::chooseSplittingAxis(const Spheres& spheres, const BoundingBox& bbox) const
{
	SAHCost sahCosts[3];
	for(int i = 0; i < 3; ++i)
	{
		sahCosts[i].cost = std::numeric_limits<float>::max();
	}

	thread xCost(&KDTree::minSAHCost, this, std::cref(spheres), std::cref(bbox), AXIS_X, std::ref(sahCosts[0]));
	thread yCost(&KDTree::minSAHCost, this, std::cref(spheres), std::cref(bbox), AXIS_Y, std::ref(sahCosts[1]));
	minSAHCost(spheres, bbox, AXIS_Z, sahCosts[2]);

	xCost.join();
	yCost.join();

	SAHCost res = sahCosts[0];
	if(res.cost < sahCosts[1].cost) res = sahCosts[1];
	if(res.cost < sahCosts[2].cost) res = sahCosts[2];

	return res;
}

void KDTree::initInnerNode(unsigned nodeIdx, Axis axis, float splitPos, unsigned firstChildIdx)
{
	nodes[nodeIdx].inner.flagDimAndOffset = 0;
	nodes[nodeIdx].inner.flagDimAndOffset |= (firstChildIdx - nodeIdx) * sizeof(KDNode);
	nodes[nodeIdx].inner.flagDimAndOffset |= static_cast<unsigned>(axis);
	nodes[nodeIdx].inner.splitCoord = splitPos;
}

void KDTree::initLeafNode(unsigned nodeIdx, unsigned dataIdx)
{
	nodes[nodeIdx].leaf.flagAndOffset = 0;
	nodes[nodeIdx].leaf.flagAndOffset |= static_cast<unsigned>(1 << 31);
	nodes[nodeIdx].leaf.flagAndOffset |= dataIdx;
}

void KDTree::build(const Spheres& spheres)
{
	leaves = 0;
	this->spheres = spheres;
	BoundingBox bbox = createBoundingBox(spheres);
	int axis = static_cast<int>(AXIS_X);

	KDNode root;
	nodes.push_back(root);

	StackNode stackNode;
	stackNode.bbox = bbox;
	stackNode.nodeIdx = nodes.size() - 1;
	stackNode.sphereIndices.resize(spheres.count);
	iota(stackNode.sphereIndices.begin(), stackNode.sphereIndices.end(), 0);

	stack<StackNode> st;
	st.push(stackNode);

	while(!st.empty())
	{
		StackNode stackNode = st.top();
		st.pop();

		if(stackNode.sphereIndices.size() <= maxSpheresInLeaf || nodes.size() >= maxNodes)
		{
			leavesChildren.push_back(stackNode.sphereIndices);
			initLeafNode(stackNode.nodeIdx, leavesChildren.size() - 1);
			++leaves;
			continue;
		}

		axis = axis % 3;
		float leftLimit = stackNode.bbox.vmin[axis];
		float rightLimit = stackNode.bbox.vmax[axis];

		float splitPos = (leftLimit + rightLimit) * 0.5f; // TODO use SAH

		Axis splitAxis = static_cast<Axis>(axis);

		StackNode child1StackNode, child2StackNode;
		nodes.push_back(KDNode());
		child1StackNode.nodeIdx = nodes.size() - 1;
		nodes.push_back(KDNode());
		child2StackNode.nodeIdx = nodes.size() - 1;

		stackNode.bbox.split(splitAxis, splitPos, child1StackNode.bbox, child2StackNode.bbox);

		for(int i = 0; i < stackNode.sphereIndices.size(); ++i)
		{
			if(spheres.centerCoords[axis][stackNode.sphereIndices[i]] < splitPos)
			{
				child1StackNode.sphereIndices.push_back(stackNode.sphereIndices[i]);
			}
			else
			{
				child2StackNode.sphereIndices.push_back(stackNode.sphereIndices[i]);
			}
		}
		initInnerNode(stackNode.nodeIdx, splitAxis, splitPos, child1StackNode.nodeIdx);
		st.push(child2StackNode);
		st.push(child1StackNode);

		axis += 1;
	}
}
