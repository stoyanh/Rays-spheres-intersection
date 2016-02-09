#include "KDTree.h"
#include <stack>
#include <thread>
#include <algorithm>
#include <numeric>
#include <xmmintrin.h>
#include "Utils.h"

using std::stack;
using std::thread;
using std::sort;
using std::iota;
using std::min;
using std::max;
using std::numeric_limits;

unsigned KDTree::leftChild(const unsigned nodeIdx) const
{
	return nodeIdx + offset(nodeIdx) / sizeof(KDNode);
}
unsigned KDTree::rightChild(const unsigned nodeIdx) const
{
	return leftChild(nodeIdx) + 1;
}

void KDTree::findMinMax(const Spheres& spheres, Axis axis, float& min, float& max) const
{
	min = spheres.centerCoords[axis][0] - spheres.radiuses[0];
	max = spheres.centerCoords[axis][0] + spheres.radiuses[0];

	for(int i = 1; i < spheres.count; ++i)
	{
		if(spheres.centerCoords[axis][i] - spheres.radiuses[i] < min)
		{
			min = spheres.centerCoords[axis][i] - spheres.radiuses[i];
		}

		if(spheres.centerCoords[axis][i] + spheres.radiuses[i] > max)
		{
			max = spheres.centerCoords[axis][i] + spheres.radiuses[i];
		}
	}
}

BoundingBox KDTree::createBoundingBox(const Spheres& spheres) const
{
	BoundingBox bbox;
	float minCoords[3], maxCoords[3];
	thread xValues(&KDTree::findMinMax, this, std::cref(spheres),
			AXIS_X, std::ref(minCoords[0]), std::ref(maxCoords[0]));
	thread yValues(&KDTree::findMinMax, this, std::cref(spheres),
				AXIS_Y, std::ref(minCoords[1]), std::ref(maxCoords[2]));

	findMinMax(spheres, AXIS_Z, minCoords[2], minCoords[2]);

	xValues.join();
	yValues.join();

	bbox.vmin = Vec3(minCoords[0], minCoords[1], minCoords[2]);
	bbox.vmax = Vec3(maxCoords[0], maxCoords[1], maxCoords[2]);

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
		sahCosts[i].cost = numeric_limits<float>::max();
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
	sceneBBox = createBoundingBox(spheres);

	int axis = static_cast<int>(AXIS_X);

	KDNode root;
	nodes.push_back(root);

	StackNode stackNode;
	stackNode.bbox = sceneBBox;
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


void BoundingBox::intersectRay(const Ray& ray, float& tnear, float& tfar) const
{
	Vec3 invDir;
	invDir.vec = 1.f / ray.direction.vec;

	float t1 = (vmin.x - ray.origin.x) * invDir.x;
	float t2 = (vmax.x - ray.origin.x) * invDir.x;
	float t3 = (vmin.y - ray.origin.y) * invDir.y;
	float t4 = (vmax.y - ray.origin.y) * invDir.y;
	float t5 = (vmin.z - ray.origin.z) * invDir.z;
	float t6 = (vmax.z - ray.origin.z) * invDir.z;

	tnear = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
	tfar = min(min(max(t1, t2), max(t3, t4)), max(t6, t6));
}

IntersectionData KDTree::intersectRay(const Ray& ray) const
{
	IntersectionData data;
	Ray rayCpy = ray;
	rayCpy.direction = normalize(rayCpy.direction);

	TraversalNode node;
	node.tnear = 0.f;
	node.tfar = std::numeric_limits<float>::max();

	sceneBBox.intersectRay(rayCpy, node.tnear, node.tfar);

	if(node.tnear > node.tfar)
	{
		data.intersection = false;
		return data;
	}

	data.intersection = false;
	stack<TraversalNode> st;

	float invRayDir[3] = { 1.f / ray.direction[0], 1.f / ray.direction[1], 1.f / ray.direction[3] };

	node.nodeIdx = 0; // root
	while(1)
	{
		while(!isLeaf(node.nodeIdx))
		{
			Axis axis = static_cast<Axis>(splittingAxis(node.nodeIdx));
			float tsplit = (nodes[node.nodeIdx].inner.splitCoord - ray.origin[axis]) * invRayDir[axis];
			if(tsplit <= node.tnear)
			{
				node.nodeIdx = rightChild(node.nodeIdx);
			}
			else if(tsplit >= node.tfar)
			{
				node.nodeIdx = leftChild(node.nodeIdx);
			}
			else
			{
				TraversalNode traversalNode;
				traversalNode.nodeIdx = rightChild(node.nodeIdx);
				traversalNode.tnear = tsplit;
				traversalNode.tfar = node.tfar;
				st.push(traversalNode);

				node.nodeIdx = leftChild(node.nodeIdx);
			}
		}

		data = Intersection::intersectRaySpheres(ray, leavesChildren[leafChildrenIdx(node.nodeIdx)], spheres);
		if(data.intersection)
		{
			return data;
		}

		if(st.empty())
		{
			data.intersection = false;
			return data;
		}

		node = st.top();
		st.pop();
	}
}

