#ifndef KDTREE_H_
#define KDTREE_H_

struct KDLeaf
{
};

struct KDInner
{
	unsigned flagDimAndOffset;
	float splitCoord;
};

union KDNode
{
	KDInner inner;
	KDLeaf leaf;
};


class KDTree
{

};



#endif /* KDTREE_H_ */
