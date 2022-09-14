#pragma once
#include <vector>
#include <cassert>
#include "primitive.h"

enum Axis { x = 0, y = 1, z = 2 };

class BVHnode {
public:
	Eigen::AlignedBox3d box;
	std::shared_ptr<BVHnode> left;
	std::shared_ptr<BVHnode> right;
	std::vector<std::shared_ptr<Primitive>> primitives;

	BVHnode(Eigen::AlignedBox3d bbox, std::shared_ptr<BVHnode> l, std::shared_ptr<BVHnode> r);
	BVHnode(Eigen::AlignedBox3d bbox, std::vector<std::shared_ptr<Primitive>> prims);
	Intersection intersect(Ray ray);
};

Axis findAxis(std::vector<std::shared_ptr<Primitive>> primitives);

std::vector<std::shared_ptr<Primitive>> reorder(std::vector<std::shared_ptr<Primitive>> primitives, Axis ax);

int splitInd(std::vector<std::shared_ptr<Primitive>> primitives);

std::shared_ptr<BVHnode> buildTree(std::vector<std::shared_ptr<Primitive>> primitives);