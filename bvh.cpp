#include "bvh.h"

constexpr auto LEAF_PRIM_COUNT = 4;


BVHnode::BVHnode(Eigen::AlignedBox3d bbox, std::shared_ptr<BVHnode> l, std::shared_ptr<BVHnode> r)
{
	box = bbox;
	left = l;
	right = r;
}

BVHnode::BVHnode(Eigen::AlignedBox3d bbox, std::vector<std::shared_ptr<Primitive>> prims)
{
	box = bbox;
	left = nullptr;
	right = nullptr;
	primitives = prims;
}

double BVHnode::intersect(Ray ray)
{

	return -1;
}

Axis findAxis(std::vector<std::shared_ptr<Primitive>> primitives) {
	double min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
	for (auto p: primitives)
	{
		Eigen::Vector3d c = p->bbox.center();
		if (c[0] < min_x) {
			min_x = c[0];
		}
		else if (c[0] > max_x)
		{
			max_x = c[0];
		}

		if (c[1] < min_y) {
			min_y = c[1];
		}
		else if (c[1] > max_y)
		{
			max_y = c[1];
		}

		if (c[2] < min_z) {
			min_z = c[2];
		}
		else if (c[2] > max_z)
		{
			max_z = c[2];
		}
	}
	min_x = max_x - min_x;
	min_y = max_y - min_y;
	min_z = max_z - min_z;
	double max_value = std::max(std::max(min_x, min_y), min_z);
	if (abs(min_x - max_value) < eps) {
		return Axis::x;
	}
	else if (abs(min_y - max_value) < eps) {
		return Axis::y;
	}
	else {
		return Axis::z;
	}
}

bool cmp_bbox_x(std::shared_ptr<Primitive> p1, std::shared_ptr<Primitive> p2) {
	return p1->bbox.center()[Axis::x] < p2->bbox.center()[Axis::x];
}

bool cmp_bbox_y(std::shared_ptr<Primitive> p1, std::shared_ptr<Primitive> p2) {
	return p1->bbox.center()[Axis::y] < p2->bbox.center()[Axis::y];
}

bool cmp_bbox_z(std::shared_ptr<Primitive> p1, std::shared_ptr<Primitive> p2) {
	return p1->bbox.center()[Axis::z] < p2->bbox.center()[Axis::z];
}

std::vector<std::shared_ptr<Primitive>> reorder(std::vector<std::shared_ptr<Primitive>> primitives, Axis ax) {
	std::vector<std::shared_ptr<Primitive>> primitives_sorted = primitives;
	if (ax == Axis::x) {
		std::sort(primitives_sorted.begin(), primitives_sorted.end(), cmp_bbox_x);
	}
	else if (ax == Axis::y) {
		std::sort(primitives_sorted.begin(), primitives_sorted.end(), cmp_bbox_y);
	}
	else {
		std::sort(primitives_sorted.begin(), primitives_sorted.end(), cmp_bbox_z);
	}
	return primitives_sorted;
}

int splitInd(std::vector<std::shared_ptr<Primitive>> primitives)
{
// TODO: implement SAH
	return primitives.size()/2;
}


std::shared_ptr<BVHnode> buildTree(std::vector<std::shared_ptr<Primitive>> primitives)
{
	Eigen::AlignedBox3d bbox;
	assert(bbox.isEmpty());

	if (primitives.size() <= LEAF_PRIM_COUNT) {
		for (auto p : primitives) {
			bbox.extend(p->bbox);
		}
		return std::make_shared<BVHnode>(bbox, primitives);
	}

	Axis ax = findAxis(primitives);
	primitives = reorder(primitives, ax);
	int split = splitInd(primitives);
	std::vector<std::shared_ptr<Primitive>> priml(primitives.begin(), primitives.begin() + split);
	std::vector<std::shared_ptr<Primitive>> primr(primitives.begin() + split, primitives.end());
	std::shared_ptr<BVHnode> left(buildTree(priml));
	std::shared_ptr<BVHnode> right(buildTree(primr));
	bbox.extend(left->box);
	bbox.extend(right->box);
	return std::make_shared<BVHnode>(bbox, left, right);
}
