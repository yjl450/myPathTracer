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

bool bbox_hit(Ray ray, Eigen::AlignedBox3d bbox) {
	Eigen::Vector3d maxpoint = bbox.max();
	Eigen::Vector3d minpoint = bbox.min();
	double t_min, t_max, tx1, tx2, ty1, ty2, tz1, tz2;
	tx1 = (minpoint[0] - ray.p0[0]) * ray.rpt[0];
	tx2 = (maxpoint[0] - ray.p0[0]) * ray.rpt[0];

	t_min = std::min(tx1, tx2);
	t_max = std::max(tx1, tx2);

	ty1 = (minpoint[1] - ray.p0[1]) * ray.rpt[1];
	ty2 = (maxpoint[1] - ray.p0[1]) * ray.rpt[1];

	t_min = std::max(t_min, std::min(ty1, ty2));
	t_max = std::min(t_max, std::max(ty1, ty2));

	tz1 = (minpoint[2] - ray.p0[2]) * ray.rpt[2];
	tz2 = (maxpoint[2] - ray.p0[2]) * ray.rpt[2];

	t_min = std::max(t_min, std::min(tz1, tz2));
	t_max = std::min(t_max, std::max(tz1, tz2));
	return (t_max > 0 && t_max >= t_min);
}

Intersection BVHnode::intersect(Ray ray)
{
	if (!bbox_hit(ray, box)) {
		return { -1 , nullptr };
	}
	if (left == nullptr && right == nullptr) {
		double min_t = -1;
		double temp_t = -1;
		std::shared_ptr<Primitive> prim = nullptr;
		for (auto p : primitives) {
			temp_t = p->intersect(ray);
			if (temp_t > 0 && (min_t == -1 || temp_t < min_t)) {
				min_t = temp_t;
				prim = p;
			}
		}
		return Intersection{ min_t, prim };
	}
	Intersection l_intersect = left->intersect(ray);
	Intersection r_intersect = right->intersect(ray);
	if (l_intersect.t != -1 && r_intersect.t != -1) {
		double min_t = std::min(l_intersect.t, r_intersect.t);
		if (l_intersect.t == min_t) {
			return l_intersect;
		}
		else {
			return r_intersect;
		}
	}
	else if (l_intersect.t == -1 && r_intersect.t == -1) {
		return Intersection{ -1, nullptr };
	}
	else if (l_intersect.t == -1) {
		return r_intersect;
	}
	else {
		return l_intersect;
	}
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
		for (std::shared_ptr<Primitive> p : primitives) {
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
