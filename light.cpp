#include "light.h"

// Light methods
Directional::Directional(Eigen::Vector3d direction, Eigen::Array3d color)
{
	kind = "directional";
	v0 = direction;
	c = color;
}

PointLight::PointLight(Eigen::Vector3d origin, Eigen::Array3d color)
{
	kind = "point";
	v0 = origin;
	c = color;
}


QuadLight::QuadLight(Eigen::Vector3d origin, Eigen::Vector3d edge1, Eigen::Vector3d edge2, Eigen::Array3d color)
{
	kind = "quad";
	va = origin;
	vb = edge1 + va;
	vc = edge2 + va;
	vd = edge1 + edge2 + va;
	e1 = edge1;
	e2 = edge2;
	c = color;
	area = edge1.cross(edge2).norm();
	n = edge1.cross(edge2).normalized();
}

double QuadLight::intersect(Ray ray) {

	//ray-plane intersection
	double t = ray.pt.dot(n);
	if (abs(t) < eps) {
		return -1;
	}

	t = (va - ray.p0).dot(n) / t;
	if (t < eps) {
		return -1;
	}
	//point inside triangle
	Eigen::Vector3d bary = barycentric(Eigen::Vector3d(ray.p0 + t * ray.pt), 1);
	if (bary[0] != -1) {
		return t;
	}
	bary = barycentric(Eigen::Vector3d(ray.p0 + t * ray.pt), 2);
	if (bary[0] == -1) {
		t = -1;
	}
	return t;
}

Eigen::Vector3d QuadLight::barycentric(Eigen::Vector3d point, int partition) {
	Eigen::Vector3d t0, t1, t2;
	if (partition == 1) {
		t0 = vd - va;
		t1 = vb - va;
		t2 = point - va;
	}
	else {
		t0 = vc - va;
		t1 = vd - va;
		t2 = point - va;
	}

	double d00, d01, d11, d20, d21, denom;
	d00 = t0.dot(t0);
	d01 = t0.dot(t1);
	d11 = t1.dot(t1);
	d20 = t2.dot(t0);
	d21 = t2.dot(t1);
	denom = d00 * d11 - d01 * d01;
	double u, v, w;
	v = (d11 * d20 - d01 * d21) / denom;
	w = (d00 * d21 - d01 * d20) / denom;
	u = 1.0f - v - w;
	if (u < 0 || v < 0 || w < 0) {
		return Eigen::Vector3d(-1, -1, -1);
	}
	return Eigen::Vector3d(u, v, w);
}