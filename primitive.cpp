#include "primitive.h"

// Ray methods
Ray::Ray(Eigen::Vector3d v0, Eigen::Vector3d vt)
{
	p0 = v0;
	pt = vt.normalized();
}

// Sphere methods
Sphere::Sphere(Eigen::Vector3d center, double radius, Material material, Eigen::Transform<double, 3, Eigen::Affine> transformation, bool trans_flag)
{
	o = center;
	r = radius;
	mat = material;
	if (trans_flag) {
		transformed = true;
		trans = transformation;
	}
}

double Sphere::intersect(Ray ray)
{
	Ray newRay = ray;
	if (transformed) {
		newRay.p0 = trans.inverse() * ray.p0;
		newRay.pt = trans.inverse().linear() * ray.pt;
	}
	double a, b, d;
	a = newRay.pt.dot(newRay.pt);
	b = newRay.pt.dot(newRay.p0 - o) * 2;
	d = b * b - 4 * a * ((newRay.p0 - o).dot(newRay.p0 - o) - r * r);
	if (d < 0) return -1;
	double t1, t2;
	d = sqrt(d);
	t1 = (-b - d) / (2 * a);
	t2 = (-b + d) / (2 * a);
	if (t1 > 0 && t2 > 0) return (t1 < t2) ? t1 : t2;
	else if (t1 > 0) return t1;
	else if (t2 > 0) return t2;
	else return -1;
}

Eigen::Vector3d Sphere::normal(Eigen::Vector3d point)
{
	
	if (!transformed) {
		Eigen::Vector3d normal = (point - o).normalized();
		return normal;
	}
	Eigen::Vector3d normal = (trans.inverse() * point - o).normalized();
	normal = trans.linear().inverse().transpose() * normal;
	normal.normalize();
	return normal;
}

// Triangle methods
Triangle::Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Transform<double, 3, Eigen::Affine > transformation, Material material)
{
	Eigen::Vector3d edge1, edge2;
	v0 = transformation * vertex0;
	v1 = transformation * vertex1;
	v2 = transformation * vertex2;
	mat = material;
	edge1 = v1 - v0;
	edge2 = v2 - v0;
	n = edge1.cross(edge2);
	n = transformation.linear().inverse().transpose() * n;
	n.normalize();
}

Eigen::Vector3d Triangle::barycentric(Eigen::Vector3d point) {
	Eigen::Vector3d t0 = v1 - v0;
	Eigen::Vector3d t1 = v2 - v0;
	Eigen::Vector3d t2 = point - v0;

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

double Triangle::intersect(Ray ray)
{
	//ray-plane intersection
	double t = ray.pt.dot(n);
	if (abs(t) < eps) {
		return -1;
	}
	t = (v0 - ray.p0).dot(n) / t;
	if (t < eps) {
		return -1;
	}
	// point inside triangle
	Eigen::Vector3d bary = barycentric(Eigen::Vector3d(ray.p0 + t * ray.pt));
	if (bary[0] == -1) {
		return -1;
	}
	return t;
	return -1;
}

Eigen::Vector3d Triangle::normal(Eigen::Vector3d point)
{
	return n;
}

// TriNormal methods
void TriNormal::setNormal(Eigen::Vector3d normal0, Eigen::Vector3d normal1, Eigen::Vector3d normal2)
{
	n0 = normal0.normalized();
	n1 = normal1.normalized();
	n2 = normal2.normalized();
}

Eigen::Vector3d TriNormal::normal(Eigen::Vector3d point)
{
	Eigen::Vector3d bary = barycentric(point);
	return (bary[0] * n0 + bary[1] * n1 + bary[2] * n2).normalized();
}
