#include <math.h>
#include "primitive.h"

// Ray methods
Ray::Ray(Eigen::Vector3d v0, Eigen::Vector3d vt)
{
	p0 = v0;
	pt = vt.normalized();
}

// Sphere methods
Sphere::Sphere(Eigen::Vector3d center, double radius, Eigen::Vector3d amb, Material material, Eigen::Transform<double, 3, Eigen::Affine> transformation, bool trans_flag)
{
	o = center;
	r = radius;
	ambient = amb;
	mat = material;
	if (trans_flag) {
		transformed = true;
		transInverse = transformation.inverse();
	}
}

double Sphere::intersect(Ray ray)
{
	if (transformed) {
		ray = Ray(transInverse * ray.p0, transInverse.linear() * ray.pt);
	}
	double a, b, d;
	a = ray.pt.dot(ray.pt);
	b = ray.pt.dot(ray.p0 - o) * 2;
	d = b * b - 4 * a * ((ray.p0 - o).dot(ray.p0 - o) - r * r);
	if (d < 0) return -1;
	double t1, t2;
	d = sqrt(d);
	t1 = (-b - d) / (2 * a);
	t2 = (-b + d) / (2 * a);
	if (t1 > 0) return t1;
	else if (t2 > 0) return t2;
	else return -1;
}

Eigen::Vector3d Sphere::normal(Eigen::Vector3d point)
{
	return (point - o).normalized();
}

// Triangle methods
Triangle::Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Vector3d amb, Material material)
{
	Eigen::Vector3d edge1, edge2;
	v0 = vertex0;
	v1 = vertex1;
	v2 = vertex2;
	mat = material;
	ambient = amb;
	edge1 = v1 - v0;
	edge2 = v2 - v0;
	n = edge1.cross(edge2);
	n.normalize();
}

Eigen::Vector3d Triangle::barycentric(Eigen::Vector3d point) {
	Eigen::MatrixXd M(3, 2);
	M.col(0) = v1 - v0;
	M.col(1) = v2 - v0;
	Eigen::Vector3d partial = M.inverse() * (point - v0);
	double m, n, k;
	n = partial[0];
	k = partial[1];
	m = 1 - n - k;
	if (m > 0 && n > 0 && k > 0) {
		return Eigen::Vector3d(m, n, k);
	}
	return Eigen::Vector3d(0, 0, 0);
}

double Triangle::intersect(Ray ray)
{
	// point in plane
	double t = ray.pt.dot(n);
	if (abs(t) < e) {
		return -1;
	}
	t = (v0 - ray.p0).dot(n) / t;
	if (t < 0) {
		return -1;
	}
	// point inside triangle
	Eigen::Vector3d bary = barycentric(Eigen::Vector3d(ray.p0 + t * ray.pt));
	if (bary.isZero()) {
		return -1;
	}
	return t;
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
	n << 0, 0, 0;
}

Eigen::Vector3d TriNormal::normal(Eigen::Vector3d point)
{
	Eigen::Vector3d bary = barycentric(point);
	return (bary[0] * n0 + bary[1] * n1 + bary[2] * n2).normalized();
}
