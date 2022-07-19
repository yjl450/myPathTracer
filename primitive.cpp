#include "primitive.h"

// Scene methods
Scene::~Scene()
{
	//TODO:
}


// Ray methods
Ray::Ray(Eigen::Vector3d v0, Eigen::Vector3d vt)
{
	p0 = v0;
	pt = vt;
}

// Sphere methods
Sphere::Sphere(Eigen::Vector3d center, double radius)
{
	o = center;
	r = radius;
}

double Sphere::intersect(Ray ray)
{
	//TODO:
	return 0.0;
}

Eigen::Vector3d Sphere::normal(Eigen::Vector3d point)
{
	return point - o;
}

// Triangle methods
Triangle::Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2)
{
	Eigen::Vector3d edge1, edge2;
	v0 = vertex0;
	v1 = vertex1;
	v2 = vertex2;
	edge1 = v1 - v0;
	edge2 = v2 - v0;
	n = edge1.cross(edge2);
	n.normalize();
}

double Triangle::intersect(Ray ray)
{
	// TODO:
	return 0.0;
}

Eigen::Vector3d Triangle::normal(Eigen::Vector3d point)
{
	return n;
}

// TriNormal methods
void TriNormal::setNormal(Eigen::Vector3d normal0, Eigen::Vector3d normal1, Eigen::Vector3d normal2)
{
	n0 = normal0;
	n1 = normal1;
	n2 = normal2;
	n << 0, 0, 0;
}

Eigen::Vector3d TriNormal::normal(Eigen::Vector3d point)
{
	//TODO: barycentric coor
	return Eigen::Vector3d();
}
