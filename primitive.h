#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

const double e = 1e-10;

class Material{
public:
	Eigen::Vector3d diffuse = Eigen::Vector3d(1, 0, 0);
	Eigen::Vector3d specualr = Eigen::Vector3d(0, 0, 0);
	double shininess = 0;
	Eigen::Vector3d emission = Eigen::Vector3d(0, 0, 0);
};

class Ray {
public:
	Eigen::Vector3d p0;
	Eigen::Vector3d pt;
	Ray(Eigen::Vector3d p0, Eigen::Vector3d pt);
};

// abstract class for all primitives
class Primitive {
public:
	Eigen::Vector3d ambient;
	Material mat;
	virtual double intersect(Ray ray) = 0;
	virtual Eigen::Vector3d normal(Eigen::Vector3d point) = 0;
};

class Sphere: public Primitive {
public:
	Eigen::Vector3d o;
	double r;
	bool transformed = false;
	Eigen::Transform<double, 3, Eigen::Affine> transInverse = Eigen::Affine3d::Identity();

	Sphere(Eigen::Vector3d center, double radius, Eigen::Vector3d amb, Material material, Eigen::Transform<double, 3, Eigen::Affine> transfromation, bool trans_flag);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class Triangle: public Primitive {
public:
	Eigen::Vector3d v0;
	Eigen::Vector3d v1;
	Eigen::Vector3d v2;
	Eigen::Vector3d n;

	Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Vector3d amb, Material material);
	Eigen::Vector3d barycentric(Eigen::Vector3d point);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class TriNormal : public Triangle {
public:
	Eigen::Vector3d n0;
	Eigen::Vector3d n1;
	Eigen::Vector3d n2;

	TriNormal(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Vector3d amb, Material material) :Triangle(vertex0, vertex1, vertex2, amb, material) {
	};
	void setNormal(Eigen::Vector3d normal0, Eigen::Vector3d normal1, Eigen::Vector3d normal2);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};
