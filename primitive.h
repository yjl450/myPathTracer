#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>
#define PI M_PI
#include <algorithm>

#define eps 1e-10

class Material{
public:
	Eigen::Array3d ambient = Eigen::Array3d(0.2, 0.2, 0.2);
	Eigen::Array3d diffuse = Eigen::Array3d(1, 0, 0);
	Eigen::Array3d specualr = Eigen::Array3d(0, 0, 0);
	double shininess = 0;
	Eigen::Array3d emission = Eigen::Array3d(0, 0, 0);
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
	Material mat;
	virtual double intersect(Ray ray) = 0;
	virtual Eigen::Vector3d normal(Eigen::Vector3d point) = 0;
};

class Sphere: public Primitive {
public:
	Eigen::Vector3d o;
	double r;
	bool transformed = false;
	Eigen::Transform<double, 3, Eigen::Affine> trans = Eigen::Affine3d::Identity();

	Sphere(Eigen::Vector3d center, double radius, Material material, Eigen::Transform<double, 3, Eigen::Affine> transformation, bool trans_flag);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class Triangle: public Primitive {
public:
	Eigen::Vector3d v0;
	Eigen::Vector3d v1;
	Eigen::Vector3d v2;
	Eigen::Vector3d n;

	Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Transform<double, 3, Eigen::Affine > transformation, Material material);
	Eigen::Vector3d barycentric(Eigen::Vector3d point);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class TriNormal : public Triangle {
public:
	Eigen::Vector3d n0;
	Eigen::Vector3d n1;
	Eigen::Vector3d n2;

	TriNormal(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2, Eigen::Transform<double, 3, Eigen::Affine > transformation, Material material) :Triangle(vertex0, vertex1, vertex2, transformation, material) {
	};
	void setNormal(Eigen::Vector3d normal0, Eigen::Vector3d normal1, Eigen::Vector3d normal2);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};
