#pragma once
#include "primitive.h"

class Light {
public:
	std::string kind;
	Eigen::Vector3d v0;
	Eigen::Array3d c;
};

class Directional : public Light {
public:
	Directional(Eigen::Vector3d direction, Eigen::Array3d color);
};

class PointLight : public Light {
public:
	PointLight(Eigen::Vector3d origin, Eigen::Array3d color);
};

class QuadLight : public Light {
public:
	Eigen::Vector3d va;
	Eigen::Vector3d vb;
	Eigen::Vector3d vc;
	Eigen::Vector3d vd;
	Eigen::Vector3d n;
	Eigen::Vector3d e1;
	Eigen::Vector3d e2;
	double area;

	QuadLight(Eigen::Vector3d origin, Eigen::Vector3d edge1, Eigen::Vector3d edge2, Eigen::Array3d color);
	double intersect(Ray ray);
	Eigen::Vector3d barycentric(Eigen::Vector3d point, int partition);
	std::vector<Eigen::Vector3d> samples(int count, bool stratify, std::default_random_engine random);
};