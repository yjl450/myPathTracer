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
	Eigen::Vector3d a;
	Eigen::Vector3d ab;
	Eigen::Vector3d ac;
	QuadLight(Eigen::Vector3d origin, Eigen::Vector3d edge1, Eigen::Vector3d edge2, Eigen::Array3d color);
};