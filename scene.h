#pragma once

#include "primitive.h"
#include "bvh.h"
#include <fstream>
#include <cassert>
#include <sstream>
#include <vector>
#include <stack>
#include <memory>

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

class Scene {
public:
	// canvas size
	int width = 0;
	int height = 0;
	double aspect = 0;
	// max number of ray bounce
	int maxdepth = 5;
	// camera setting
	Eigen::Vector3d cameraFrom;
	Eigen::Vector3d cameraAt;
	Eigen::Vector3d cameraUp;
	double fov = 0;
	// primitives
	std::vector<std::shared_ptr<Primitive>> primitives;
	std::shared_ptr<BVHnode> BVHtree = nullptr;
	// lighting
	std::vector<double> attenuation{ 1, 0, 0 };
	std::vector<std::unique_ptr<Light>> lights;
	//output
	std::string outname = "output.png";

	Scene() = default;
	Scene(std::ifstream& scenefile);
};