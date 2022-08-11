#pragma once
#include <algorithm>
#include "scene.h"
#include "progressbar.hpp" // https://github.com/gipert/progressbar

class Intersection {
public:
	double t = -1;
	int ind = -1;
};

class RayTracer {
public:
	Scene scene;

	RayTracer(Scene s);
	Intersection intersect(Ray ray);
	Ray camRay(int x, int y);
	Ray reflRay(Eigen::Vector3d point, int primInd, Eigen::Vector3d eye);
	bool visible(Eigen::Vector3d point, int lightInd);
	Eigen::Array3d findColor(Eigen::Vector3d point, int primInd, int bounce, Eigen::Vector3d eye);
	Eigen::Array3d diffuse(Eigen::Vector3d point, int primInd, int lightInd);
	Eigen::Array3d specular(Eigen::Vector3d point, int primInd, int lightInd, Eigen::Vector3d eye);
	unsigned char* rayTraceInit();
};
