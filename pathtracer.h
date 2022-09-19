#pragma once
#include <algorithm>
#include "scene.h"
#include "progressbar.hpp" // https://github.com/gipert/progressbar

class PathTracer {
public:
	Scene scene;

	PathTracer(Scene s);
	Intersection intersect(Ray ray);
	Ray camRay(int x, int y);
	Ray reflRay(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, Eigen::Vector3d eye);
	bool visible(Eigen::Vector3d point, int lightInd);
	Eigen::Array3d findColor(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye);
	Eigen::Array3d diffuse(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd);
	Eigen::Array3d specular(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd, Eigen::Vector3d eye);
	unsigned char* pathTraceInit();
};
