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

	// initialize shading process
	unsigned char* pathTraceInit();
	Eigen::Array3d integratorDispatch(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye);
	// methods for raytracing
	Eigen::Array3d raytracer(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye);
	Eigen::Array3d diffuse(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd);
	Eigen::Array3d specular(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd, Eigen::Vector3d eye);
	// methods for analytic integrator
	Eigen::Array3d analytic(Eigen::Vector3d r, std::shared_ptr<Primitive> prim);
	double theta(Eigen::Vector3d r, Eigen::Vector3d vk, Eigen::Vector3d vk1);
	Eigen::Vector3d gamma(Eigen::Vector3d r, Eigen::Vector3d vk, Eigen::Vector3d vk1);
	Eigen::Vector3d phi(Eigen::Vector3d r, int lightInd);
	// methods for direct monte carlo path tracing
	Eigen::Array3d direct();
	//utils
	bool lightVisible(Ray ray, double primDepth, int lightInd);
};
