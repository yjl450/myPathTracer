#pragma once
#include "scene.h"

#include <algorithm>
class Intersection {
public:
	double t = -1;
	int ind = -1;
};

class RayTracer {
public:
	Scene scene;

	RayTracer(Scene s);
	Intersection intersect(Ray ray, bool early_stop = false);
	Ray camRay(int x, int y);
	bool visible(Eigen::Vector3d point, int lightInd);
	Eigen::Vector3d findColor(Eigen::Vector3d point, int primInd, int bounce, Eigen::Vector3d eye);
	Eigen::Vector3d diffuse(Eigen::Vector3d point, int primInd, int lightInd);
	Eigen::Vector3d specular(Eigen::Vector3d point, int primInd, int lightInd, Eigen::Vector3d eye, int bounce);
	unsigned char* rayTraceInit();
};
