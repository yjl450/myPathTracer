#pragma once
#include "scene.h"

class Intersection {
public:
	double t = -1;
	int i = -1;
};

class RayTracer {
public:
	Scene scene;

	RayTracer(Scene s);
	Intersection intersect(Ray ray, bool early_stop = false);
	Ray camRay(int x, int y);
	unsigned char* rayTraceInit();
};

//unsigned char* rayTraceInit(Scene scene);