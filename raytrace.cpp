#include "raytrace.h"
//TODO: fix build fail
unsigned char* rayTraceInit(Scene scene)
{
	auto canvas = new unsigned char[scene.height * scene.width * 3];
	int x = 0, y = 0;
	//TODO: add progress bar
	for (int i = 0; i < scene.height * scene.width * 3; i += 3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;

		// camera (primary) ray generation
		Eigen::Vector3d w, u, v;
		w = (scene.cameraFrom - scene.cameraAt).normalized();
		u = (scene.cameraUp.cross(w)).normalized();
		v = w.cross(u);
		double hfov, alpha, beta;
		hfov = tan(scene.fov * PI / 180 / 2);
		alpha = hfov * scene.aspect * (2.0f * x / scene.width - 1);
		beta = hfov * (1 - 2.0f * y / scene.height);
		Ray cameraRay(scene.cameraFrom, alpha * u + beta * v - w);
		// intersection test
		// TODO: use BVH
		double dist = std::numeric_limits<double>::infinity();
		double t = -1;
		int ind = -1;

		for (int p = 0; p < scene.primitives.size(); p++)
		{
			t = scene.primitives[p]->intersect(cameraRay);
			if (t > 0 && t < dist) {
				dist = t;
				ind = p;
			}
		}
		Eigen::Vector3d shade(0, 0, 0);
		if (ind != -1) {
			shade = scene.primitives[ind]->mat.ambient + scene.primitives[ind]->mat.emission;
			shade = shade * 255;
		}
		std::copy_n(shade.data(), 3, canvas + i);
	}
	return canvas;
}
