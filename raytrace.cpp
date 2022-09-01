#include "raytrace.h"
#include "bvh.h"

void NormalizeColor(Eigen::Vector3d& color) {
	color[0] = (color[0] < 1) ? color[0] * 255 : 255;
	color[1] = (color[1] < 1) ? color[1] * 255 : 255;
	color[2] = (color[2] < 1) ? color[2] * 255 : 255;
}

RayTracer::RayTracer(Scene s)
{
	scene = std::move(s);
}

Intersection RayTracer::intersect(Ray ray)
{
	// TODO: use BVH
	double dist = std::numeric_limits<double>::infinity();
	double t = -1;
	int ind = -1;

	for (int p = 0; p < scene.primitives.size(); p++)
	{
		t = scene.primitives[p]->intersect(ray);
		if (t > eps && t < dist) {
			dist = t;
			ind = p;
		}
	}
	if (ind == -1) {
		dist = -1;
	}
	return Intersection{ dist, ind };
}

bool RayTracer::visible(Eigen::Vector3d point, int lightInd)
{
	double dist = std::numeric_limits<double>::infinity();
	Eigen::Vector3d color = scene.lights[lightInd]->c;
	Eigen::Vector3d direction = scene.lights[lightInd]->v0;
	if (scene.lights[lightInd]->kind == "point") {
		direction = direction - point;
		direction.normalize();
		dist = (point - scene.lights[lightInd]->v0).norm();
	}
	Ray shadowRay(point + eps * direction, direction);
	Intersection hit = intersect(shadowRay);
	if (hit.ind != -1 && hit.t < dist) {
		return false;
	}
	return true;
}

Eigen::Array3d RayTracer::diffuse(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd)
{
	Eigen::Vector3d LiDir = scene.lights[lightInd]->v0;
		Eigen::Vector3d a;
	if (scene.lights[lightInd]->kind == "point") {
		LiDir = LiDir - point;
		LiDir.normalize();
	}
	Eigen::Vector3d normal = prim->normal(point);
	double intensity = std::max(normal.dot(LiDir), 0.0);
	Eigen::Array3d color = scene.lights[lightInd]->c * prim->mat.diffuse;
	return color * intensity;
}

Eigen::Array3d RayTracer::specular(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int lightInd, Eigen::Vector3d eye)
{
	Eigen::Vector3d LiDir = scene.lights[lightInd]->v0;
	if (scene.lights[lightInd]->kind == "point") {
		LiDir = LiDir - point;
		LiDir.normalize();
	}
	Eigen::Vector3d viewDir = (eye - point).normalized();
	Eigen::Vector3d normal = prim->normal(point);
	Eigen::Vector3d halfAngle = (LiDir + viewDir).normalized();
	double intensity = std::max(normal.dot(halfAngle), 0.0);
	Eigen::Array3d color = scene.lights[lightInd]->c * prim->mat.specualr;
	color *= std::pow(intensity, prim->mat.shininess);
	return color;
}

Ray RayTracer::reflRay(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, Eigen::Vector3d eye) {
	Eigen::Vector3d normal = prim->normal(point);
	Eigen::Vector3d viewDir = (eye - point).normalized();
	Eigen::Vector3d refDir = 2 * normal * viewDir.dot(normal) - viewDir;
	refDir.normalize();
	return Ray(point + eps * refDir, refDir);
}

Eigen::Array3d RayTracer::findColor(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye) {
	Eigen::Array3d shade = prim->mat.ambient + prim->mat.emission;
	for (int i = 0; i < scene.lights.size(); i++) {
		if (visible(point, i)) {
			double attenuation = 1;
			if (scene.lights[i]->kind == "point") {
				double r = (scene.lights[i]->v0 - point).norm();
				attenuation = scene.attenuation[0] + r * scene.attenuation[1] + r * r * scene.attenuation[2];
			}
			shade += (diffuse(point, prim, i) + specular(point, prim, i, eye)) / attenuation;
		}
	}
	if (bounce > 1 && prim->mat.specualr.sum() > eps) {
		Ray reflection = reflRay(point, prim, eye);
		Intersection hit = intersect(reflection);
		if (hit.ind != -1) {
			Eigen::Vector3d newpoint = point + hit.t * reflection.pt;
			shade += prim->mat.specualr * findColor(newpoint, scene.primitives[hit.ind], bounce - 1, point);
		}
	}
	return shade;
}

Ray RayTracer::camRay(int x, int y)
{
	Eigen::Vector3d w, u, v;
	w = (scene.cameraFrom - scene.cameraAt).normalized();
	u = (scene.cameraUp.cross(w)).normalized();
	v = w.cross(u);
	double hfov, alpha, beta;
	hfov = tan(scene.fov * PI / 180 / 2);
	double dx = x + 0.5;
	double dy = y + 0.5;
	alpha = hfov * scene.aspect * (2.0f * dx / scene.width - 1);
	beta = hfov * (1 - 2.0f * dy / scene.height);
	return Ray(scene.cameraFrom, alpha * u + beta * v - w);
}


unsigned char* RayTracer::rayTraceInit()
{
	auto canvas = new unsigned char[scene.height * scene.width * 3];
	int x = 0, y = 0;

	// setup progress bar
	int onePercent = (scene.height * scene.width * 3) / 100;
	progressbar bar(100);
	bar.set_done_char("¨€");

	for (int i = 0; i < scene.height * scene.width * 3; i += 3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;
		if (i % onePercent == 0) {
			bar.update();
		}
		// camera (primary) ray generation
		Ray cameraRay = camRay(x, y);

		// intersection test
		Intersection hit = intersect(cameraRay);
		Eigen::Vector3d shade(0, 0, 0);
		if (hit.ind != -1) {
			Eigen::Vector3d point = cameraRay.p0 + hit.t * cameraRay.pt;
			shade = findColor(point, scene.primitives[hit.ind], scene.maxdepth, scene.cameraFrom);
		}
		NormalizeColor(shade);
		std::copy_n(shade.data(), 3, canvas + i);
	}
	return canvas;
}
