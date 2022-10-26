#include "pathtracer.h"

void NormalizeColor(Eigen::Vector3d& color) {
	color[0] = (color[0] < 1) ? color[0] * 255 : 255;
	color[1] = (color[1] < 1) ? color[1] * 255 : 255;
	color[2] = (color[2] < 1) ? color[2] * 255 : 255;
}

PathTracer::PathTracer(Scene s, double randomSeed)
{
	scene = std::move(s);
	seed = randomSeed;
	random = std::default_random_engine(seed);
}

Intersection PathTracer::intersect(Ray ray)
{
	if (scene.BVHtree != nullptr) {
		return scene.BVHtree->intersect(ray);
	}
	double dist = std::numeric_limits<double>::infinity();
	double t = -1;
	std::shared_ptr<Primitive> prim = nullptr;

	for (int p = 0; p < scene.primitives.size(); p++)
	{
		t = scene.primitives[p]->intersect(ray);
		if (t > eps && t < dist) {
			dist = t;
			prim = scene.primitives[p];
		}
	}
	if (prim == nullptr) {
		dist = -1;
	}
	return Intersection{ dist, prim };
}

// Simple ray tracing
Eigen::Array3d PathTracer::raytracer(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye) {
	Eigen::Array3d shade = prim->mat.ambient + prim->mat.emission;
	for (auto i: scene.simpleLights) {
		if (visible(point, i)) {
			double attenuation = 1;
			if (i->kind == "point") {
				double r = (i->v0 - point).norm();
				attenuation = scene.attenuation[0] + r * scene.attenuation[1] + r * r * scene.attenuation[2];
			}
			shade += (diffuse(point, prim, i) + specular(point, prim, i, eye)) / attenuation;
		}
	}
	if (bounce > 1 && prim->mat.specualr.sum() > eps) {
		Ray reflection = reflRay(point, prim, eye);
		Intersection hit = intersect(reflection);
		if (hit.prim != nullptr) {
			Eigen::Vector3d newpoint = point + hit.t * reflection.pt;
			shade += prim->mat.specualr * raytracer(newpoint, hit.prim, bounce - 1, point);
		}
	}
	return shade;
}


bool PathTracer::visible(Eigen::Vector3d point, std::shared_ptr<Light> light)
{
	double dist = std::numeric_limits<double>::infinity();
	Eigen::Vector3d color = light->c;
	Eigen::Vector3d direction = light->v0;
	if (light->kind == "point") {
		direction = direction - point;
		direction.normalize();
		dist = (point - light->v0).norm();
	}
	Ray shadowRay(point + eps * direction, direction);
	Intersection hit = intersect(shadowRay);
	if (hit.prim != nullptr && hit.t < dist) {
		return false;
	}
	return true;
}

Eigen::Array3d PathTracer::diffuse(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, std::shared_ptr<Light> light)
{
	Eigen::Vector3d LiDir = light->v0;
	Eigen::Vector3d a;
	if (light->kind == "point") {
		LiDir = LiDir - point;
		LiDir.normalize();
	}
	Eigen::Vector3d normal = prim->normal(point);
	double intensity = std::max(normal.dot(LiDir), 0.0);
	Eigen::Array3d color = light->c * prim->mat.diffuse;
	return color * intensity;
}

Eigen::Array3d PathTracer::specular(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, std::shared_ptr<Light> light, Eigen::Vector3d eye)
{
	Eigen::Vector3d LiDir = light->v0;
	if (light->kind == "point") {
		LiDir = LiDir - point;
		LiDir.normalize();
	}
	Eigen::Vector3d viewDir = (eye - point).normalized();
	Eigen::Vector3d normal = prim->normal(point);
	Eigen::Vector3d halfAngle = (LiDir + viewDir).normalized();
	double intensity = std::max(normal.dot(halfAngle), 0.0);
	Eigen::Array3d color = light->c * prim->mat.specualr;
	color *= std::pow(intensity, prim->mat.shininess);
	return color;
}

Ray PathTracer::reflRay(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, Eigen::Vector3d eye) {
	Eigen::Vector3d normal = prim->normal(point);
	Eigen::Vector3d viewDir = (eye - point).normalized();
	Eigen::Vector3d refDir = 2 * normal * viewDir.dot(normal) - viewDir;
	refDir.normalize();
	return Ray(point + eps * refDir, refDir);
}

// analytic solution
Eigen::Array3d PathTracer::analytic(Eigen::Vector3d r, std::shared_ptr<Primitive> prim)
{
	Eigen::Vector3d n = prim->normal(r);
	Eigen::Array3d color(0, 0, 0);
	for (auto i: scene.polyLights) {
		if (prim->mat.emission.sum() - 0 <  eps) {
			color += prim->mat.diffuse * i->c * (phi(r, i).dot(n)) / PI;
		}
		else {
			color += prim->mat.emission;
		}
	}
	return color;
}

double PathTracer::theta(Eigen::Vector3d r, Eigen::Vector3d vk, Eigen::Vector3d vk1) {
	return acos((vk - r).normalized().dot((vk1 - r).normalized()));
}

Eigen::Vector3d PathTracer::gamma(Eigen::Vector3d r, Eigen::Vector3d vk, Eigen::Vector3d vk1) {
	return (vk - r).cross(vk1 - r).normalized();
}

Eigen::Vector3d PathTracer::phi(Eigen::Vector3d r, std::shared_ptr<QuadLight> light) {
	Eigen::Vector3d a = light->va;
	Eigen::Vector3d b = light->vb;
	Eigen::Vector3d c = light->vc;
	Eigen::Vector3d d = light->vd;
	Eigen::Vector3d irradiance(0, 0, 0);
	irradiance += theta(r, a, b) * gamma(r, a, b);
	irradiance += theta(r, b, d) * gamma(r, b, d);
	irradiance += theta(r, d, c) * gamma(r, d, c);
	irradiance += theta(r, c, a) * gamma(r, c, a);
	return irradiance * 0.5;
}

// Monte Carlo direct illumination
Eigen::Array3d PathTracer::direct(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, Eigen::Vector3d eye)
{
	// TODO: rendering incorrect
	Eigen::Vector3d n = prim->normal(point);
	Eigen::Array3d color(0, 0, 0), color_i(0, 0, 0);
	Eigen::Array3d constant(1,1,1);
	for (std::shared_ptr<QuadLight> li : scene.polyLights) {
		color_i.setZero();
		std::vector<Eigen::Vector3d> lightSamples = li->samples(scene.sample, scene.stratify, random);
		for (auto p : lightSamples) {
			if (visibility(point, p)) {
				color_i += phoneBRDF(prim, eye, point, p) * geometry(prim, li, point, p);
			}
		}
		color += color_i * li->c * li->area / lightSamples.size();
	}
	return color;
}

bool PathTracer::visibility(Eigen::Vector3d x1, Eigen::Vector3d x2) {
	//x1: point of primitive
	//x2: point of light
	Eigen::Vector3d direction = (x2 - x1).normalized();
	Ray r(x1 + eps * direction, direction);
	Intersection hit = intersect(r);
	if (hit.t > eps && hit.t < (x1 - x2).norm()) return false;
	return true;
}

double PathTracer::geometry(std::shared_ptr<Primitive> prim, std::shared_ptr<QuadLight> light, Eigen::Vector3d x1, Eigen::Vector3d x2) {
	double R, nldir;
	Eigen::Vector3d n, nl, dir;
	n = prim->normal(x1);
	nl = light->n;
	dir = (x2 - x1);
	R = dir.norm();
	dir.normalize();
	nldir = nl.dot(dir);
	if (nldir < 0) nldir *= -1;
	return (n.dot(dir)) * nldir / (R * R);
}

Eigen::Array3d PathTracer::phoneBRDF(std::shared_ptr<Primitive> prim, Eigen::Vector3d eye, Eigen::Vector3d x1, Eigen::Vector3d x2) {
	Eigen::Array3d diffuse, specular;
	Eigen::Vector3d r, lm, n;
	double intensity;
	diffuse = prim->mat.diffuse / PI;
	specular = prim->mat.specualr * (prim->mat.shininess + 2) / (2 * PI);
	lm = (x2 - x1).normalized();
	n = prim->normal(x1);
	r = 2 * (lm.dot(n)) * n - lm;
	intensity = pow(r.dot((eye - x1).normalized()), prim->mat.shininess);
	return diffuse + specular * intensity;
}

bool PathTracer::lightVisible(Ray ray, std::shared_ptr<QuadLight> light, double hit) {
	double lightDepth = -1.0;
	double t = -1.0;
	int lightVisible = -1;
	if (hit == -2) {
		Intersection primHit = intersect(ray);
		hit = primHit.t;
	}
	t = light->intersect(ray);
	if (hit == -1) return (t != -1);
	else return (hit > t);
}

Ray PathTracer::camRay(int x, int y)
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

Eigen::Array3d PathTracer::integratorDispatch(Eigen::Vector3d point, std::shared_ptr<Primitive> prim, int bounce, Eigen::Vector3d eye) {
	if (scene.integrator == "raytracer") {
		return raytracer(point, prim, bounce, eye);
	}
	else if (scene.integrator == "analyticdirect") {
		return analytic(point, prim);
	}
	else if (scene.integrator == "direct") {
		return direct(point, prim, eye);
	}
	return Eigen::Array3d(0, 0, 0);
}

unsigned char* PathTracer::pathTraceInit()
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

		double lightDepth = -1.0;
		double lt = -1.0;
		std::shared_ptr<QuadLight> light = nullptr;
		for (auto l: scene.polyLights) {
			lt = l->intersect(cameraRay);
			if (lt > 0 && (lt < lightDepth || lightDepth < 0)) {
				lightDepth = lt;
				light = l;
			}
		}
		bool lightVisiility = false;
		if (light != nullptr) {
			lightVisiility = lightVisible(cameraRay, light, hit.t);
		}

		if (lightVisiility) {
			shade = light->c;
		} 
		else if (hit.prim != nullptr) {
			Eigen::Vector3d point = cameraRay.p0 + hit.t * cameraRay.pt;
			shade = integratorDispatch(point, hit.prim, scene.maxdepth, scene.cameraFrom);
		}
		NormalizeColor(shade);
		std::copy_n(shade.data(), 3, canvas + i);
	}
	return canvas;
}
