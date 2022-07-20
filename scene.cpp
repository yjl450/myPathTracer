#include "scene.h"

// Scene methods
Scene::~Scene()
{
	//TODO:
}

Directional::Directional(Eigen::Vector3d direction, Eigen::Vector3d color)
{
	kind = "directional";
	v0 = direction;
	c = color;
}

PointLight::PointLight(Eigen::Vector3d origin, Eigen::Vector3d color)
{
	kind = "point";
	v0 = origin;
	c - color;
}
