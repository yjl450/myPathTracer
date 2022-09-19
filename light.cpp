#include "light.h"

// Light methods
Directional::Directional(Eigen::Vector3d direction, Eigen::Array3d color)
{
	kind = "directional";
	v0 = direction;
	c = color;
}

PointLight::PointLight(Eigen::Vector3d origin, Eigen::Array3d color)
{
	kind = "point";
	v0 = origin;
	c = color;
}


QuadLight::QuadLight(Eigen::Vector3d origin, Eigen::Vector3d edge1, Eigen::Vector3d edge2, Eigen::Array3d color)
{
	kind = "quad";
	a = origin;
	ab = edge1;
	ac = edge2;
	c = color;
}