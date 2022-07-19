#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class Ray {
public:
	Eigen::Vector3d p0;
	Eigen::Vector3d pt;
	Ray(Eigen::Vector3d p0, Eigen::Vector3d pt);
};

// abstract class for all primitives
class Primitive {
public:
	virtual double intersect(Ray ray) = 0;
	virtual Eigen::Vector3d normal(Eigen::Vector3d point) = 0;
};

class Sphere: public Primitive {
public:
	Eigen::Vector3d o;
	double r;

	Sphere(Eigen::Vector3d center, double radius);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class Triangle: public Primitive {
public:
	Eigen::Vector3d v0;
	Eigen::Vector3d v1;
	Eigen::Vector3d v2;
	Eigen::Vector3d n;

	Triangle(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2);
	virtual double intersect(Ray ray);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class TriNormal : public Triangle {
public:
	Eigen::Vector3d n0;
	Eigen::Vector3d n1;
	Eigen::Vector3d n2;

	TriNormal(Eigen::Vector3d vertex0, Eigen::Vector3d vertex1, Eigen::Vector3d vertex2) :Triangle(vertex0, vertex1, vertex2) {
	};
	void setNormal(Eigen::Vector3d normal0, Eigen::Vector3d normal1, Eigen::Vector3d normal2);
	virtual Eigen::Vector3d normal(Eigen::Vector3d point);
};

class Scene {
public:
	// canvas size
	int width;
	int height;
	// max number of ray bounce
	int maxdepth;
	// camera setting
	Eigen::Vector3d cameraFrom;
	Eigen::Vector3d cameraAt;
	Eigen::Vector3d cameraUp;
	double fov;
	// primitives
	std::vector<std::unique_ptr<Primitive>> primitives;


	~Scene();

};