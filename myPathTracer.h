#pragma once

#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/core>
// fpng at https ://github.com/richgel999/fpng
#include "fpng.h"
#include "fpng.cpp"

//typedef struct {
//	Eigen::Vector3d
//} Camera;

typedef struct {
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
}Scene;