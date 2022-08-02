#include <limits>
// External libraries
#include <FreeImage.h>
// Project components
#include "scene.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

int main(int argc, char** argv)
{
	cout << "Simple Path Tracer v0.1\nBy Yijian Liu" << endl;
	if (argc != 2) {
		cerr << "\nOne argument needed for scene description." << endl;
		return 0;
	}
	ifstream scenefile(argv[1], ios::in);
	if (!scenefile.is_open()) {
		cerr << "\nCannot open scene description file." << endl;
	} else {
		cout << "\nParsing " << argv[1] << endl; 
	}
	//TODO: add subfolder support
	string outprefix("out\\");

	// start parsing scene description
	Scene scene;
	parse_scene(scenefile, scene);
	cout << "\tOutput: " << scene.outname << " (" << scene.width << "x" << scene.height << ")" << endl;
	cout << "\t" << scene.primitives.size() << " Primitives" << endl;
	cout << "\t" << scene.lights.size() << " Lights" << endl;
	cout << "\tMax recursion depth: " << scene.maxdepth << endl;

	// start shading
	auto canvas = new unsigned char[scene.height*scene.width*3];
	int x = 0, y = 0;
	//TODO: add progress bar
	for (int i = 0; i < scene.height * scene.width * 3; i+=3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;

		// camera (primary) ray generation
		Eigen::Vector3d w, u, v;
		w = (scene.cameraFrom - scene.cameraAt).normalized();
		u = (scene.cameraUp.cross(w)).normalized();
		v = w.cross(u);
		double hfov, alpha, beta;
		hfov = tan(scene.fov * M_PI / 180 / 2);
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
			shade = scene.primitives[ind]->ambient;
			shade = shade * 255;
		}
		copy_n(shade.data(), 3, canvas + i);
	}

	// save image and cleanup memory
	FIBITMAP* img = FreeImage_ConvertFromRawBits(canvas, scene.width, scene.height, scene.width * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);
	FreeImage_Initialise();
	bool success = FreeImage_Save(FIF_PNG, img, scene.outname.c_str(), 0);
	if (success) {
		cout << "\nImage generated at " << scene.outname;
	}
	else {
		cout << "\nImage generation failed";
	}
	FreeImage_Unload(img);
	FreeImage_DeInitialise();
	delete[] canvas;
	 cout<< ", exiting renderer..." << endl;
	return 0;
}
