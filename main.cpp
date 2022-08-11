#include <limits>
#include <chrono>
// External libraries
#include <FreeImage.h>
// Project components
#include "scene.h"
#include "raytrace.h"

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

	// parsing scene description
	Scene scene(scenefile);
	cout << "\tOutput: " << scene.outname << " (" << scene.width << "x" << scene.height << ")" << endl;
	cout << "\t" << scene.primitives.size() << " Primitives" << endl;
	cout << "\t" << scene.lights.size() << " Lights" << endl;
	cout << "\tMax recursion depth: " << scene.maxdepth << endl;
	int width = scene.width;
	int height = scene.height;
	string outname = scene.outname;

	// start shading
	auto begin = chrono::steady_clock::now();
	RayTracer raytracer(move(scene));
	auto canvas = raytracer.rayTraceInit();

	//save image and cleanup memory
	FIBITMAP* img = FreeImage_ConvertFromRawBits(canvas, width, height, width * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);
	auto end = chrono::steady_clock::now();
	FreeImage_Initialise();
	if (FreeImage_Save(FIF_PNG, img, outname.c_str(), 0)) {
		cout << "\nImage generated at " << outname << endl;
		cout << "Time spent: " << chrono::duration_cast<chrono::milliseconds>(end - begin).count()/1000.0 << "s" << endl;
	}
	else {
		cout << "\nImage generation failed" << endl;
	}
	FreeImage_Unload(img);
	FreeImage_DeInitialise();
	delete[] canvas;
	cout << "Exiting renderer..." << endl;
	return 0;
}
