#include <iostream>
// External libraries
#include <FreeImage.h>
// Project components
#include "scene.h"

#if _WIN32
#define r 2
#define g 1
#define b 0
#else
#define r 0
#define g 1
#define b 2
#endif

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
	//string outprefix("out\\");

	// start parsing scene description
	Scene scene;
	parse_scene(scenefile, scene);
	cout << "\tOutput: " << scene.outname << " (" << scene.width << "x" << scene.height << ")" << endl;
	cout << "\t" << scene.primitives.size() << " Primitives" << endl;
	cout << "\t" << scene.lights.size() << " Lights" << endl;
	cout << "\tMax recursion depth: " << scene.maxdepth << endl;

	// start shading
	unsigned char* canvas = new unsigned char[scene.height*scene.width*3];
	int x = 0, y = 0;
	//TODO: add progress bar
	for (int i = 0; i < scene.height * scene.width * 3; i+=3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;
		// TODO: ray tracing
		canvas[i + r] = 0;
		canvas[i + g] = 0;//x * 127 / scene.width + y * 127 / scene.height;
		canvas[i + +b] = x * 127 / scene.width + y * 127 / scene.height;
	}

	// save image and cleanup memory
	FIBITMAP* img = FreeImage_ConvertFromRawBits(canvas, scene.width, scene.height, scene.width * 3, 24, 0xFF0000, 0x00FF00, 0x0000FF, true);
	FreeImage_Initialise();
	FreeImage_Save(FIF_PNG, img, scene.outname.c_str(), 0);
	FreeImage_Unload(img);
	FreeImage_DeInitialise();
	delete[] canvas;
	cout << "\nImage generated at " << scene.outname << ", exiting renderer..." << endl;
	return 0;
}
