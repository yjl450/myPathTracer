#include <iostream>
// External libraries
#include "fpng.h" // fpng at https ://github.com/richgel999/fpng
#include "fpng.cpp"
// Project components
#include "scene.h"

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
	for (int i = 0; i < scene.height * scene.width * 3; i+=3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;
		//TODO: ray tracing
	}
	fpng::fpng_init();
	fpng::fpng_encode_image_to_file((scene.outname).c_str(), canvas, scene.width, scene.height, 3);
	cout << "\nImage generated at " << scene.outname << ", exiting renderer..." << endl;
	delete[] canvas;
	return 0;
}
