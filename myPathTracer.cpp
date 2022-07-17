
#include "myPathTracer.h"

using namespace std;

vector<double> read_vals(stringstream &s, int num) {
	string val;
	vector<double> vals;
	for (int i = 0; i < num; i++) {
		s >> val;
		vals.push_back(stod(val));
	}
	return vals;
}

int main(int argc, char** argv)
{
	cout << "Simple Path Tracer v0.1\nBy Yijian Liu" << endl;
	if (argc != 2) {
		cerr << "\nOne argument needed for scene description." << endl;
		return 0;
	}
	ifstream scenefile(argv[1], ios::in);
	if (!scenefile.is_open()) {
		cerr << "Cannot open scene description file." << endl;
	} else {
		cout << "\nParsing " << argv[1] << endl; 
	}

	// start parsing scene description
	Scene scene{0};
	string parseline;
	string cmd;
	vector<double> vals;
	stringstream s;

	scene.maxdepth = 5; // default maxdepth if not provided
	while (getline(scenefile, parseline)) {
		if (parseline[0] == '#' || parseline.length() == 0) {
			continue;
		}
		s.clear();
		s.str(parseline);
		s >> cmd;
		if (cmd == "size"){
			s >> scene.width;
			s >> scene.height;
		} else if (cmd == "camera") {
			vals = read_vals(s, 10);
			scene.cameraFrom << vals[0], vals[1], vals[2];
			scene.cameraAt << vals[3], vals[4], vals[5];
			scene.cameraUp << vals[6], vals[7], vals[8];
			scene.fov = vals[9];
		}
		cout << "\t" << parseline << endl;
	}

	// start shading
	unsigned char* canvas = new unsigned char[scene.height*scene.width*3];
	int x = 0, y = 0;
	for (int i = 0; i < scene.height * scene.width * 3; i+=3) {
		y = i / 3 / scene.width;
		x = (i / 3) - (i / 3 / scene.width) * scene.width;
		canvas[i] = x * 127 / scene.width + y * 127 / scene.height;
		canvas[i+1] = x * 127 / scene.width + y * 127 / scene.height;
		canvas[i+2] = x * 127 / scene.width + y * 127 / scene.height;
	}
	fpng::fpng_init();
	fpng::fpng_encode_image_to_file("output.png", canvas, scene.width, scene.height, 3);
	delete[] canvas;

	return 0;
}
