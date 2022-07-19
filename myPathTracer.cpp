
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/core>
#include <memory>
// fpng at https ://github.com/richgel999/fpng
#include "fpng.h"
#include "fpng.cpp"
#include "primitive.h"

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
	vector<Eigen::Vector3d> vertices;
	vector<Eigen::Vector3d> vertnormal_vertices;
	vector<Eigen::Vector3d> vertnormal_normal;

	scene.maxdepth = 5; // default maxdepth if not provided
	while (getline(scenefile, parseline)) {
		if (parseline[0] == '#' || parseline.length() == 0) continue;
		s.clear();
		s.str(parseline);
		s >> cmd;
		if (cmd == "size"){
			s >> scene.width;
			s >> scene.height;
		} 
		else if (cmd == "camera"){
			vals = read_vals(s, 10);
			scene.cameraFrom << vals[0], vals[1], vals[2];
			scene.cameraAt << vals[3], vals[4], vals[5];
			scene.cameraUp << vals[6], vals[7], vals[8];
			scene.fov = vals[9];
		}
		else if (cmd == "vertex") {
			vals = read_vals(s, 3);
			vertices.push_back(Eigen::Vector3d(vals[0], vals[1], vals[2]));
			//cout << vertices.size() << endl;
		}
		else if (cmd == "vertexnormal") {
			vals = read_vals(s, 6);
			vertnormal_vertices.push_back(Eigen::Vector3d(vals[0], vals[1], vals[2]));
			vertnormal_normal.push_back(Eigen::Vector3d(vals[3], vals[4], vals[5]));
		}
		else if (cmd == "sphere"){
			vals = read_vals(s, 4);
			Eigen::Vector3d center;
			center << vals[0], vals[1], vals[2];
			scene.primitives.push_back(make_unique<Sphere>(center, vals[3]));

		}
		else if (cmd == "tri") {
			vals = read_vals(s, 3);
			scene.primitives.push_back(make_unique<Triangle>(vertices[(int)vals[0]], vertices[(int)vals[1]], vertices[(int)vals[2]]));
			//cout << scene.primitives.size() << endl << scene.primitives[0]->normal(vertices[0]) << endl;
		}
		else if (cmd == "trinormal") {
			// TODO: untested
			vals = read_vals(s, 3);
			unique_ptr<TriNormal> temp = make_unique<TriNormal>(vertnormal_vertices[(int)vals[0]], vertnormal_vertices[(int)vals[1]], vertnormal_vertices[(int)vals[2]]);
			temp->setNormal(vertnormal_normal[(int)vals[0]], vertnormal_normal[(int)vals[1]], vertnormal_normal[(int)vals[2]]);
			scene.primitives.push_back(move(temp));
		}
		else {
			cout << "\t" << parseline << endl;
		}
	}
	s.clear();

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
