
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
#include <sstream>
#include <vector>
#include <Eigen/core>
#include <memory>
#include <stack>
// External libraries
#include "fpng.h" // fpng at https ://github.com/richgel999/fpng
#include "fpng.cpp"
// Project components
#include "scene.h"

using namespace std;

vector<double> read_vals(stringstream &s, int num) {
	double val;
	vector<double> vals;
	for (int i = 0; i < num; i++) {
		s >> val;
		vals.push_back(val);
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
	Scene scene;
	string parseline;
	string cmd;
	vector<double> vals;
	stringstream s;
	vector<Eigen::Vector3d> vertices;
	vector<Eigen::Vector3d> vertnormal_vertices;
	vector<Eigen::Vector3d> vertnormal_normal;
	Eigen::Vector3d ambientMem(0.2, 0.2, 0.2);
	Material matMem;
	string outname("output.png");
	//TODO: add subfolder support
	//string outprefix("out\\");
	stack<Eigen::Transform<double, 3, Eigen::Affine>> transStack;
	Eigen::Transform<double, 3, Eigen::Affine> trans = Eigen::Affine3d::Identity();

	while (getline(scenefile, parseline)) {
		s.clear();
		s.str(parseline);
		s >> cmd;
		if (parseline.length() == 0 || parseline[0] == '#' || cmd == "maxverts" || cmd == "maxvertnorms") {
			// ignore
			continue;
		}
		else if (cmd == "size"){
			s >> scene.width >> scene.height;
		}
		else if (cmd == "output") {
			s >> outname;
		}
		else if (cmd == "maxdepth") {
			s >> scene.maxdepth;
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
			scene.primitives.push_back(make_unique<Sphere>(center, vals[3], ambientMem, matMem));

		}
		else if (cmd == "tri") {
			vals = read_vals(s, 3);
			scene.primitives.push_back(make_unique<Triangle>(vertices[(int)vals[0]], vertices[(int)vals[1]], vertices[(int)vals[2]], ambientMem, matMem));
			//cout << scene.primitives.size() << endl << scene.primitives[0]->normal(vertices[0]) << endl;
		}
		else if (cmd == "trinormal") {
			// TODO: untested
			vals = read_vals(s, 3);
			unique_ptr<TriNormal> temp = make_unique<TriNormal>(vertnormal_vertices[(int)vals[0]], vertnormal_vertices[(int)vals[1]], vertnormal_vertices[(int)vals[2]], ambientMem, matMem);
			temp->setNormal(vertnormal_normal[(int)vals[0]], vertnormal_normal[(int)vals[1]], vertnormal_normal[(int)vals[2]]);
			scene.primitives.push_back(move(temp));
		}
		else if (cmd == "directional" || cmd == "point") {
			vals = read_vals(s, 6);
			Eigen::Vector3d p(vals[0], vals[1], vals[2]);
			Eigen::Vector3d c(vals[3], vals[4], vals[5]);
			if (cmd == "directional") {
				scene.lights.push_back(make_unique<Directional>(p, c));
			}
			else {
				scene.lights.push_back(make_unique<PointLight>(p, c));
			}
			cout << scene.lights.size() << endl;
		}
		else if (cmd == "ambient") {
			vals = read_vals(s, 3);
			ambientMem << vals[0], vals[1], vals[2];
		}
		else if (cmd == "attenuation") {
			vals = read_vals(s, 3);
			scene.attenuation[0] = vals[0];
			scene.attenuation[1] = vals[1];
			scene.attenuation[2] = vals[2];
		}
		else if (cmd == "diffuse") {
			vals = read_vals(s, 3);
			matMem.diffuse << vals[0], vals[1], vals[2];
		}
		else if (cmd == "specular") {
			vals = read_vals(s, 3);
			matMem.specualr << vals[0], vals[1], vals[2];
		}
		else if (cmd == "emission") {
			vals = read_vals(s, 3);
			matMem.emission << vals[0], vals[1], vals[2];
		}
		else if (cmd == "shininess") {
			vals = read_vals(s, 1);
			matMem.shininess = vals[0];
		}
		else if (cmd == "pushTransform") {
			transStack.push(trans);
		}
		else if (cmd == "popTransform") {
			trans = transStack.top();
			transStack.pop();
		}
		else if (cmd == "translate") {
			vals = read_vals(s, 3);
			trans = Eigen::Translation<double, 3>(Eigen::Vector3d(vals[0], vals[1], vals[2])) * trans;
		}
		else if (cmd == "scale") {
			vals = read_vals(s, 3);
			trans = Eigen::Scaling(Eigen::Vector3d(vals[0], vals[1], vals[2])) * trans;
		}
		else if (cmd == "rotate") {
			vals = read_vals(s, 4);
			trans = Eigen::AngleAxis(vals[3], Eigen::Vector3d(vals[0], vals[1], vals[2])) * trans;
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
	fpng::fpng_encode_image_to_file((outname).c_str(), canvas, scene.width, scene.height, 3);
	cout << "Image generated at " << outname << ", exiting renderer..." << endl;
	delete[] canvas;
	return 0;
}
