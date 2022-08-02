#include "scene.h"
#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// Scene methods
Scene::~Scene()
{
	//TODO:
}

// Light methods
Directional::Directional(Eigen::Vector3d direction, Eigen::Vector3d color)
{
	kind = "directional";
	v0 = direction;
	c = color;
}

PointLight::PointLight(Eigen::Vector3d origin, Eigen::Vector3d color)
{
	kind = "point";
	v0 = origin;
	c - color;
}

vector<double> read_vals(stringstream& s, int num) {
	double val;
	vector<double> vals;
	for (int i = 0; i < num; i++) {
		s >> val;
		vals.push_back(val);
	}
	return vals;
}

// only works on windows
void reorder_color(Eigen::Vector3d& rgb) {
#if _WIN32
	double tmp = rgb[0];
	rgb[0] = rgb[2];
	rgb[2] = tmp;
#endif
}

void parse_scene(std::ifstream& scenefile, Scene& scene) {
	string parseline;
	string cmd;
	vector<double> vals;
	stringstream s;
	vector<Eigen::Vector3d> vertices;
	vector<Eigen::Vector3d> vertnormal_vertices;
	vector<Eigen::Vector3d> vertnormal_normal;
	Eigen::Vector3d ambientMem(0.2, 0.2, 0.2);
	Material matMem;
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
		else if (cmd == "size") {
			s >> scene.width >> scene.height;
			scene.aspect = (double)scene.width / scene.height;
		}
		else if (cmd == "output") {
			s >> scene.outname;
		}
		else if (cmd == "maxdepth") {
			s >> scene.maxdepth;
		}
		else if (cmd == "camera") {
			vals = read_vals(s, 10);
			scene.cameraFrom << vals[0], vals[1], vals[2];
			scene.cameraAt << vals[3], vals[4], vals[5];
			scene.cameraUp << vals[6], vals[7], vals[8];
			scene.cameraUp.normalize();
			scene.fov = vals[9];
		}
		else if (cmd == "vertex") {
			vals = read_vals(s, 3);
			vertices.push_back(Eigen::Vector3d(vals[0], vals[1], vals[2]));
		}
		else if (cmd == "vertexnormal") {
			vals = read_vals(s, 6);
			vertnormal_vertices.push_back(Eigen::Vector3d(vals[0], vals[1], vals[2]));
			vertnormal_normal.push_back(Eigen::Vector3d(vals[3], vals[4], vals[5]));
		}
		else if (cmd == "sphere") {
			vals = read_vals(s, 4);
			Eigen::Vector3d center;
			center << vals[0], vals[1], vals[2];
			if (trans.isApprox(trans.Identity())) {
				scene.primitives.push_back(make_unique<Sphere>(center, vals[3], ambientMem, matMem, trans, false));
			}
			else {
				scene.primitives.push_back(make_unique<Sphere>(center, vals[3], ambientMem, matMem, trans, true));
			}
		}
		else if (cmd == "tri") {
			vals = read_vals(s, 3);
			Eigen::Vector3d v0, v1, v2;
			v0 = vertices[(int)vals[0]];
			v1 = vertices[(int)vals[1]];
			v2 = vertices[(int)vals[2]];
			scene.primitives.push_back(make_unique<Triangle>(v0, v1, v2, trans, ambientMem, matMem));
		}
		else if (cmd == "trinormal") {
			// TODO: untested
			vals = read_vals(s, 3);
			Eigen::Vector3d v0, v1, v2, n0, n1, n2;
			v0 = vertnormal_vertices[(int)vals[0]];
			v1 = vertnormal_vertices[(int)vals[1]];
			v2 = vertnormal_vertices[(int)vals[2]];
			n0 = trans.linear().inverse().transpose() * vertnormal_normal[(int)vals[0]];
			n1 = trans.linear().inverse().transpose() * vertnormal_normal[(int)vals[1]];
			n2 = trans.linear().inverse().transpose() * vertnormal_normal[(int)vals[2]];
			n0.normalize();
			n1.normalize();
			n2.normalize();
			unique_ptr<TriNormal> temp = make_unique<TriNormal>(v0, v1, v2, trans, ambientMem, matMem);
			temp->setNormal(n0, n1, n2);
			scene.primitives.push_back(move(temp));
		}
		else if (cmd == "directional" || cmd == "point") {
			vals = read_vals(s, 6);
			Eigen::Vector3d p(vals[0], vals[1], vals[2]);
			Eigen::Vector3d c(vals[3], vals[4], vals[5]);
			reorder_color(c);
			if (cmd == "directional") {
				p.normalize();
				scene.lights.push_back(make_unique<Directional>(p, c));
			}
			else {
				scene.lights.push_back(make_unique<PointLight>(p, c));
			}
		}
		else if (cmd == "ambient") {
			vals = read_vals(s, 3);
			ambientMem << vals[0], vals[1], vals[2];
			reorder_color(ambientMem);
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
			reorder_color(matMem.diffuse);
		}
		else if (cmd == "specular") {
			vals = read_vals(s, 3);
			matMem.specualr << vals[0], vals[1], vals[2];
			reorder_color(matMem.specualr);
		}
		else if (cmd == "emission") {
			vals = read_vals(s, 3);
			matMem.emission << vals[0], vals[1], vals[2];
			reorder_color(matMem.emission);
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
			trans = trans * Eigen::Translation<double, 3>(Eigen::Vector3d(vals[0], vals[1], vals[2]));
		}
		else if (cmd == "scale") {
			vals = read_vals(s, 3);
			trans = trans * Eigen::Scaling(Eigen::Vector3d(vals[0], vals[1], vals[2]));
		}
		else if (cmd == "rotate") {
			vals = read_vals(s, 4);
			Eigen::Vector3d axis(vals[0], vals[1], vals[2]);
			axis.normalize();
			trans = trans * Eigen::AngleAxis(vals[3] * M_PI / 180, axis);
		}
	}
}