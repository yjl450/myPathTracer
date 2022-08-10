#include "scene.h"

using namespace std;

// Light methods
Directional::Directional(Eigen::Vector3d direction, Eigen::Array3d color)
{
	kind = "directional";
	v0 = direction;
	c = color;
}

PointLight::PointLight(Eigen::Vector3d origin, Eigen::Array3d color)
{
	kind = "point";
	v0 = origin;
	c = color;
}

// Utils
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
inline void reorder_color(Eigen::Array3d& rgb) {
#if _WIN32 || __linux__
	double tmp = rgb(0);
	rgb(0) = rgb(2);
	rgb(2) = tmp;
#endif
}

// Scene methods
Scene::Scene(std::ifstream& scenefile) {
	string parseline;
	string cmd;
	vector<double> vals;
	stringstream s;
	vector<Eigen::Vector3d> vertices;
	vector<Eigen::Vector3d> vertnormal_vertices;
	vector<Eigen::Vector3d> vertnormal_normal;
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
			s >> width >> height;
			aspect = (double)width / height;
		}
		else if (cmd == "output") {
			s >> outname;
		}
		else if (cmd == "maxdepth") {
			s >> maxdepth;
		}
		else if (cmd == "camera") {
			vals = read_vals(s, 10);
			cameraFrom << vals[0], vals[1], vals[2];
			cameraAt << vals[3], vals[4], vals[5];
			cameraUp << vals[6], vals[7], vals[8];
			cameraUp.normalize();
			//Eigen::Vector3d camDir = cameraAt - cameraFrom;
			//cameraUp = cameraUp.cross(camDir);
			//cameraUp = camDir.cross(cameraUp);
			//cameraUp.normalize();
			fov = vals[9];
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
				primitives.push_back(make_unique<Sphere>(center, vals[3], matMem, trans, false));
			}
			else {
				primitives.push_back(make_unique<Sphere>(center, vals[3], matMem, trans, true));
			}
		}
		else if (cmd == "tri") {
			vals = read_vals(s, 3);
			Eigen::Vector3d v0, v1, v2;
			v0 = vertices[(int)vals[0]];
			v1 = vertices[(int)vals[1]];
			v2 = vertices[(int)vals[2]];
			primitives.push_back(make_unique<Triangle>(v0, v1, v2, trans, matMem));
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
			unique_ptr<TriNormal> temp = make_unique<TriNormal>(v0, v1, v2, trans, matMem);
			temp->setNormal(n0, n1, n2);
			primitives.push_back(move(temp));
		}
		else if (cmd == "directional" || cmd == "point") {
			vals = read_vals(s, 6);
			Eigen::Vector3d p(vals[0], vals[1], vals[2]);
			Eigen::Array3d c(vals[3], vals[4], vals[5]);
			reorder_color(c);
			if (cmd == "directional") {
				p.normalize();
				lights.push_back(make_unique<Directional>(p, c));
			}
			else {
				lights.push_back(make_unique<PointLight>(p, c));
			}
		}
		else if (cmd == "ambient") {
			vals = read_vals(s, 3);
			matMem.ambient << vals[0], vals[1], vals[2];
			reorder_color(matMem.ambient);
		}
		else if (cmd == "attenuation") {
			vals = read_vals(s, 3);
			attenuation[0] = vals[0];
			attenuation[1] = vals[1];
			attenuation[2] = vals[2];
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
			trans = trans * Eigen::AngleAxis(vals[3] * PI / 180, axis);
		}
	}
}
