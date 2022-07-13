
#include "myPathTracer.h"


int main(int argc, char** argv)
{
	std::cout << "Simple Path Tracer v0.1\nBy Yijian Liu" << std::endl;
	if (argc != 2) {
		std::cout << "\nOne argument needed for scene description." << std::endl;
		return 0;
	}
	std::ifstream scenefile(argv[1], std::ios::in);
	assert(scenefile.is_open());
	std::cout << "\n" << argv[1] << " load successfully" << std::endl;


	return 0;
}
