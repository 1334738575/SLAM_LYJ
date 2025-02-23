#include <iostream>
#include <pybind11/pybind11.h>
#include <SLAM_LYJ.h>


using namespace std;

namespace py = pybind11;

struct PybindStruct
{
	PybindStruct() : version(0) {}
	PybindStruct(int v) : version(v) {}
	int version = 0;
};

int addPy(int v1, int v2) {
	return v1 + v2;
}

PYBIND11_MODULE(examplePy, m) {
	m.doc() = "python example of SLAM_LYJ";
	m.def("addPy", &addPy, "int + int, return int");
	py::class_<PybindStruct>(m, "PyStruct")
		.def(py::init<>())
		.def(py::init<int>())
		.def_readwrite("version", &PybindStruct::version)
	;

	m.def("SLAM_LYJ_VERSION", &SLAM_LYJ::getVersion, "SLAM_LYJ's version");
}





//int main(int argc, char* argv[]){
//    std::cout<<"hello examplePy!"<<std::endl;
//    return 0;
//}

