#include <iostream>
#include <pybind11/pybind11.h>
#include <Eigen/core>
#include <pybind11/stl.h>
#include <vector>
#include <list>
#include <string>

using namespace std;

namespace py = pybind11;

struct PybindStruct
{
	PybindStruct() : version(0) {}
	PybindStruct(int v) : version(v) {}
	int version = 0;
	void print() {
		std::cout << "version is: " << version << std::endl;
	}
};

int addPy(int v1, int v2) {
	return v1 + v2;
}

auto testVector(std::vector<Eigen::Vector3d>& vec)
{
	if (!vec.empty())
		vec[0](0) += 1;
	return vec;
}

auto testPtr(std::vector<PybindStruct*>& vec)
{
	if (!vec.empty())
		vec[0]->version = 10;
	return vec;
}


PYBIND11_MODULE(example_Py, m) {
	m.doc() = "python example";
	m.def("addPy", &addPy, "int + int, return int");
	py::class_<PybindStruct>(m, "PyStruct")
		.def(py::init<>())
		.def(py::init<int>())
		.def_readwrite("version", &PybindStruct::version)
		.def("print", &PybindStruct::print)
	;

	py::class_<Eigen::Vector3d>(m, "Vector3d")
		.def(py::init<>())
		.def(py::init<double, double, double>())
		.def("x", [](Eigen::Vector3d& v) {return v.x(); })
		.def("y", [](Eigen::Vector3d& v) {return v.y(); })
		.def("z", [](Eigen::Vector3d& v) {return v.z(); })
		.def("__repr__", [](const Eigen::Vector3d& v) {
		return "<example.Vector3d x=" + std::to_string(v.x()) +
			" y=" + std::to_string(v.y()) +
			" z=" + std::to_string(v.z()) + ">";
			})
		;

	m.def("testVector", &testVector);
	m.def("testPtr", &testPtr);
}





//int main(int argc, char* argv[]){
//    std::cout<<"hello examplePy!"<<std::endl;
//    return 0;
//}

