#include <iostream>
#include <pybind11/pybind11.h>

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

PYBIND11_MODULE(example_Py, m) {
	m.doc() = "python example";
	m.def("addPy", &addPy, "int + int, return int");
	py::class_<PybindStruct>(m, "PyStruct")
		.def(py::init<>())
		.def(py::init<int>())
		.def_readwrite("version", &PybindStruct::version)
	;
}





//int main(int argc, char* argv[]){
//    std::cout<<"hello examplePy!"<<std::endl;
//    return 0;
//}

