#include <iostream>
#include <pybind11/pybind11.h>
#include <SLAM_LYJ.h>

using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(SLAM_LYJ_Py, m) {
	m.doc() = "python example of SLAM_LYJ";
	m.def("SLAM_LYJ_VERSION", &SLAM_LYJ::getVersion, "SLAM_LYJ's version");
	m.def("SLAM_LYJ_VULKAN", &SLAM_LYJ::testVulkan, "SLAM_LYJ's vulkan");
}
