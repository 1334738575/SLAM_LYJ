#include <iostream>
#include <pybind11/pybind11.h>
#include <QT_LYJ.h>


using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(QT_LYJ_Py, m) {
	m.doc() = "python example of QT_LYJ";
	m.def("testQT", &QT_LYJ::testQT, "QT_LYJ's test");
}
