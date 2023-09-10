# include <pybind11/pybind11.h>
# include "trial.hpp"
# include "PhyVec_class.hpp"

namespace py = pybind11;

PYBIND11_MODULE(trial, m) {
    m.doc() = "pybind11 example plugin"; // optional module docstring

    m.def("add", &add, "A function that adds two numbers");
}

PYBIND11_MODULE(PhyVec_class, m) {
    py::class_<PhyVec>(m, "PhyVec")
        .def(py::init<double, double, double>())
        .def(py::init<const PhyVec&>())
        .def("__neg__", &PhyVec::operator-)
        .def("__add__", &PhyVec::operator+)
        .def("__sub__", &PhyVec::operator-)
        .def("__mul__", py::overload_cast<double>(&PhyVec::operator*))
        .def("__mul__", py::overload_cast<const PhyVec&>(&PhyVec::operator*))
}