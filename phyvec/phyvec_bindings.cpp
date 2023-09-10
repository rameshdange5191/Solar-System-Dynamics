# include <pybind11/pybind11.h>
# include <pybind11/operators.h>

# include "phyvec.hpp"

namespace py = pybind11;

PYBIND11_MODULE(phyvec, m) {
    py::class_<phyvec>(m, "phyvec")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def(py::init<const phyvec&>())
        .def("__del__", [](phyvec* self) {
            self->~phyvec();
        })
        .def(-py::self)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * py::self)
        .def(py::self * double())
        .def(py::self / double())
        .def("__getitem__", [](const phyvec& v, int i) {return v[i];}, py::is_operator())
        .def(py::self==py::self)
        .def(py::self!=py::self)
        .def("__abs__", &phyvec::abs)
        .def("set", &phyvec::set)
        .def("__repr__", &phyvec::to_string);
    m.def("__mul__", [](double k, const phyvec& v) {return v * k;}, py::is_operator());
    // py::class_<Copyable>(m, "Copyable")
    //     .def("__copy__",  [](const Copyable &self) {
    //         return Copyable(self);
    //     })
    //     .def("__deepcopy__", [](const Copyable &self, py::dict) {
    //         return Copyable(self);
    //     }, "memo"_a);
}