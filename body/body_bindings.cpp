// please write bindings for destructors of twobody and crtb

# include <pybind11/numpy.h>
# include <pybind11/operators.h>
// # include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
# include "../phyvec/phyvec.hpp"
# include "Body.hpp"
# include "twobody/Twobody.hpp"
// # include "crtb/Crtb.hpp"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<double, std::allocator<double>>);
PYBIND11_MAKE_OPAQUE(std::vector<std::vector<double>, std::allocator<std::vector<double>>>);

using vec_doub_lis = std::vector<double, std::allocator<double>>;
using vec_vec_doub_lis = std::vector<std::vector<double>, std::allocator<std::vector<double>>>;

PYBIND11_MODULE(body, m) {
    // py::bind_vector<std::vector<double>>(m, "VectorDoubl");
    // py::bind_vector<std::vector<std::vector<double>>>(m, "MatrixDouble");
    // py::module phyvec_module = py::module::import("phyvec");
    // py::class_<vec_doub_lis>(m, "VectorDoubl", py::buffer_protocol())
    //     .def(py::init<>())
    //     .def("__getitem__", [](const std::vector<double> &v, size_t i) { return v[i]; }, py::is_operator())
    //     .def("__setitem__", [](std::vector<double> &v, size_t i, double value) { v[i] = value; }, py::is_operator())
    //     .def("__len__", [](const std::vector<double> &v) { return v.size(); })
    //     .def("__iter__", [](std::vector<double> &v) {
    //         return py::make_iterator(v.begin(), v.end());
    //     }, py::keep_alive<0, 1>())
    //     .def("push_back", (void (vec_doub_lis::*)(const double&)) &vec_doub_lis::push_back)
    //     .def("pop_back", &std::vector<double>::pop_back)
    //     .def("clear", &std::vector<double>::clear);

    // py::class_<vec_vec_doub_lis>(m, "MatrixDouble", py::buffer_protocol())
    //     .def(py::init<>())
    //     .def("__getitem__", [](const std::vector<std::vector<double>> &m, size_t i) { return m[i]; }, py::is_operator())
    //     .def("__setitem__", [](std::vector<std::vector<double>> &m, size_t i, const std::vector<double> &row) { m[i] = row; }, py::is_operator())
    //     .def("__len__", [](const std::vector<std::vector<double>> &v) { return v.size(); })
    //     .def("__iter__", [](std::vector<std::vector<double>> &v) {
    //         return py::make_iterator(v.begin(), v.end());
    //     }, py::keep_alive<0, 1>())
    //     // .def("push_back", py::overload_cast<const std::vector<double>&>(&std::vector<std::vector<double>>::push_back))
    //     .def("push_back", (void(vec_vec_doub_lis::*)(const std::vector<double> &)) &vec_vec_doub_lis::push_back)
    //     .def("pop_back", &std::vector<std::vector<double>>::pop_back)
    //     .def("clear", &std::vector<std::vector<double>>::clear);

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
    py::class_<Body>(m, "Body")
        .def(py::init<>())
        .def(py::init<double, phyvec, phyvec>())
        .def("__del__", [](Body* self) {
            self->~Body();
        })
        .def(py::init<const Body&>())
        .def_static("getG", &Body::getG)
        .def_static("setG", &Body::setG)
        .def("get_pos", &Body::get_pos)
        .def("get_vel", &Body::get_vel)
        .def("get_mass", &Body::get_mass)
        .def("set_pos", py::overload_cast<double, int>(&Body::set_pos), py::return_value_policy::reference_internal,py::keep_alive<0, 1>())
        .def("set_vel", py::overload_cast<double, int>(&Body::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_pos", py::overload_cast<phyvec>(&Body::set_pos), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_vel", py::overload_cast<phyvec>(&Body::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_mass", &Body::set_mass, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("__repr__", &Body::to_string)
        .def("save_state", &Body::save_state, py::arg("time") = -1.0, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("get_states", &Body::get_state, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("clear_states", &Body::clear_state)
        .def("states_to_csv", &Body::state_to_csv);
    // m.attr("phyvec") = phyvec_module.attr("phyvec");

    auto m_twb = m.def_submodule("twobody");
    py::class_<Twobody, Body>(m_twb, "Twobody")
        .def(py::init<>())
        .def(py::init<Body&, Body&>())
        .def("set_body", &Twobody::set_body, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_mass", &Twobody::set_mass, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_pos", py::overload_cast<double, int, int>(&Twobody::set_pos), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_pos", py::overload_cast<phyvec, int> (&Twobody::set_pos), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_vel", py::overload_cast<double, int, int> (&Twobody::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("set_vel", py::overload_cast<phyvec, int> (&Twobody::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("save_states", &Twobody::save_states, py::arg("time") = -1.0, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("states_to_csv", &Twobody::states_to_csv)
        .def("clear_states", &Twobody::clear_states)
        .def("__getitem__", [](const Twobody& t, int i) {return t[i];}, py::is_operator(), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("evolve_fwdeuler", &Twobody::evolve_fwdeuler, py::arg("time"), py::arg("step_size"), py::arg("save_state") = false, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
        .def("evolve_eulric", &Twobody::evolve_eulric, py::arg("time"), py::arg("step_size"), py::arg("save_state") = false, py::return_value_policy::reference_internal, py::keep_alive<0, 1>());

    // auto m_crtb = m.def_submodule("crtb");
    // py::class_<Crtb, Body> (m_crtb, "Crtb")
    //     .def(py::init<>())
    //     .def(py::init<Body&, Body&, Body&>())
    //     .def_static("get_w", &Crtb::get_w)
    //     .def("set_body", &Crtb::set_body, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("set_mass", &Crtb::set_mass, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("set_pos", py::overload_cast<double, int, int>(&Crtb::set_pos), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("set_pos", py::overload_cast<phyvec, int>(&Crtb::set_pos), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("set_vel", py::overload_cast<double, int, int>(&Crtb::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("set_vel", py::overload_cast<phyvec, int>(&Crtb::set_vel), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("save_states", &Crtb::save_states, py::arg("time") = -1.0, py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("states_to_csv", &Crtb::states_to_csv)
    //     .def("clear_states", &Crtb::clear_states)
    //     .def("__getitem__", [](const Crtb& t, int i) {return t[i];}, py::is_operator(), py::return_value_policy::reference_internal, py::keep_alive<0, 1>())
    //     .def("L4", &Crtb::L4)
    //     .def("L5", &Crtb::L5)
    //     .def("evolve_eulric", &Crtb::evolve_eulric, py::arg("time"), py::arg("step_size"), py::arg("save_state") = false, py::return_value_policy::reference_internal, py::keep_alive<0, 1>());
}