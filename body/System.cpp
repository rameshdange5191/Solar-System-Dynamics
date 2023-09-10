# include "System.hpp"

double System::G = 6.67408e-11; // SI units
double System::c = 299792458; // SI units

double System::getG() {
    return G;
}

void System::setG(double _G) {
    G = _G;
}

double System::get_c() {
    return c;
}

void System::set_c(double _c) {
    c = _c;
}

System::System() {
    body.resize(0);
}

System::System(int n) {
    body.resize(n, Body());
}

const Body& System::operator[] (int i) const {
    return body[i]; // i should be within range
}

System& System::set_body(Body& _body, int i) {
    body[i].set_body(_body);
    set_body_internal();
    return *this;
}

System& System::set_mass(double mass, int i) {
    body[i].set_mass(mass);
    set_mass_internal();
    return *this;
}

System& System::set_pos(double x, int i, int j) {
    body[j].set_pos(x, i);
    set_pos_internal();
    return *this;
}

System& System::set_pos(phyvec x, int i) {
    body[i].set_pos(x);
    set_pos_internal();
    return *this;
}

System& System::set_vel(double x, int i, int j) {
    body[j].set_vel(x, i);
    set_vel_internal();
    return *this;
}

System& System::set_vel(phyvec x, int i) {
    body[i].set_vel(x);
    set_vel_internal();
    return *this;
}

System& System::save_state(double time = -1.0) {
    for (int i = 0; i < body.size(); i++) {
        body[i].save_state(time);
    }
    return *this;
}

System& System::clear_state() {
    for (int i = 0; i < body.size(); i++) {
        body[i].clear_state();
    }
    clear_state_internal();
    return *this;
}

void System::state_to_csv(std::string filename, int i) const {
    body[i].state_to_csv(filename);
}