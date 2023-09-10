# include "../phyvec/phyvec.hpp"
# include "Body.hpp"
# include <string>
# include <iostream>
# include <cmath>
# include <vector>
# include <fstream>

// save and clear are so designed that the first state is always the default state.

double Body::G = 6.67408e-11;

double Body::getG() {
    return G;
}

void Body::setG(double _G) {
    G = _G;
}

Body::Body()
    : mass{0.0}, pos{phyvec(0.0, 0.0, 0.0)}, vel{phyvec(0.0, 0.0, 0.0)}
{
    std::vector<std::vector<double>> vc;
    std::vector <double> state(8);
    state[0] = -1;
    state[1] = mass;
    state[2] = pos[0];
    state[3] = pos[1];
    state[4] = pos[2];
    state[5] = vel[0];
    state[6] = vel[1];
    state[7] = vel[2];
    vc.push_back(state);
    states = vc;
}

Body::Body(double ms, phyvec ps, phyvec vl)
    : mass{ms}, pos{ps}, vel{vl} 

{
    std::vector<std::vector<double>> vc;
    std::vector <double> state(8);
    state[0] = -1;
    state[1] = mass;
    state[2] = pos[0];
    state[3] = pos[1];
    state[4] = pos[2];
    state[5] = vel[0];
    state[6] = vel[1];
    state[7] = vel[2];
    vc.push_back(state);
    states = vc;
}

// Body::Body(double ms = 0.0) {
//     mass = ms;
// }
        
Body::Body (const Body& other)
    : mass{other.mass}, pos{other.pos}, vel{other.vel}
{
    std::vector<std::vector<double>> vc;
    std::vector <double> state(8);
    state[0] = -1;
    state[1] = mass;
    state[2] = pos[0];
    state[3] = pos[1];
    state[4] = pos[2];
    state[5] = vel[0];
    state[6] = vel[1];
    state[7] = vel[2];
    vc.push_back(state);
    states = vc;
}

Body::~Body() {}

phyvec Body::get_pos() const {
    return pos;
}

phyvec Body::get_vel() const {
    return vel;
}

double Body::get_mass() const {
    return mass;
}

Body& Body::set_pos(double x, int i) {
    pos.set(x, i);
    return *this;
}

Body& Body::set_vel(double x, int i) {
    vel.set(x, i);
    return *this;
}

Body& Body::set_pos(phyvec x) {
    pos.set(x[0], 0);
    pos.set(x[1], 1);
    pos.set(x[2], 2);
    return *this;
}

Body& Body::set_vel(phyvec x) {
    vel.set(x[0], 0);
    vel.set(x[1], 1);
    vel.set(x[2], 2);
    return *this;
}

Body& Body::set_body(Body& _body) {
    set_mass(_body.get_mass());
    set_pos(_body.get_pos());
    set_vel(_body.get_vel());
    return *this;
}

Body& Body::set_mass(double x) {
    mass = x;
    return *this;
}

Body& Body::save_state(double time = -1.0) {
    // -1.0 means that the user was just fooling around and didn't want to save the time
    std::vector <double> state(8);
    state[0] = time;
    state[1] = mass;
    state[2] = pos[0];
    state[3] = pos[1];
    state[4] = pos[2];
    state[5] = vel[0];
    state[6] = vel[1];
    state[7] = vel[2];
    states.push_back(state);
    return *this;
}

Body& Body::clear_state() {
    states.clear();
    states.push_back({-1.0, mass, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]});
    return *this;
}

std::vector <std::vector<double>> Body::get_state() const{
    return states;
}

void Body::state_to_csv(std::string filename) const {
    std::ofstream file;
    file.open(filename);
    for (int i = 0; i < states.size(); i++) {
        for (int j = 0; j < 7; j++) {
            file << states[i][j] << ",";
        }
        file << states[i][7];
        file << "\n";
    }
    file.close();
}

std::string Body::to_string() const {
    std::stringstream ss;
    ss << std::scientific << std::setprecision(std::numeric_limits<double>::digits10) << mass;
    std::string m0 = ss.str();
    ss.str("");
    ss.clear();
    return "mass: " + m0 + "\n" +
           "pos: " + pos.to_string() + "\n" +
           "vel: " + vel.to_string() + "\n";
}