# include <iostream>
# include <cmath>
# include <functional>
# include <string>
#include <stdexcept>
# include "../body/Body.hpp"
# include "../phyvec/phyvec.hpp"
// using namespace std;

# ifndef INTEGRATOR_HPP
# define INTEGRATOR_HPP

using acc_func = std::function<phyvec()>; // acceleration function
std::pair <unsigned long long int, unsigned long long int> *set_rng();

const std::pair<unsigned long long int, unsigned long long int> *fwd_euler(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state);
const std::pair<unsigned long long int, unsigned long long int> *eulric(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state);
const std::pair<unsigned long long int, unsigned long long int> *rk4(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state);

const std::pair<unsigned long long int, unsigned long long int> *integrate(Body& body, acc_func& af, double t0, double tf, double step_size, std::string method, bool save_state);

const std::pair<unsigned long long int, unsigned long long int> *fwd_euler(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state = false) {
    std::pair <unsigned long long int, unsigned long long int> *rng = NULL;
    unsigned long long int rng_size = 0;
    if (save_state) {
        rng = new std::pair <unsigned long long int, unsigned long long int>;
        rng->first = body.get_state().size();
    }
    for (double i = t0; i <= tf; i = i + step_size) {
        phyvec a_n = af();
        body.set_vel(body.get_vel() + a_n * step_size);
        body.set_pos(body.get_pos() + body.get_vel() * step_size);
        if (save_state) {
            body.save_state(i);
            rng_size++;
        }
    }
    if (save_state) {
        rng->second = rng->first + rng_size - 1;
    }
    return rng;
}

const std::pair<unsigned long long int, unsigned long long int> *eulric(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state = false) {
    std::pair <unsigned long long int, unsigned long long int> *rng = NULL;
    unsigned long long int rng_size = 0;
    if (save_state) {
        rng = new std::pair <unsigned long long int, unsigned long long int>;
        rng->first = body.get_state().size();
    }
    for (double i = t0; i <= tf; i = i + step_size) {
        phyvec a_n, a_mid;
        a_n = af();
        body.set_vel(body.get_vel() + 0.5 * a_n * step_size);
        body.set_pos(body.get_pos() + 0.5 * body.get_vel() * step_size);
        a_mid = af();
        body.set_vel(body.get_vel() + 0.5 * a_mid * step_size);
        body.set_pos(body.get_pos() + 0.5 * body.get_vel() * step_size);
        if (save_state) {
            body.save_state(i);
            rng_size++;
        }
    }
    if (save_state) {
    rng->second = rng->first + rng_size - 1;
    }
    return rng;
}

const std::pair<unsigned long long int, unsigned long long int> *rk4(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state) {
    std::pair <unsigned long long int, unsigned long long int> *rng = NULL;
    unsigned long long int rng_size = 0;
    if (save_state) {
        rng = new std::pair <unsigned long long int, unsigned long long int>;
        rng->first = body.get_state().size();
    }
    for (double i = t0; i <= tf; i = i + step_size) {
        phyvec k1v = af() * step_size;
        // if (i < t0 + 5 * step_size) std::cout << af().to_string() << std::endl;
        phyvec k1x = body.get_vel() * step_size;
        body.set_pos(body.get_pos() + 0.5 * k1x);
        body.set_vel(body.get_vel() + 0.5 * k1v);
        phyvec k2v = af() * step_size;
        phyvec k2x = body.get_vel() * step_size;
        body.set_pos(body.get_pos() - 0.5 * k1x + 0.5 * k2x);
        body.set_vel(body.get_vel() -0.5 * k1v + 0.5 * k2v);
        phyvec k3v = af() * step_size;
        phyvec k3x = body.get_vel() * step_size;
        body.set_pos(body.get_pos() - 0.5 * k2x + k3x);
        body.set_vel(body.get_vel() - 0.5 * k2v + k3v);
        phyvec k4v = af() * step_size;
        phyvec k4x = body.get_vel() * step_size;
        body.set_pos(body.get_pos() - k3x + (k1x + 2 * k2x + 2 * k3x + k4x) / 6);
        body.set_vel(body.get_vel() - k3v + (k1v + 2 * k2v + 2 * k3v + k4v) / 6);
        if (save_state) {
            body.save_state(i);
            rng_size++;
        }
    }
    if (save_state) {
    rng->second = rng->first + rng_size - 1;
    }
    return rng;   
}

const std::pair<unsigned long long int, unsigned long long int> *integrate(Body& body, acc_func& af, double t0, double tf, double step_size, std::string method, bool save_state = false) {
    if (method == "fwd_euler") {
        return fwd_euler(body, af, t0, tf, step_size, save_state);
    } 
    else if (method == "eulric") {
        return eulric(body, af, t0, tf, step_size, save_state);
    } 
    else if (method == "rk4") {
        return rk4(body, af, t0, tf, step_size, save_state);
    }
    else {
        throw std::invalid_argument("Invalid method name");
    }
}

# endif // INTEGRATOR_HPP