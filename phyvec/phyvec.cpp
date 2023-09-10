# include "phyvec.hpp"
# include <cmath>
# include <string>
# include <sstream>
# include <iomanip>
# include <limits>
// havenot compiled and tested yet

phyvec::phyvec()
    : v{0.0, 0.0, 0.0}
{}

phyvec::phyvec(double x, double y, double z)
    : v{x, y, z}
{}

phyvec::phyvec(const phyvec& other) {
    v[0] = other.v[0];
    v[1] = other.v[1];
    v[2] = other.v[2];
}

phyvec::~phyvec() {}

phyvec& phyvec::operator=(const phyvec& other) {
    if (this != &other) {
        v[0] = other.v[0];
        v[1] = other.v[1];
        v[2] = other.v[2];
    }
    return *this;
}

phyvec phyvec::operator-() const {
    return phyvec(-v[0], -v[1], -v[2]);
}

phyvec phyvec::operator+(const phyvec& other) const {
    return phyvec(v[0] + other.v[0], v[1] + other.v[1], v[2] + other.v[2]);
}

phyvec phyvec::operator-(const phyvec& other) const {
    return phyvec(v[0] - other.v[0], v[1] - other.v[1], v[2] - other.v[2]);
}

phyvec phyvec::operator*(double k) const {
    return phyvec(v[0] * k, v[1] * k, v[2] * k);
}

phyvec phyvec::operator^(const phyvec& other) const{
    return phyvec(v[1] * other.v[2] - v[2] * other.v[1],
                  v[2] * other.v[0] - v[0] * other.v[2],
                  v[0] * other.v[1] - v[1] * other.v[0]);
}

phyvec operator*(double k, const phyvec& v) {
    return v * k;
}

phyvec phyvec::operator/(double k) const {
    return phyvec(v[0] / k, v[1] / k, v[2] / k);
}

bool phyvec::operator==(const phyvec& other) const {
    const double tolerance = 0.0;
    return fabs(v[0] - other.v[0]) <= tolerance &&
           fabs(v[1] - other.v[1]) <= tolerance &&
           fabs(v[2] - other.v[2]) <= tolerance;
}

bool phyvec::operator!=(const phyvec& other) const {
    return !(operator==(other));
}

double phyvec::operator*(const phyvec& other) const {
    return v[0] * other.v[0] + v[1] * other.v[1] + v[2] * other.v[2];
}

double phyvec::operator[](int i) const{
    return v[i];
}

phyvec phyvec::set(double x, int i) {
    v[i] = x;
    return *this;
}

double phyvec::abs() const {
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

std::string phyvec::to_string() const {
    std::stringstream ss;
    ss << std::scientific << std::setprecision(std::numeric_limits<double>::digits10) << v[0];
    std::string s0 = ss.str();
    ss.str("");
    ss.clear();
    ss << std::scientific << std::setprecision(std::numeric_limits<double>::digits10) << v[1];
    std::string s1 = ss.str();
    ss.str("");
    ss.clear();
    ss << std::scientific << std::setprecision(std::numeric_limits<double>::digits10) << v[2];
    std::string s2 = ss.str();
    ss.str("");
    ss.clear();
    return "[" + s0 + ", " + s1 + ", " + s2 + "]";
}