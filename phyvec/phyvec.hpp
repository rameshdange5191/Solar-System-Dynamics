# include <cmath>
# include <string>
# include <sstream>
# include <iomanip>
# include <limits>

#ifndef PHYVEC_HPP
#define PHYVEC_HPP

// one helpful function to add may be the function to compute 
// the cross product

class phyvec {
public:
    phyvec();
    phyvec(double x, double y, double z);
    phyvec(const phyvec& other);
    ~phyvec();

    phyvec& operator=(const phyvec& other);
    phyvec operator-() const;
    phyvec operator+(const phyvec& other) const;
    phyvec operator-(const phyvec& other) const;
    phyvec operator*(double k) const;
    phyvec operator/(double k) const;
    phyvec operator^(const phyvec& other) const;
    double operator*(const phyvec& other) const;
    double operator[](int i) const;
    bool operator==(const phyvec& other) const;
    bool operator!=(const phyvec& other) const;
    std::string to_string() const;
    phyvec set(double x, int i);

    double abs() const;

private:
    double v[3];
};

phyvec operator*(double k, const phyvec& v);

#endif // PHYVEC_HPP