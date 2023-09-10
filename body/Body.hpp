# include "../phyvec/phyvec.hpp"
# include <string>
# include <iostream>
# include <cmath>
# include <vector>
# include <fstream>

# ifndef BODY_HPP
# define BODY_HPP

class Body {
    public:
        static double getG();
        static void setG(double _G);
        Body();
        Body (double ms, phyvec ps, phyvec vl);
        // Body (double ms);
        Body (const Body& other);
        ~Body();
        phyvec get_pos() const;
        phyvec get_vel() const;
        double get_mass() const;
        Body& set_pos(double x, int i);
        Body& set_pos(phyvec x);
        Body& set_vel(double x, int i);
        Body& set_vel(phyvec x);
        Body& set_mass(double x);
        Body& set_body(Body& _body);
        std::string to_string() const;
        Body& save_state(double time);
        Body& clear_state();
        std::vector <std::vector<double>> get_state() const;
        void state_to_csv(std::string filename) const;
    private:
        phyvec pos;
        phyvec vel;
        double mass;
        std::vector <std::vector<double>> states;
    protected:
        static double G; // gravitational constant
};


# endif // Body_HPP