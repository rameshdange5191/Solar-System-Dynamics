# include <iostream>
# include <string>
# include <vector>
# include "../phyvec/phyvec.hpp"
# include "../integrator/integrator.cpp"
# include "Body.hpp"

#ifndef SYSTEM_HPP
#define SYSTEM_HPP

class System {
    friend const std::pair<unsigned long long int, unsigned long long int> *fwd_euler(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state);
    friend const std::pair<unsigned long long int, unsigned long long int> *eulric(Body& body, acc_func& af, double t0, double tf, double step_size, bool save_state);
    friend const std::pair<unsigned long long int, unsigned long long int> *integrate(Body& body, acc_func& af, double t0, double tf, double step_size, std::string method, bool save_state);
    public:
        System();
        System(int n);
        static double getG();
        static void setG(double _G);
        static double get_c();
        static void set_c(double _c);
        const Body& operator[] (int i) const; 
        System& set_body(Body& _body, int i);
        System& set_mass(double mass, int i);
        System& set_pos(double x, int i, int j);
        System& set_pos(phyvec x, int i);
        System& set_vel(double x, int i, int j);
        System& set_vel(phyvec x, int i);
        System& save_state(double time);
        System& clear_state();
        void state_to_csv(std::string filename, int i) const;
    private:
        virtual System& set_body_internal() = 0;
        virtual System& set_mass_internal() = 0;
        virtual System& set_pos_internal() = 0;
        virtual System& set_vel_internal() = 0;
        virtual System& clear_state_internal() = 0;
    protected:
        static double G; // gravitational constant
        static double c; // speed of light in vacuum
        std::vector <Body> body;
};

#endif // SYSTEM_HPP