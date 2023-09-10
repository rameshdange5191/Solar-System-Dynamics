# include <iostream>
# include <vector>
# include <cmath>
# include <algorithm>
# include <fstream>

// save and clear are so designed that the first state is always the default state.
// runs into issues when no of years > 1. that needs to be fixed.

class AOS {
    public:
        AOS();
        AOS(double _a, double _e, double _T, double _epsilon, double _theta0, double _theta);
        AOS& set_a(double _a);
        AOS& set_e(double _e);
        AOS& set_T(double _T);
        AOS& set_theta0(double _theta0);
        AOS& set_epsilon(double _epsilon0);
        AOS& set_theta(double _theta);
        AOS& save_state(double time = -1.0);
        void state_to_csv(std::string filename) const;
        AOS& clear_state();
        double get_a() const;
        double get_e() const;
        double get_T() const;
        double get_theta0() const;
        double get_epsilon() const;
        double get_theta() const;
        double get_alt(long long int i = -1) const;
        double get_az(long long int i = -1) const;
        const std::vector <std::vector<double>>& get_state() const;
        AOS& evolve(double t0, double tf, double step_size, std::string method, bool saveState=false);
        
    private:
        double a; // semi major axis
        double e; // eccentricity
        double T; // time period of revolution
        double epsilon; // angle of tilt;
        double theta0; // angle between perihilion and vernal equinox
        double theta;
        double cal_theta_dot();
        std::vector <std::vector<double>> states;
        const std::pair<unsigned long long int, unsigned long long int>* evolve_helper(double t0, double tf, double step_size, std::string method, bool save_state=false);
};

AOS::AOS()
    : a{0.0}, e{0.0}, theta0{0.0}, theta{theta0}, T{0.0}, epsilon{0.0}
    {
        std::vector <std::vector<double>> vc;
        std::vector<double> state(2);
        state[0] = -1;
        state[1] = theta;
        vc.push_back(state);
        states = vc;  
    }

AOS::AOS (double _a, double _e, double _T, double _epsilon, double _theta0, double _theta) {
    a = _a;
    e = _e;
    T = _T;
    epsilon = _epsilon;
    theta0 = _theta0;
    theta = _theta;
    std::vector <std::vector<double>> vc;
    std::vector<double> state(2);
    state[0] = -1;
    state[1] = theta;
    vc.push_back(state);
    states = vc;  
}

AOS& AOS::set_a(double _a) {
    a = _a;
    return *this;
}

AOS& AOS::set_e(double _e) {
    e = _e;
    return *this;
}

AOS& AOS::set_T(double _T) {
    T = _T;
    return *this;
}

AOS& AOS::set_theta0(double _theta0) {
    theta0 = _theta0;
    return *this;
}

AOS& AOS::set_epsilon(double _epsilon) {
    epsilon = _epsilon;
    return *this;
}

AOS& AOS::set_theta(double _theta) {
    theta = _theta;
    return *this;
}

AOS& AOS::save_state(double time) {
    //-1.0 means that the user was just fooling around and didn't want to save the time
    std::vector<double> state(2);
    state[0] = time;
    state[1] = theta;
    states.push_back(state);
    return *this;
}

AOS& AOS::clear_state() {
    states.clear();
    save_state();
    return *this;
}

double AOS::get_a() const{
    return a;
}

double AOS::get_e() const{
    return e;
}

double AOS::get_T() const{
    return T;
}

double AOS::get_theta0() const{
    return theta0;
}

double AOS::get_epsilon() const{
    return epsilon;
}

double AOS::get_theta() const{
    return theta;
}

double AOS::cal_theta_dot() {
    return (2 * M_PI * pow((1 + e * cos(theta)), 2)) / (T * pow((1 - e * e), 1.5));
    // return (2 * M_PI / (T * (1 - 2 * e * cos(theta))));
}

double AOS::get_alt(long long int i) const {
    if (i == -1)
        return std::asin(sin(epsilon) * sin(theta - theta0));
    else
        return std::asin(sin(epsilon) * sin(states[i][1] - theta0));
}

double AOS::get_az(long long int i) const {
    double hour_angle;
    double ang_comp = ((2 * M_PI / T) * states[i][0] / (2 * M_PI) - floor(2 * M_PI/T * states[i][0] / (2 * M_PI))) * 2 * M_PI;
    if (((states[i][1] - theta0) / (2 * M_PI) - floor((states[i][1] - theta0) / (2 * M_PI))) * 2 * M_PI <= M_PI) {
        hour_angle = std::acos(cos(states[i][1] - theta0) / cos(std::asin(sin(epsilon) * sin(states[i][1] - theta0)))) - ang_comp;
        
    }
    else  {
        hour_angle = -std::acos(cos(states[i][1] - theta0) / cos(std::asin(sin(epsilon) * sin(states[i][1] - theta0)))) - ang_comp + 2 * M_PI; 
        // - 2 * M_PI - 0.07272543;
    }
    return hour_angle;
}

const std::vector <std::vector<double>>& AOS::get_state() const {
    return states;
}

const std::pair<unsigned long long int, unsigned long long int>* AOS::evolve_helper(double t0, double tf, double step_size, std::string method, bool saveState) {
    std::pair <unsigned long long int, unsigned long long int> *rng = NULL;
    unsigned long long int rng_size = 0;
    if (saveState) {
        rng = new std::pair <unsigned long long int, unsigned long long int>;
        rng->first = states.size();
    }
    double k1;
    double k2;
    double k3;
    double k4;
    // std::cout << "tf: " << tf << std::endl;
    for (double i = t0; i <= tf; i = i + step_size) {
        // std::cout << "i: " << i << std::endl;
        k1 = cal_theta_dot() * step_size;
        theta = theta + k1 / 2.0;
        k2 = cal_theta_dot() * step_size;
        theta = theta - k1 / 2.0 + k2 / 2.0;
        k3 = cal_theta_dot() * step_size;
        theta = theta - k2 / 2.0 + k3;
        k4 = cal_theta_dot() * step_size;
        theta = theta - k3;
        theta = theta + (k1 + 2 * k2 + 2 * k3 + k4) / 6.0;
        if (saveState) {
            save_state(i);
            rng_size++;
        }
    }
    if (saveState) {
        rng->second = rng->first + rng_size - 1;
    }
    return rng;
}

AOS& AOS::evolve(double t0, double tf, double step_size, std::string method, bool saveState) {
    const std::pair<unsigned long long int, unsigned long long int> *rng = evolve_helper(t0, tf, step_size, method, saveState);
    return *this;
}

void AOS::state_to_csv(std::string filename) const {
    // didn't want to add alt and az. But I am helpless
    // I am eagerly waiting for xplot to be fixed
    std::ofstream file;
    file.open(filename);
    for (int i = 0; i < states.size(); i++) {
        file << states[i][0] << ",";
        file << states[i][1] << ",";
        file << get_az(i) << ",";
        file << get_alt(i);
        file << "\n";
    }
    file.close();
}

// int main() {
//     return 0;
// }