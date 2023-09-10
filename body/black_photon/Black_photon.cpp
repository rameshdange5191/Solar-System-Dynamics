# include "../System.hpp"

// Assumptions:
// 1. Black hole is non rotating
// 2. Black hole and the photon both start their wondeful journey from x-axis

class Black_photon: public System {
    public:
        Black_photon();
        Black_photon(Body &body0, Body &body1);
        phyvec get_l(); 
        double get_E(); 
        double get_rs();
        System& evolve(double t0, double tf, double step_size, std::string method, bool saveState=false);
    
    private:
        Body rel_pho; // relative photon
        void set_l();
        void set_E();
        void set_rs();
        const std::pair<unsigned long long int, unsigned long long int>* evolve_rel_pho(double t0, double tf, double step_size, std::string method, bool save_state);
        const double cal_r_dot(); // r dot of rel_pho
        System& set_body_external();
        virtual System& set_body_internal() override;
        virtual System& set_mass_internal() override;
        virtual System& set_pos_internal() override;
        virtual System& set_vel_internal() override;
        virtual System& clear_state_internal() override;
    protected:
        phyvec l; // angular momentum per unit mass
        double E; // Energy of the system
        double rs; // Schwarzschild radius
};



Black_photon::Black_photon()
    : System(2), rel_pho{Body()} {}

Black_photon::Black_photon(Body &body0, Body &body1)
    : System(2), rel_pho{Body()} {
    body[0].set_body(body0);
    body[1].set_body(body1);
    set_body_internal();
}

phyvec Black_photon::get_l() {
    return l;
}

double Black_photon::get_E() {
    return E;
}

void Black_photon::set_l() {
    l = rel_pho.get_pos() ^ rel_pho.get_vel();
}

void Black_photon::set_E() {
    double r_dot_mag = rel_pho.get_pos() * rel_pho.get_vel() / rel_pho.get_pos().abs();
    double r = rel_pho.get_pos().abs();
    E = 0.5 * r_dot_mag * r_dot_mag + pow(l.abs(), 2) / (2 * pow(r, 2)) * (1 - rs / r);
}  

void Black_photon::set_rs() {
    rs = 2 * G * body[0].get_mass() / (c * c);
}

const double Black_photon::cal_r_dot() {
    double r = rel_pho.get_pos().abs();
    int f = -1;
    // std::cout << "r: " << r << std::endl;
    double temp = 2 * E - 2 * (pow(l.abs(), 2) / (2 * pow(r, 2))) * (1 - rs / r); // needs to be fixed correctly
    // assumed that initially rdot < 0
    if (temp > 0)
        return f * sqrt(std::abs(temp));
    else {
        // std::cout << "Oops!" << std::endl;
        return -f * sqrt(std::abs(temp));
    }
}

System& Black_photon::set_body_internal() {
    set_mass_internal();
    set_pos_internal();
    set_vel_internal();
    return *this;
}

System& Black_photon::set_mass_internal() {
    rel_pho.set_mass(body[1].get_mass());
    return *this;
}

System& Black_photon::set_pos_internal() {
    // assuming both bodies are on the x-axis
    rel_pho.set_pos((body[1].get_pos() - body[0].get_pos()).abs(), 0);
    rel_pho.set_pos(0.0, 1);
    // std::cout << "rel_pho_pos_pos" << rel_pho.get_pos().to_string() << std::endl;
    set_l();
    set_E();
    set_rs();
    return *this;
}

System& Black_photon::set_vel_internal() {
    rel_pho.set_vel(body[1].get_vel() - body[0].get_vel());
    set_l();
    set_E();
    set_rs();
    return *this;
}

System& Black_photon::clear_state_internal() {
    rel_pho.clear_state();
    return *this;
}

const std::pair<unsigned long long int, unsigned long long int>* Black_photon::evolve_rel_pho(double t0, double tf, double step_size, std::string method, bool save_state=false) {
    std::pair <unsigned long long int, unsigned long long int> *rng = NULL;
    unsigned long long int rng_size = 0;
    if (save_state) {
        rng = new std::pair <unsigned long long int, unsigned long long int>;
        rng->first = body[1].get_state().size();
    }
    double r;
    double theta;
    double theta_dot;
    double r_dot_mag;
    std::cout << rel_pho.get_pos().to_string();
    r = rel_pho.get_pos().abs();
    if (rel_pho.get_pos()[0])
        theta = atan(rel_pho.get_pos()[1] / rel_pho.get_pos()[0]);
    else 
        theta = M_PI / 2.0;
    theta_dot = (l / (r * r)).abs();
    r_dot_mag = rel_pho.get_pos() * rel_pho.get_vel() / r;
    for (double i = t0; i <= tf; i = i + step_size) {
        if (r > rs) {
            r = r + r_dot_mag * step_size;
            theta_dot = (l / (r * r)).abs();
            theta = theta + theta_dot * step_size;
        }
        rel_pho.set_pos(phyvec(r * cos(theta), r * sin(theta), 0.0));
        rel_pho.set_vel(phyvec(r_dot_mag * cos(theta) - r * theta_dot * sin(theta), r_dot_mag * sin(theta) + r * theta_dot * cos(theta), 0));
        r_dot_mag = cal_r_dot();
        if (save_state) {
            rel_pho.save_state(i);
            rng_size++;
        }
    }
    if (save_state) {
        rng->second = rng->first + rng_size - 1;
    }
    
    return rng;
}

System& Black_photon::evolve(double t0, double tf, double step_size, std::string method, bool saveState) {
    const std::pair<unsigned long long int, unsigned long long int> *rng = evolve_rel_pho(t0, tf, step_size, method, saveState);
    if (saveState) {
        const std::vector<std::vector<double>>& states = rel_pho.get_state();
        body[1].set_pos(phyvec(states[rng->first][2], states[rng->first][3], states[rng->first][4]) + body[0].get_pos());
        body[1].set_vel(phyvec(states[rng->first][5], states[rng->first][6], states[rng->first][7]) + body[0].get_vel());
        save_state(states[rng->first][0]);
        for (int i = rng->first + 1; i <= rng->second; i++) {
            body[1].set_pos(phyvec(states[i][2], states[i][3], states[i][4]) + body[0].get_pos());
            body[1].set_vel(phyvec(states[i][5], states[i][6], states[i][7]) + body[0].get_vel());
            save_state(states[i][0]);
        }
    }
    return *this;
}