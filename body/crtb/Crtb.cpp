# include "../System.hpp"

/*
Assumptions:
1. The primary bodies, body0 and body1, are on the x-axis.
2. The tertiary body, body2, is in the xy plane.
3. w is positive 
*/


class Crtb:public System{

    public:
        Crtb();
        Crtb(Body& body0, Body& body1, Body& body2);
        double p0();
        double p1();
        phyvec L4();
        phyvec L5();
        double get_w();
        System& evolve(double t0, double tf, double step_size, std::string method, bool save_state=false);
        void light_rcom_to_csv(std::string filename);
    
    private:
        Body rcmf[3]; // rotating frame centered at COM
        Body com; // of body0 and body1

        void set_w();
        const phyvec cal_a_rcmf2();

        virtual System& set_body_internal() override;
        virtual System& set_mass_internal() override;
        virtual System& set_pos_internal() override;
        virtual System& set_vel_internal() override;
        virtual System& clear_state_internal() override;

        double get_x(double time, double x, double y);
        double get_y(double time, double x, double y);
    protected:
        double w;
};

Crtb::Crtb() 
    : System(3), rcmf{Body(), Body(), Body()}, com{Body()} {}

Crtb::Crtb(Body& body0, Body& body1, Body& body2) : System(3), rcmf{Body(), Body(), Body()}, com{Body()} {
    body[0].set_body(body0);
    body[1].set_body(body1);
    body[2].set_body(body2);
    set_body_internal();
}

double Crtb::p0() {
    return (rcmf[2].get_pos() - rcmf[0].get_pos()).abs();
}

double Crtb::p1() {
    return (rcmf[2].get_pos() - rcmf[1].get_pos()).abs();
}

phyvec Crtb::L4() {
    double dist = (body[1].get_pos() - body[0].get_pos()).abs();
    double x0 = body[0].get_pos()[0];
    double x1 = body[1].get_pos()[0];
    double y0 = body[0].get_pos()[1];
    double y1 = body[1].get_pos()[1];
    double X = (x0 + x1) / 2.0;
    double Y = (y0 + y1) / 2.0;
    double slope = (y1 - y0) / (x1 - x0);
    if (slope) {
        double splendid = (slope * dist) / 2.0 * sqrt(3 / (slope * slope + 1));
        return phyvec(X + splendid, Y - splendid, 0.0);
    }
    else {
        return phyvec(X, Y + dist / 2.0 * sqrt(3), 0.0);
    }
}

phyvec Crtb::L5() {
    double dist = (body[1].get_pos() - body[0].get_pos()).abs();
    double x0 = body[0].get_pos()[0];
    double x1 = body[1].get_pos()[0];
    double y0 = body[0].get_pos()[1];
    double y1 = body[1].get_pos()[1];
    double X = (x0 + x1) / 2.0;
    double Y = (y0 + y1) / 2.0;
    double slope = (y1 - y0) / (x1 - x0);
    if (slope) {
        double splendid = (slope * dist) / 2.0 * sqrt(3 / (slope * slope + 1));
        return phyvec(X - splendid, Y + splendid, 0.0);
    }
    else {
        return phyvec(X, Y - dist / 2.0 * sqrt(3), 0.0);
    }
}

double Crtb::get_w() {
    return w;
}

System& Crtb::set_body_internal() {
    set_mass_internal(); // should be called first
    set_pos_internal();
    set_vel_internal();
    set_w();
    return *this;
}

System& Crtb::set_mass_internal() {
    rcmf[0].set_mass(body[0].get_mass());
    rcmf[1].set_mass(body[1].get_mass());
    rcmf[2].set_mass(body[2].get_mass());
    com.set_mass(body[0].get_mass() + body[1].get_mass());
    set_w();
    return *this;
}

System& Crtb::set_pos_internal() {
    com.set_pos((body[0].get_pos() * body[0].get_mass() + body[1].get_mass() * body[1].get_pos()) / com.get_mass());
    double dist = (body[1].get_pos() - body[0].get_pos()).abs();
    rcmf[0].set_pos(-body[1].get_mass() / (com.get_mass()) * dist, 0);
    rcmf[0].set_pos(0, 1);
    rcmf[1].set_pos(body[0].get_mass() / (com.get_mass()) * dist, 0);
    rcmf[1].set_pos(0, 1);
    rcmf[2].set_pos(body[2].get_pos() - com.get_pos());
    set_w();
    return *this;
}

System& Crtb::set_vel_internal() {
    com.set_vel((body[0].get_vel() * body[0].get_mass() + body[1].get_vel() * body[1].get_mass()) / com.get_mass());
    rcmf[0].set_vel(body[0].get_vel() - com.get_vel());
    rcmf[1].set_vel(body[1].get_vel() - com.get_vel());
    rcmf[2].set_vel(body[2].get_vel() - com.get_vel() + phyvec(w * rcmf[2].get_pos()[1], -w * rcmf[2].get_pos()[0], 0.0));
    // takes care of stuff only when theta = 0
    return *this;
}

System& Crtb::clear_state_internal() {
    com.clear_state();
    for (int i = 0; i < 2; i++) {
        rcmf[i].clear_state();
    }
    return *this;
}

void Crtb::set_w() {
    w = sqrt(G * (body[0].get_mass() + body[1].get_mass()) / pow(((body[1].get_pos() - body[0].get_pos()).abs()), 3));
    // std::cout << "w: " << w << std::endl;
}

double Crtb::get_x(double time, double x, double y) {
    double theta = -w * time;
    return x*cos(theta) - y*sin(theta);
}

double Crtb::get_y(double time, double x, double y) {
    double theta = -w * time;
    return x*sin(theta) + y*cos(theta);
}

void Crtb::light_rcom_to_csv(std::string filename) { // saves the lighter body in the rotating com frame to csv
    rcmf[2].state_to_csv(filename);
}

const phyvec Crtb::cal_a_rcmf2() {
    phyvec a(0.0, 0.0, 0.0);
    a.set(2 * w * rcmf[2].get_vel()[1] + w * w * rcmf[2].get_pos()[0] 
        - G * rcmf[0].get_mass() * 
        (rcmf[2].get_pos()[0] - rcmf[0].get_pos()[0]) / (pow(p0(), 3))
        - G * rcmf[1].get_mass() *
        (rcmf[2].get_pos()[0] - rcmf[1].get_pos()[0]) / (pow(p1(), 3)), 0);
    a.set(-2 * w * rcmf[2].get_vel()[0] + w * w * rcmf[2].get_pos()[1]
        - G * rcmf[0].get_mass() *
        (rcmf[2].get_pos()[1] - rcmf[0].get_pos()[1]) / (pow(p0(), 3))
        - G * rcmf[1].get_mass() *
        (rcmf[2].get_pos()[1] - rcmf[1].get_pos()[1]) / (pow(p1(), 3)), 1);
    return a;
}

System& Crtb::evolve(double t0, double tf, double step_size, std::string method, bool saveState) {
    acc_func cal_a = std::bind(&Crtb::cal_a_rcmf2, this);
    
    const std::pair<unsigned long long int, unsigned long long int> *rng = integrate(rcmf[2], cal_a, t0, tf, step_size, method, saveState);
    // std::cout << "evolution_complete";
    // rcmf[2].state_to_csv("kirkkkwood_gap_1.csv");/
    // std::cout << com.get_mass() << "kg";
    if (saveState) {
        const std::vector<std::vector<double>>& states = rcmf[2].get_state();
        com.set_pos(com.get_pos() + states[rng->first][0] * com.get_vel());
        body[0].set_pos(rcmf[0].get_pos() + com.get_pos());
        body[1].set_pos(rcmf[1].get_pos() + com.get_pos());
        body[2].set_pos(phyvec(get_x(states[rng->first][0], states[rng->first][2], states[rng->first][3]), get_y(states[rng->first][0], states[rng->first][2], states[rng->first][3]), states[rng->first][4])); // need to ad  
        body[0].set_vel(rcmf[0].get_vel() + com.get_vel());
        body[1].set_vel(rcmf[1].get_vel() + com.get_vel());
        body[2].set_vel(phyvec(states[rng->first][5], states[rng->first][6], states[rng->first][7]) + com.get_vel()); // to be fixed
        save_state(states[rng->first][0]);
        for (int i = rng->first + 1; i <= rng->second; i++) {
            com.set_pos(com.get_pos() + com.get_vel() * (states[i][0] - states[i-1][0]));
            body[0].set_pos(rcmf[0].get_pos() + com.get_pos());
            body[1].set_pos(rcmf[1].get_pos() + com.get_pos());
            body[2].set_pos(phyvec(get_x(states[i][0], states[i][2], states[i][3]), get_y(states[i][0], states[i][2], states[i][3]) ,states[i][4]));
            body[0].set_vel(rcmf[0].get_vel() + com.get_vel());
            body[1].set_vel(rcmf[1].get_vel() + com.get_vel());
            body[2].set_vel(phyvec(states[i][5], states[i][6], states[i][7]) + com.get_vel()); // to be fixed
            save_state(states[i][0]);
        }
    }
    return *this;
}