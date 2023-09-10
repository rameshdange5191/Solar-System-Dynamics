# include "../System.hpp"

class Twobody:public System{

    public:
        Twobody ();
        Twobody (Body& body0, Body& body1);
        System& evolve(double t0, double tf, double step_size, std::string method, bool save_state=false);
    
    private:
        Body com;
        Body body0_com;
        const phyvec cal_a_body0_com();

        virtual System& set_body_internal() override;
        virtual System& set_mass_internal() override;
        virtual System& set_pos_internal() override;
        virtual System& set_vel_internal() override;
        virtual System& clear_state_internal() override;
};

Twobody::Twobody() 
    : System(2), com{Body()}, body0_com{Body()} {}

Twobody::Twobody(Body& body0, Body& body1) : System(2), com{Body()}, body0_com{Body()} {
    body[0].set_body(body0);
    body[1].set_body(body1);
    set_body_internal();
}

System& Twobody::set_body_internal() {
    set_mass_internal(); // should be called first
    set_pos_internal();
    set_vel_internal();
    return *this;
}

System& Twobody::set_mass_internal() {
    com.set_mass(body[0].get_mass() + body[1].get_mass());
    body0_com.set_mass(body[0].get_mass());
    return *this;
}

System& Twobody::set_pos_internal() {
    com.set_pos((body[0].get_pos() * body[0].get_mass() + body[1].get_mass() * body[1].get_pos()) / com.get_mass());
    body0_com.set_pos(body[0].get_pos() - com.get_pos());
    return *this;
}

System& Twobody::set_vel_internal() {
    com.set_vel((body[0].get_vel() * body[0].get_mass() + body[1].get_vel() * body[1].get_mass()) / com.get_mass());
    body0_com.set_vel(body[0].get_vel() - com.get_vel());
    return *this;
}

System& Twobody::clear_state_internal() {
    com.clear_state();
    body0_com.clear_state();
    return *this;
}

const phyvec Twobody::cal_a_body0_com() {
    phyvec a(0.0, 0.0, 0.0);
    a = -G * body[0].get_mass() * body[1].get_mass() / com.get_mass() * body0_com.get_pos() / pow(body0_com.get_pos().abs(), 3);
    return a;
}

System& Twobody::evolve(double t0, double tf, double step_size, std::string method, bool saveState) {
    acc_func cal_a = std::bind(&Twobody::cal_a_body0_com, this);
    
    const std::pair<unsigned long long int, unsigned long long int> *rng = integrate(body0_com, cal_a, t0, tf, step_size, method, saveState);
    if (saveState) {
        const std::vector<std::vector<double>>& states = body0_com.get_state();
        com.set_pos(com.get_pos() + states[rng->first][0] * com.get_vel());
        body[0].set_pos(phyvec(states[rng->first][2], states[rng->first][3], states[rng->first][4]) + com.get_pos());
        body[1].set_pos((com.get_mass() * com.get_pos() - body[0].get_mass() * body[0].get_pos()) / body[1].get_mass());
        body[0].set_vel(phyvec(states[rng->first][5], states[rng->first][6], states[rng->first][7])  + com.get_vel());
        body[1].set_vel((com.get_mass() * com.get_vel() - body[0].get_mass() * body[0].get_vel()) / body[1].get_mass());
        save_state(states[rng->first][0]);
        for (int i = rng->first + 1; i <= rng->second; i++) {
            com.set_pos(com.get_pos() + (states[i][0] - states[i-1][0]) * com.get_vel());
            body[0].set_pos(phyvec(states[i][2], states[i][3], states[i][4]) + com.get_pos());
            body[1].set_pos((com.get_mass() * com.get_pos() - body[0].get_mass() * body[0].get_pos()) / body[1].get_mass());
            body[0].set_vel(phyvec(states[i][5], states[i][6], states[i][7])  + com.get_vel());
            body[1].set_vel((com.get_mass() * com.get_vel() - body[0].get_mass() * body[0].get_vel()) / body[1].get_mass());
            save_state(states[i][0]);
        }
    }
    return *this;
}