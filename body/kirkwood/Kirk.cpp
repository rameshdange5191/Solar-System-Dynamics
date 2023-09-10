# include "../System.hpp"
# include "stdio.h"
/*
Assumptions:
1. The primary bodies, body0 and body1, are on the x-axis.
2. The tertiary body, body2, is in the xy plane.
*/


class Kirk:public System{

    public:
        Kirk();
        Kirk(Body& body0, Body& body1, Body& body2);
        void save_array(std::string filename);
        void clear_array();
        double p0();
        double p1();
        phyvec L4();
        phyvec L5();
        double get_w();
        System& evolve(double t0, double tf, double step_size);
        // void to_csv(std::string filename);
    
    private:
        Body rcmf[3]; // rotating frame centered at COM
        Body com; // of body0 and body1
        int ai = 0; // number of elements stored in ar
        void set_w();
        const phyvec cal_a_rcmf2();
        double **ar; // only stores x and y pos
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

Kirk::Kirk() 
    : System(3), rcmf{Body(), Body(), Body()}, com{Body()} {}

Kirk::Kirk(Body& body0, Body& body1, Body& body2) : System(3), rcmf{Body(), Body(), Body()}, com{Body()} {
    body[0].set_body(body0);
    body[1].set_body(body1);
    body[2].set_body(body2);
    set_body_internal();
}

double Kirk::p0() {
    return (rcmf[2].get_pos() - rcmf[0].get_pos()).abs();
}

double Kirk::p1() {
    return (rcmf[2].get_pos() - rcmf[1].get_pos()).abs();
}

phyvec Kirk::L4() {
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

phyvec Kirk::L5() {
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

double Kirk::get_w() {
    return w;
}

System& Kirk::set_body_internal() {
    set_mass_internal(); // should be called first
    set_pos_internal();
    set_vel_internal();
    set_w();
    return *this;
}

System& Kirk::set_mass_internal() {
    rcmf[0].set_mass(body[0].get_mass());
    rcmf[1].set_mass(body[1].get_mass());
    rcmf[2].set_mass(body[2].get_mass());
    com.set_mass(body[0].get_mass() + body[1].get_mass());
    set_w();
    return *this;
}

System& Kirk::set_pos_internal() {
    com.set_pos((body[0].get_pos() * body[0].get_mass() + body[1].get_mass() * body[1].get_pos()) / com.get_mass());
    double dist = (body[1].get_pos() - body[0].get_pos()).abs();
    rcmf[0].set_pos(-body[1].get_mass() / (com.get_mass()) * dist, 0);
    rcmf[1].set_pos(body[0].get_mass() / (com.get_mass()) * dist, 0);
    rcmf[2].set_pos(body[2].get_pos() - com.get_pos());
    set_w();
    return *this;
}

System& Kirk::set_vel_internal() {
    com.set_vel((body[0].get_vel() * body[0].get_mass() + body[1].get_vel() * body[1].get_mass()) / com.get_mass());
    rcmf[0].set_vel(body[0].get_vel() - com.get_vel());
    rcmf[1].set_vel(body[1].get_vel() - com.get_vel());
    rcmf[2].set_vel(body[2].get_vel() - com.get_vel() + phyvec(w * rcmf[2].get_pos()[1], -w * rcmf[2].get_pos()[0], 0.0));
    // takes care of stuff only when theta = 0
    return *this;
}

void Kirk::set_w() {
    w = sqrt(G * (body[0].get_mass() + body[1].get_mass()) / pow(((body[1].get_pos() - body[0].get_pos()).abs()), 3));
    // std::cout << "w: " << w << std::endl;
}

double Kirk::get_x(double time, double x, double y) {
    double theta = -w * time;
    return x*cos(theta) - y*sin(theta);
}

double Kirk::get_y(double time, double x, double y) {
    double theta = -w * time;
    return x*sin(theta) + y*cos(theta);
}

// void Kirk::to_csv(std::string filename) {
//     rcmf[2].to_csv
// }

System& Kirk::clear_state_internal() {
    // fill it
    ;
    return *this;
}

const phyvec Kirk::cal_a_rcmf2() {
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

System& Kirk::evolve(double t0, double tf, double step_size) {
    int cnt = 0;
    double theta;
    // ar = new double*[((int) (ceil(ceil((tf - t0) / step_size) / 2000.0))) + 1];
    for (double i = t0; i <= tf; i = i + step_size) {
        // cnt = (cnt + 1) % 2000;
        phyvec k1v = cal_a_rcmf2() * step_size;
        phyvec k1x = rcmf[2].get_vel() * step_size;
        rcmf[2].set_pos(rcmf[2].get_pos() + 0.5 * k1x);
        rcmf[2].set_vel(rcmf[2].get_vel() + 0.5 * k1v);
        phyvec k2v = cal_a_rcmf2() * step_size;
        phyvec k2x = rcmf[2].get_vel() * step_size;
        rcmf[2].set_pos(rcmf[2].get_pos() - 0.5 * k1x + 0.5 * k2x);
        rcmf[2].set_vel(rcmf[2].get_vel() -0.5 * k1v + 0.5 * k2v);
        phyvec k3v = cal_a_rcmf2() * step_size;
        phyvec k3x = rcmf[2].get_vel() * step_size;
        rcmf[2].set_pos(rcmf[2].get_pos() - 0.5 * k2x + k3x);
        rcmf[2].set_vel(rcmf[2].get_vel() - 0.5 * k2v + k3v);
        phyvec k4v = cal_a_rcmf2() * step_size;
        phyvec k4x = rcmf[2].get_vel() * step_size;
        rcmf[2].set_pos(rcmf[2].get_pos() - k3x + (k1x + 2 * k2x + 2 * k3x + k4x) / 6);
        rcmf[2].set_vel(rcmf[2].get_vel() - k3v + (k1v + 2 * k2v + 2 * k3v + k4v) / 6);
        // if (!cnt) {
        //     ar[ai] = new double[2];
        //     double x = rcmf[2].get_pos()[0];
        //     double y = rcmf[2].get_pos()[1];
        //     theta = -w * i;
        //     ar[ai][0] = (x*cos(theta) + y*sin(theta));
        //     ar[ai][1] = -x*sin(theta) + y*cos(theta);
        //     ai++;
        // }
    }
    return *this;
}

void Kirk::save_array(std::string filename) {
    FILE *file = fopen(filename.c_str(), "a");
    for (int i = 0; i < ai; i++) {
        fprintf(file, "%.6e,%.6e\n", ar[i][0], ar[i][1]);
    }
    fclose(file);
}

void Kirk::clear_array() {
    for (int i = 0; i < ai; i++) {
        delete [] ar[i];
    }
    delete [] ar;
}