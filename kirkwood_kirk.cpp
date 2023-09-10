# include "body/kirkwood/Kirk.cpp"
# include "phyvec/phyvec.cpp"
# include <iostream>
# include <cmath>
# include <functional>
# include <string>
# include <thread>
# include <vector>
# include "integrator/integrator.cpp"
# include "body/Body.cpp"
# include "body/System.cpp"
# include <chrono>
# include <random>
using namespace std;

string extension = ".csv";

double m_to_au = 6.68459e-12;
double mtr_p_s_to_kmps = 0.001;
double kg_to_sun_mass = 1.98847e-30;
double sun_mass = 1;
double jupiter_mass = 1.89813e27 * kg_to_sun_mass;
double G = 6.67408e-11 * pow(m_to_au, 3)  / (kg_to_sun_mass);

double circ_orb_vel(double r, double mass = sun_mass) {
    return sqrt(G * mass / r);
}

void evolution (Kirk &a, double start, double evolution_duration, double step_size) {
    a.evolve(start, evolution_duration, step_size);
}

void process_asteroid(vector<Kirk> &a, double start, double evolution_duration, double step_size, int thread_number, int asteroid_start, int asteroid_end) {
    char thread_number_char = (char) (thread_number + '0');
    string filename_prefix = "kirkout/kirk_gap_lab_end_abracadabra_";
    filename_prefix.push_back(thread_number_char);
    filename_prefix = filename_prefix + extension;
    for (int i = asteroid_start; i <= asteroid_end; i++) {
        a[i].evolve(start, evolution_duration, step_size);
        a[i].save_array(filename_prefix);
        a[i].clear_array();
    }
}

int main() {
    Body sun(sun_mass, phyvec(0, 0.0, 0.0), phyvec(0.0, -circ_orb_vel(778.479e9 * m_to_au, jupiter_mass), 0.0));
    Body jupiter(jupiter_mass, phyvec(778.479e9 * m_to_au, 0.0, 0.0), phyvec(0.0, circ_orb_vel(778.479e9 * m_to_au), 0.0));
    double asteriod_start = 2.2;
    double asteriod_end = 3.2;
    double asteriod_length = asteriod_end - asteriod_start;
    double asteriod_mean_starting_vel = 0.44704;

    double evolution_duration = 100 * 365.25 * 24 * 60 * 60;
    double start = 0.0;
    double step_size = 6.5 * 24 * 60 * 60;

    unsigned long long int no_of_asteroids = 10;
    unsigned long long int no_of_threads = 10;
    unsigned long long int no_of_asteroids_per_thread = no_of_asteroids / no_of_threads;
    vector <std::thread> threads(no_of_threads);
    vector <Kirk> asteroids(no_of_asteroids);

    const unsigned int seedValue = 42;
    mt19937 gen(seedValue);
    uniform_real_distribution<double> r_dis(1.6, 3.6);
    uniform_real_distribution<double> theta_dis(0, 2 * M_PI);

    for (int i = 0; i < no_of_asteroids; i++) {
        double asteroid_r = r_dis(gen);
        double asteroid_theta = theta_dis(gen);
        double asteroid_speed = circ_orb_vel(asteroid_r);
        Body asteroid(0.0, phyvec(asteroid_r * cos(asteroid_theta), asteroid_r * sin(asteroid_theta), 0.0), phyvec(-asteroid_speed * sin(asteroid_theta), asteroid_speed * cos(asteroid_theta), 0.0));
        asteroids[i].setG(G);
        asteroids[i].set_body(sun, 0);
        asteroids[i].set_body(jupiter, 1);
        asteroids[i].set_body(asteroid, 2);
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < no_of_threads; i++) {
        threads[i] = std::thread(process_asteroid, std::ref(asteroids), start, evolution_duration, step_size, i, i * no_of_asteroids_per_thread, (i + 1) * no_of_asteroids_per_thread - 1);
    }
    for (auto& thread : threads) {
        thread.join();
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    cout << "Total time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << endl;
}
