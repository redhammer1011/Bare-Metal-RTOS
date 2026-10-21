// sim_plant.hpp - C++ port of sim/plant.py, for host closed-loop tests only
//
// Same equations, parameters and integration (RK4, <= 1 ms substeps) as the
// Python model; test_control.cpp checks the two agree on reference
// trajectories computed by sim/plant.py. Doubles here: this is the "real
// world", the controller under test uses floats like the firmware.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace sim {

constexpr double kG = 9.81;
constexpr double kPi = 3.14159265358979323846;

inline double deg(double rad){ return rad * 180.0 / kPi; }
inline double rad(double deg){ return deg * kPi / 180.0; }

struct PlantParams {
    double inertia = 0.01;
    double arm = 0.2;
    double max_thrust = 2.0;
    double mass = 0.5;
    double com_height = 0.02;
    double damping = 0.01;
    double motor_tau = 0.03;
    double stop_deg = 45.0;
    double max_substep = 0.001;
    double motor2_scale = 1.0;      // test-only: < 1 models a weaker motor 2
};

struct SensorParams {
    double gyro_bias_dps = 1.5;
    double gyro_bias_walk = 0.05;
    double gyro_noise_dps = 0.3;
    double accel_noise_g = 0.03;
};

inline constexpr SensorParams kQuietSensors{0.0, 0.0, 0.0, 0.0};

struct Imu {
    double gyro_dps;
    double accel_x_g;
    double accel_z_g;
};

class Plant {
public:
    Plant(PlantParams p = {}, SensorParams s = {}, double theta0_deg = 0.0, unsigned seed = 1)
        : p_(p), s_(s), rng_(seed), theta_(rad(theta0_deg)), gyro_bias_(s.gyro_bias_dps) {}

    void step(double cmd1, double cmd2, double dt){
        cmd1 = std::clamp(cmd1, 0.0, 1.0);
        cmd2 = std::clamp(cmd2, 0.0, 1.0);
        const int n = std::max(1, static_cast<int>(std::ceil(dt / p_.max_substep - 1e-9)));
        const double h = dt / n;
        const double stop = rad(p_.stop_deg);
        for(int i = 0; i < n; ++i){
            rk4(h, cmd1, cmd2);
            if(std::abs(theta_) >= stop){
                theta_ = std::copysign(stop, theta_);
                if(omega_ * theta_ > 0.0){ omega_ = 0.0; }
            }
        }
        gyro_bias_ += s_.gyro_bias_walk * std::sqrt(dt) * normal_(rng_);
        t_ += dt;
    }

    Imu read_sensors(){
        return {deg(omega_) + gyro_bias_ + s_.gyro_noise_dps * normal_(rng_),
                std::sin(theta_) + s_.accel_noise_g * normal_(rng_),
                std::cos(theta_) + s_.accel_noise_g * normal_(rng_)};
    }

    void set_rate_dps(double rate){ omega_ = rad(rate); }

    double theta() const     { return theta_; }
    double theta_deg() const { return deg(theta_); }
    double omega() const     { return omega_; }
    double thrust1() const   { return thrust1_; }
    double thrust2() const   { return thrust2_; }
    double time() const      { return t_; }
    double gyro_bias() const { return gyro_bias_; }

private:
    using State = std::array<double, 4>;   // theta, omega, thrust1, thrust2

    State derivatives(const State& y, double cmd1, double cmd2) const {
        const double torque = p_.arm * (y[2] - y[3])
                            + p_.mass * kG * p_.com_height * std::sin(y[0])
                            - p_.damping * y[1];
        return {y[1],
                torque / p_.inertia,
                (cmd1 * p_.max_thrust - y[2]) / p_.motor_tau,
                (cmd2 * p_.max_thrust * p_.motor2_scale - y[3]) / p_.motor_tau};
    }

    void rk4(double h, double cmd1, double cmd2){
        const State y{theta_, omega_, thrust1_, thrust2_};
        auto shifted = [&](const State& k, double f){
            State out{};
            for(int i = 0; i < 4; ++i){ out[i] = y[i] + f * k[i]; }
            return out;
        };
        const State k1 = derivatives(y, cmd1, cmd2);
        const State k2 = derivatives(shifted(k1, h / 2), cmd1, cmd2);
        const State k3 = derivatives(shifted(k2, h / 2), cmd1, cmd2);
        const State k4 = derivatives(shifted(k3, h), cmd1, cmd2);
        State next{};
        for(int i = 0; i < 4; ++i){ next[i] = y[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]); }
        theta_ = next[0]; omega_ = next[1]; thrust1_ = next[2]; thrust2_ = next[3];
    }

    PlantParams p_;
    SensorParams s_;
    std::mt19937 rng_;
    std::normal_distribution<double> normal_{0.0, 1.0};
    double t_ = 0.0;
    double theta_ = 0.0;
    double omega_ = 0.0;
    double thrust1_ = 0.0;
    double thrust2_ = 0.0;
    double gyro_bias_;
};

}  // namespace sim
