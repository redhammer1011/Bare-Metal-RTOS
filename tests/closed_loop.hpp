// closed_loop.hpp - run AttitudeController against the C++ plant on the host
//
// Same order of operations as the firmware's lockstep loop: read the IMU,
// compute motor commands, hold them for one period while the plant advances.
// Used by test_control.cpp and by the closed_loop_sim tool (CSV for plots).
#pragma once

#include "attitude_controller.hpp"
#include "sim_plant.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace sim {

struct Scenario {
    double calibration_s = 0.5;        // held still before arming (the stand is clamped: no physics)
    double duration_s = 4.0;
    double dt = 0.005;                 // control period: 200 Hz
    double theta0_deg = 0.0;
    double step_time = 0.5;            // setpoint: from `step_from` to `step_to` at step_time
    double step_from = 0.0;
    double step_to = 10.0;
    PlantParams plant{};
    SensorParams sensors{};
    unsigned seed = 1;
    AttitudeConfig config = kDefaultAttitudeConfig;
};

struct Sample {
    double t;
    double setpoint;
    double theta;          // truth
    double estimate;       // filter output
    double bias_est;
    double bias_true;
    double motor1;
    double motor2;
};

inline std::vector<Sample> run_closed_loop(const Scenario& sc){
    Plant plant(sc.plant, sc.sensors, sc.theta0_deg, sc.seed);
    AttitudeController ctrl(sc.config);
    const int cal_steps = static_cast<int>(std::lround(sc.calibration_s / sc.dt));
    for(int k = 0; k < cal_steps; ++k){
        const Imu imu = plant.read_sensors();
        ctrl.calibrate(static_cast<float>(imu.gyro_dps), static_cast<float>(imu.accel_x_g),
                       static_cast<float>(imu.accel_z_g));
    }
    std::vector<Sample> log;
    const int steps = static_cast<int>(std::lround(sc.duration_s / sc.dt));
    log.reserve(static_cast<std::size_t>(steps));
    for(int k = 0; k < steps; ++k){
        const double t = k * sc.dt;
        const double sp = t >= sc.step_time ? sc.step_to : sc.step_from;
        const Imu imu = plant.read_sensors();
        const MotorCommands cmd = ctrl.update(static_cast<float>(sp), static_cast<float>(imu.gyro_dps),
                                              static_cast<float>(imu.accel_x_g), static_cast<float>(imu.accel_z_g),
                                              static_cast<float>(sc.dt));
        log.push_back({t, sp, plant.theta_deg(), ctrl.angle_deg(), ctrl.bias_dps(), plant.gyro_bias(),
                       cmd.motor1, cmd.motor2});
        plant.step(cmd.motor1, cmd.motor2, sc.dt);
    }
    return log;
}

// ------------------------------------------------------------------ metrics
struct StepMetrics {
    double overshoot_pct;      // peak beyond the target, as % of the step size
    double settling_s;         // time after the step until the angle stays within band of the target
    double steady_error_deg;   // mean |theta - target| over the last second
    double steady_rms_deg;     // rms of theta - target over the last second
    double max_abs_deg;        // largest |theta| seen
};

inline StepMetrics step_metrics(const std::vector<Sample>& log, const Scenario& sc, double band_deg = 0.5){
    const double size = sc.step_to - sc.step_from;
    double peak = sc.step_from;
    double last_outside = sc.step_time;
    double max_abs = 0.0;
    for(const Sample& s : log){
        max_abs = std::max(max_abs, std::abs(s.theta));
        if(s.t < sc.step_time){ continue; }
        if(size >= 0 ? s.theta > peak : s.theta < peak){ peak = s.theta; }
        if(std::abs(s.theta - sc.step_to) > band_deg){ last_outside = s.t; }
    }
    double sum_abs = 0.0, sum_sq = 0.0;
    int n = 0;
    for(const Sample& s : log){
        if(s.t < sc.duration_s - 1.0){ continue; }
        const double e = s.theta - s.setpoint;
        sum_abs += std::abs(e);
        sum_sq += e * e;
        ++n;
    }
    const double overshoot = size != 0.0 ? std::max(0.0, (peak - sc.step_to) / size * 100.0) : 0.0;
    return {overshoot, last_outside + sc.dt - sc.step_time, sum_abs / n, std::sqrt(sum_sq / n), max_abs};
}

}  // namespace sim
