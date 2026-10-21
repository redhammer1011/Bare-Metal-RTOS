// attitude_controller.hpp - sensors in, motor commands out, for the 1-axis stand
//
//   gyro, accel --> ComplementaryFilter --> angle, rate --> Pid --> diff --> mix --> motor1, motor2
//
// Hardware-independent: the firmware's control task and the host closed-loop
// tests run this exact class. Units: degrees, deg/s, g, seconds; motor
// commands are 0..1 of full thrust.
//
// Mixing: motor1 = hover + diff, motor2 = hover - diff. Motor 1 is on the
// +angle side, so positive diff tilts toward +angle.
//
// Starting gains, from a linear model of the plant (sim/plant.py):
//   diff d changes thrust1 - thrust2 by 2 * d * 2 N, torque 0.2 m * 4 N * d,
//   so angular acceleration = 0.8 d / 0.01 kg m^2 = 80 d rad/s^2 = 4584 d deg/s^2.
//   With PD control (d = kp * e - kd * rate) the closed loop is
//     s^2 + (b/J + 4584 kd) s + (4584 kp - m g h / J) = 0,  b/J = 1, m g h / J = 9.81.
//   For natural frequency 15 rad/s and damping ratio 0.8:
//     kd = (2 * 0.8 * 15 - 1) / 4584 = 0.0050,   kp = (15^2 + 9.81) / 4584 = 0.051.
//   ki = kp / (1 s integral time) removes a steady offset (e.g. a weak motor)
//   over about a second. 15 rad/s stays well below the 30 ms motor lag
//   (33 rad/s) and the loop rate, which the model above ignores.
#pragma once

#include "complementary_filter.hpp"
#include "pid.hpp"

struct MotorCommands {
    float motor1;
    float motor2;
};

struct AttitudeConfig {
    ComplementaryFilter::Config filter;
    Pid::Gains gains;
    float diff_limit;     // max |diff|: keeps both motors inside 0..1 around hover
    float hover;          // base command for both motors
    float integral_zone;  // deg: integrate only near the setpoint (see pid.hpp)
};

inline constexpr AttitudeConfig kDefaultAttitudeConfig{
    {0.5f, 1.0f},                // filter: tau 0.5 s, bias gain 1 /s^2 (critically damped)
    {0.051f, 0.05f, 0.0050f},    // kp, ki, kd
    0.4f,
    0.5f,
    2.0f,
};

class AttitudeController {
public:
    constexpr explicit AttitudeController(const AttitudeConfig& config = kDefaultAttitudeConfig)
        : config_(config),
          filter_(config.filter),
          pid_(config.gains, {-config.diff_limit, config.diff_limit, config.integral_zone}) {}

    // Before arming: one call per sample while the stand is held still.
    void calibrate(float gyro_dps, float accel_x_g, float accel_z_g){
        filter_.calibrate(gyro_dps, accel_x_g, accel_z_g);
    }

    MotorCommands update(float setpoint_deg, float gyro_dps, float accel_x_g, float accel_z_g, float dt_s){
        filter_.update(gyro_dps, accel_x_g, accel_z_g, dt_s);
        const float diff = pid_.update(setpoint_deg, filter_.angle_deg(), filter_.rate_dps(), dt_s);
        return {clamp01(config_.hover + diff), clamp01(config_.hover - diff)};
    }

    void set_gains(Pid::Gains gains){ pid_.set_gains(gains); }

    float angle_deg() const { return filter_.angle_deg(); }
    float rate_dps() const  { return filter_.rate_dps(); }
    float bias_dps() const  { return filter_.bias_dps(); }
    Pid::Terms terms() const { return pid_.terms(); }
    Pid::Gains gains() const { return pid_.gains(); }

    void reset(){
        filter_.reset();
        pid_.reset();
    }

private:
    static float clamp01(float x){ return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

    AttitudeConfig      config_;
    ComplementaryFilter filter_;
    Pid                 pid_;
};
