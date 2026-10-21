// complementary_filter.hpp - fuse a gyro and an accelerometer into one angle
//
// The gyro gives a smooth angle when integrated, but its bias makes the
// integral drift without limit. The accelerometer's gravity direction gives
// an angle that never drifts but is noisy (and, on a moving vehicle, upset by
// acceleration). Each is good where the other is bad:
//
//   predicted = angle + (gyro - bias) * dt               short term: trust the gyro
//   angle     = a * predicted + (1 - a) * accel_angle     long term: pull toward the accelerometer
//   a         = tau / (tau + dt)
//
// so motion faster than the time constant tau comes from the gyro and slower
// motion from the accelerometer. A plain complementary filter still settles
// bias * tau away from the truth (0.75 deg for 1.5 deg/s and 0.5 s), so the
// bias is also estimated: a persistent disagreement between the prediction
// and the accelerometer can only be gyro bias, so it is slowly integrated
// into `bias` and subtracted. With bias_gain = 0 this is the plain filter.
//
// Angles in degrees, rates in deg/s, accelerations in g. Assumes the
// accelerometer only sees gravity (IMU on the pivot axis).
//
// No math header: the firmware builds with -ffreestanding, where GCC 15's
// libstdc++ refuses <cmath>, and its C++ <math.h> is only a wrapper that
// includes <cmath>. __builtin_atan2f needs no declaration; GCC lowers it to a
// call to atan2f from the C math library (newlib's libm on the target, linked
// with -lm), on the host and target alike.
#pragma once

class ComplementaryFilter {
public:
    struct Config {
        float time_constant_s;   // gyro/accelerometer crossover
        float bias_gain;         // 1/s^2: how quickly the bias estimate follows; 0 disables it
    };

    constexpr explicit ComplementaryFilter(Config config) : config_(config) {}

    // Startup calibration, before control: call once per sample while the
    // sensor is held still. The gyro's true rate is then zero, so its average
    // IS the bias; the averaged accelerometer angle is a low-noise starting
    // angle. Each call leaves the running average in place, so update() can
    // follow at any point. Without it, the first update() starts from one
    // noisy accelerometer sample and a zero bias, which the bias estimator
    // then takes seconds to correct.
    void calibrate(float gyro_dps, float accel_x_g, float accel_z_g){
        ++cal_samples_;
        cal_gyro_sum_ += gyro_dps;
        cal_angle_sum_ += __builtin_atan2f(accel_x_g, accel_z_g) * kRadToDeg;
        bias_ = cal_gyro_sum_ / static_cast<float>(cal_samples_);
        angle_ = cal_angle_sum_ / static_cast<float>(cal_samples_);
        rate_ = 0.0f;
        initialized_ = true;
    }

    void update(float gyro_dps, float accel_x_g, float accel_z_g, float dt_s){
        const float accel_angle = __builtin_atan2f(accel_x_g, accel_z_g) * kRadToDeg;
        rate_ = gyro_dps - bias_;
        if(!initialized_){                      // start at the accelerometer's angle: no slow pull-in
            angle_ = accel_angle;
            initialized_ = true;
            return;
        }
        const float predicted = angle_ + rate_ * dt_s;
        const float alpha = config_.time_constant_s / (config_.time_constant_s + dt_s);
        angle_ = alpha * predicted + (1.0f - alpha) * accel_angle;
        // Gyro reading high -> prediction runs ahead -> accel_angle below angle_ on average.
        bias_ -= config_.bias_gain * (accel_angle - angle_) * dt_s;
    }

    float angle_deg() const { return angle_; }
    float rate_dps() const  { return rate_; }    // gyro rate with the estimated bias removed
    float bias_dps() const  { return bias_; }

    void reset(){
        angle_ = 0.0f;
        rate_ = 0.0f;
        bias_ = 0.0f;
        initialized_ = false;
        cal_samples_ = 0;
        cal_gyro_sum_ = 0.0f;
        cal_angle_sum_ = 0.0f;
    }

private:
    static constexpr float kRadToDeg = 57.29577951308232f;

    Config config_;
    float angle_ = 0.0f;
    float rate_ = 0.0f;
    float bias_ = 0.0f;
    bool  initialized_ = false;
    unsigned cal_samples_ = 0;
    float cal_gyro_sum_ = 0.0f;
    float cal_angle_sum_ = 0.0f;
};
