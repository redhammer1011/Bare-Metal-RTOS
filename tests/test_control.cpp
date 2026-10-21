// test_control.cpp - complementary filter, PID, and closed-loop control of the
// C++ plant model. Requirements for the default tuning (set before tuning):
//   10 deg step: overshoot < 15 %, settles within +/-0.5 deg in < 1 s,
//   steady-state |error| < 0.2 deg with the default gyro bias and noise.

#include "attitude_controller.hpp"
#include "closed_loop.hpp"
#include "complementary_filter.hpp"
#include "pid.hpp"
#include "sim_plant.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {
    // Mean and standard deviation of (estimate - truth) after `skip_s`.
    struct ErrorStats { double mean; double stdev; };

    ErrorStats filter_error_at_rest(ComplementaryFilter::Config cfg, double tilt_deg, double seconds,
                                    double skip_s, double* final_bias_error = nullptr,
                                    sim::SensorParams sensors = {}){
        sim::PlantParams params;
        params.stop_deg = tilt_deg;                       // resting on the end stop
        sim::Plant plant(params, sensors, tilt_deg, 11);
        ComplementaryFilter filter(cfg);
        const double dt = 0.005;
        double sum = 0.0, sum_sq = 0.0;
        int n = 0;
        for(int k = 0; k * dt < seconds; ++k){
            const sim::Imu imu = plant.read_sensors();
            filter.update(static_cast<float>(imu.gyro_dps), static_cast<float>(imu.accel_x_g),
                          static_cast<float>(imu.accel_z_g), static_cast<float>(dt));
            plant.step(0.0, 0.0, dt);
            if(k * dt >= skip_s){
                const double e = filter.angle_deg() - plant.theta_deg();
                sum += e;
                sum_sq += e * e;
                ++n;
            }
        }
        if(final_bias_error){ *final_bias_error = filter.bias_dps() - plant.gyro_bias(); }
        const double mean = sum / n;
        return {mean, std::sqrt(sum_sq / n - mean * mean)};
    }
}

// ================================================================ plant port

// Reference values from sim/plant.py (Plant(sensors=quiet, theta0_deg=2.0),
// step(0.6, 0.45, 0.005) x 60): the C++ port must follow the same physics.
TEST(SimPlant, MatchesPythonModel){
    sim::Plant plant({}, sim::kQuietSensors, 2.0);
    for(int i = 0; i < 60; ++i){ plant.step(0.6, 0.45, 0.005); }
    EXPECT_NEAR(plant.theta(),   0.2649880671440316, 1e-12);
    EXPECT_NEAR(plant.omega(),   1.6995217210278828, 1e-12);
    EXPECT_NEAR(plant.thrust1(), 1.199945520078522,  1e-12);
    EXPECT_NEAR(plant.thrust2(), 0.8999591400588918, 1e-12);
}

// ======================================================== complementary filter

TEST(ComplementaryFilter, StartsAtTheAccelerometerAngle){
    ComplementaryFilter f({0.5f, 1.0f});
    f.update(100.0f, std::sin(0.3f), std::cos(0.3f), 0.005f);    // gyro ignored on the first sample
    EXPECT_NEAR(f.angle_deg(), 0.3 * 180.0 / sim::kPi, 1e-4);
}

TEST(ComplementaryFilter, CalibrationAveragesOutBiasAndNoise){
    sim::Plant plant({}, {}, 4.0, 5);                      // held still at 4 deg (no step(): clamped)
    ComplementaryFilter f({0.5f, 1.0f});
    for(int k = 0; k < 100; ++k){                          // 0.5 s at 200 Hz
        const sim::Imu imu = plant.read_sensors();
        f.calibrate(static_cast<float>(imu.gyro_dps), static_cast<float>(imu.accel_x_g),
                    static_cast<float>(imu.accel_z_g));
    }
    // 100 samples shrink the noise 10x: gyro 0.3 -> 0.03 deg/s, accel angle ~1.7 -> ~0.17 deg (1 sigma).
    EXPECT_NEAR(f.bias_dps(), plant.gyro_bias(), 0.1);
    EXPECT_NEAR(f.angle_deg(), 4.0, 0.5);
}

TEST(ComplementaryFilter, PlainFilterSettlesBiasTimesTauOff){
    // Why bias estimation exists: without it, 1.5 deg/s * 0.5 s = 0.75 deg error, forever.
    // Bias random walk off, so the bias really is 1.5 deg/s throughout.
    sim::SensorParams sensors;
    sensors.gyro_bias_walk = 0.0;
    const ErrorStats e = filter_error_at_rest({0.5f, 0.0f}, 10.0, 20.0, 5.0, nullptr, sensors);
    EXPECT_NEAR(e.mean, 0.75, 0.03);
}

TEST(ComplementaryFilter, BiasEstimationRemovesTheOffset){
    double bias_error = 0.0;
    const ErrorStats e = filter_error_at_rest({0.5f, 1.0f}, 10.0, 30.0, 10.0, &bias_error);
    EXPECT_LT(std::abs(e.mean), 0.1);
    EXPECT_LT(e.stdev, 0.3);                  // accelerometer alone scatters ~1.7 deg (1 sigma)
    EXPECT_LT(std::abs(bias_error), 0.15);
}

TEST(ComplementaryFilter, FollowsFastMotionWithoutLag){
    // Constant spin, perfect sensors: the gyro carries fast motion, so the
    // estimate tracks with no time-constant lag.
    sim::PlantParams params;
    params.com_height = 0.0;
    params.damping = 0.0;
    params.stop_deg = 89.0;
    sim::Plant plant(params, sim::kQuietSensors, -30.0);
    plant.set_rate_dps(60.0);                              // no torque, no damping: spins at a constant 60 deg/s
    ComplementaryFilter f({0.5f, 1.0f});
    const double dt = 0.005;
    double worst = 0.0;
    for(int k = 0; k < 200; ++k){                          // 1 s: -30 deg -> +30 deg
        const sim::Imu imu = plant.read_sensors();
        f.update(static_cast<float>(imu.gyro_dps), static_cast<float>(imu.accel_x_g),
                 static_cast<float>(imu.accel_z_g), static_cast<float>(dt));
        worst = std::max(worst, std::abs(f.angle_deg() - plant.theta_deg()));
        plant.step(0.0, 0.0, dt);
    }
    EXPECT_LT(worst, 0.05);
}

// ===================================================================== PID

TEST(Pid, ProportionalTerm){
    Pid pid({2.0f, 0.0f, 0.0f}, {-100.0f, 100.0f});
    EXPECT_FLOAT_EQ(pid.update(10.0f, 4.0f, 0.0f, 0.01f), 12.0f);
}

TEST(Pid, IntegralAccumulatesErrorTimesTime){
    Pid pid({0.0f, 3.0f, 0.0f}, {-100.0f, 100.0f});
    float out = 0.0f;
    for(int i = 0; i < 100; ++i){ out = pid.update(1.0f, 0.0f, 0.0f, 0.01f); }   // 1 s of error 1
    EXPECT_NEAR(out, 3.0f, 1e-4f);
}

TEST(Pid, DerivativeOpposesMeasuredRate){
    Pid pid({0.0f, 0.0f, 0.5f}, {-100.0f, 100.0f});
    EXPECT_FLOAT_EQ(pid.update(0.0f, 0.0f, 4.0f, 0.01f), -2.0f);
}

TEST(Pid, SetpointStepCausesNoDerivativeKick){
    Pid pid({1.0f, 0.0f, 0.5f}, {-100.0f, 100.0f});
    const float before = pid.update(0.0f, 0.0f, 0.0f, 0.01f);
    const float after = pid.update(50.0f, 0.0f, 0.0f, 0.01f);      // setpoint jumps by 50
    EXPECT_FLOAT_EQ(after - before, 50.0f);                        // P only; D sees no rate
}

TEST(Pid, OutputIsClamped){
    Pid pid({10.0f, 0.0f, 0.0f}, {-1.0f, 1.0f});
    EXPECT_FLOAT_EQ(pid.update(5.0f, 0.0f, 0.0f, 0.01f), 1.0f);
    EXPECT_FLOAT_EQ(pid.update(-5.0f, 0.0f, 0.0f, 0.01f), -1.0f);
}

TEST(Pid, AntiWindupLetsOutputRecoverImmediately){
    // P alone (1 * 20) pins the output at +1 for 5 s. Clamping the integral to
    // the output range isn't enough: it would still charge to +1 and then hold
    // the output positive after the error reverses. Conditional integration
    // keeps it at 0, so the output follows the new error at once.
    // (Mutation testing showed the first version of this test, with a small
    // kp, passed even with conditional integration removed.)
    Pid pid({1.0f, 1.0f, 0.0f}, {-1.0f, 1.0f});
    for(int i = 0; i < 500; ++i){ pid.update(20.0f, 0.0f, 0.0f, 0.01f); }
    EXPECT_FLOAT_EQ(pid.terms().i, 0.0f);
    const float out = pid.update(-0.5f, 0.0f, 0.0f, 0.01f);       // error reverses, small
    EXPECT_LT(out, 0.0f);                                          // a wound-up integral would give ~ +0.5
}

TEST(Pid, ChangingGainsDoesNotJumpTheOutput){
    Pid pid({0.0f, 2.0f, 0.0f}, {-100.0f, 100.0f});
    for(int i = 0; i < 100; ++i){ pid.update(1.0f, 0.0f, 0.0f, 0.01f); }
    const float before = pid.update(0.0f, 0.0f, 0.0f, 0.01f);     // error now 0: output = stored integral
    pid.set_gains({0.0f, 50.0f, 0.0f});
    EXPECT_FLOAT_EQ(pid.update(0.0f, 0.0f, 0.0f, 0.01f), before);
}

// ========================================================= attitude control

TEST(AttitudeController, MixesAroundHoverAndClampsMotors){
    AttitudeConfig cfg = kDefaultAttitudeConfig;
    cfg.gains = {1.0f, 0.0f, 0.0f};
    cfg.diff_limit = 0.8f;                          // allow |diff| > hover to exercise the clamp
    AttitudeController c(cfg);
    MotorCommands m = c.update(0.1f, 0.0f, 0.0f, 1.0f, 0.005f);      // level, small positive error
    EXPECT_NEAR(m.motor1 - 0.5f, 0.5f - m.motor2, 1e-6f);             // symmetric about hover
    EXPECT_GT(m.motor1, m.motor2);                                    // +error -> more motor 1
    m = c.update(10.0f, 0.0f, 0.0f, 1.0f, 0.005f);
    EXPECT_FLOAT_EQ(m.motor1, 1.0f);
    EXPECT_FLOAT_EQ(m.motor2, 0.0f);
}

TEST(ClosedLoop, StepResponseMeetsRequirements){
    sim::Scenario sc;                                // 0 -> 10 deg at 0.5 s, default noise and bias
    const auto m = sim::step_metrics(sim::run_closed_loop(sc), sc);
    EXPECT_LT(m.overshoot_pct, 15.0);
    EXPECT_LT(m.settling_s, 1.0);
    EXPECT_LT(m.steady_error_deg, 0.2);
    RecordProperty("overshoot_pct", std::to_string(m.overshoot_pct));
    RecordProperty("settling_s", std::to_string(m.settling_s));
    RecordProperty("steady_error_deg", std::to_string(m.steady_error_deg));
}

TEST(ClosedLoop, RecoversToLevelFromATilt){
    sim::Scenario sc;
    sc.theta0_deg = 8.0;
    sc.step_time = 0.0;
    sc.step_to = 0.0;
    sc.step_from = 0.0;
    const auto log = sim::run_closed_loop(sc);
    double last_outside = 0.0;
    for(const auto& s : log){ if(std::abs(s.theta) > 0.5){ last_outside = s.t; } }
    EXPECT_LT(last_outside, 1.0);
    EXPECT_LT(sim::step_metrics(log, sc).steady_error_deg, 0.2);
}

TEST(ClosedLoop, IntegralTermCancelsAWeakMotor){
    sim::Scenario sc;
    sc.plant.motor2_scale = 0.9;                     // motor 2 makes 10 % less thrust
    EXPECT_LT(sim::step_metrics(sim::run_closed_loop(sc), sc).steady_error_deg, 0.2);

    sc.config.gains.ki = 0.0f;                       // PD only: a standing offset remains
    EXPECT_GT(sim::step_metrics(sim::run_closed_loop(sc), sc).steady_error_deg, 0.5);
}

TEST(ClosedLoop, WithoutCalibrationOrBiasEstimationTheAngleSettlesOff){
    // The controller holds the *estimate* on target, so a biased estimate puts
    // the real bar bias * tau (0.75 deg) off. Calibration or bias estimation
    // each fixes it; this run has neither.
    sim::Scenario sc;
    sc.duration_s = 8.0;
    sc.calibration_s = 0.0;
    sc.config.filter.bias_gain = 0.0f;
    EXPECT_GT(sim::step_metrics(sim::run_closed_loop(sc), sc).steady_error_deg, 0.5);
}

TEST(Pid, IntegralZoneHoldsIntegralDuringLargeErrors){
    Pid pid({0.0f, 1.0f, 0.0f}, {-100.0f, 100.0f, 2.0f});
    pid.update(10.0f, 0.0f, 0.0f, 0.1f);              // |error| 10 > zone 2: not integrated
    EXPECT_FLOAT_EQ(pid.terms().i, 0.0f);
    pid.update(1.0f, 0.0f, 0.0f, 0.1f);               // inside the zone
    EXPECT_FLOAT_EQ(pid.terms().i, 0.1f);
}

TEST(ClosedLoop, LargeStepSaturatesWithoutWindupOvershoot){
    // A 30 deg step pins the motors at full differential thrust for ~245 ms,
    // and the bar then overshoots ~16 % because the limited thrust can't brake
    // it in time: an actuator limit, present even with no integral term at all.
    // (An earlier version of this test demanded < 15 % here, which mixed that
    // physical limit with windup.) What anti-windup and the integral zone must
    // guarantee is that the integral adds no overshoot of its own.
    sim::Scenario sc;
    sc.step_to = 30.0;
    const auto with_integral = sim::step_metrics(sim::run_closed_loop(sc), sc);
    sim::Scenario pd = sc;
    pd.config.gains.ki = 0.0f;
    const auto pd_only = sim::step_metrics(sim::run_closed_loop(pd), pd);

    EXPECT_LT(with_integral.overshoot_pct - pd_only.overshoot_pct, 1.0);
    EXPECT_LT(with_integral.max_abs_deg, 40.0);      // never reaches the 45 deg stop
}

TEST(ClosedLoop, IntegralZoneRemovesTheIntegralOvershootTail){
    sim::Scenario sc;                                // the 10 deg step from the requirements
    sim::Scenario no_zone = sc;
    no_zone.config.integral_zone = 0.0f;
    const auto with_zone = sim::step_metrics(sim::run_closed_loop(sc), sc);
    const auto without = sim::step_metrics(sim::run_closed_loop(no_zone), no_zone);
    EXPECT_LT(with_zone.overshoot_pct, without.overshoot_pct - 3.0);   // measured: 11.5 % vs 18.1 %
    EXPECT_LT(with_zone.settling_s, without.settling_s / 2.0);         // measured: 0.32 s vs 1.27 s
}

TEST(ClosedLoop, StillStableAtHalfTheLoopRate){
    sim::Scenario sc;
    sc.dt = 0.010;                                   // 100 Hz, in case the link can't do 200
    const auto m = sim::step_metrics(sim::run_closed_loop(sc), sc);
    EXPECT_LT(m.overshoot_pct, 25.0);
    EXPECT_LT(m.steady_error_deg, 0.3);
}
