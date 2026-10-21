// pid.hpp - PID controller with the refinements real control loops need
//
//   output = kp * error + integral(ki * error dt) - kd * measurement_rate
//
// - Derivative on the measurement: the D term uses the measured rate of
//   change (here, straight from the gyro) instead of differentiating the
//   error. A setpoint step then causes no "derivative kick", and no noisy
//   angle is differentiated.
// - The integral is stored as its contribution to the output (ki already
//   applied), so changing gains while running doesn't make the output jump.
// - Output limits, with anti-windup by conditional integration: while the
//   output is saturated, error that would push it further into saturation is
//   not integrated. Otherwise the integral grows the whole time the actuator
//   is pinned and causes a large overshoot once the error finally reverses.
// - Integral separation (optional): error is only integrated while
//   |error| <= integral_zone. During a large transient P and D do the work;
//   the integral only trims the last small offset. Without it, the integral
//   charges up during every big setpoint step and must then unwind slowly,
//   which shows up as overshoot followed by a long tail.
#pragma once

class Pid {
public:
    struct Gains {
        float kp;
        float ki;   // per second
        float kd;   // seconds
    };
    struct Limits {
        float out_min;
        float out_max;
        float integral_zone = 0.0f;   // integrate only while |error| <= this; 0 = always integrate
    };
    struct Terms {          // last update's contributions, for telemetry and tuning
        float p;
        float i;
        float d;
    };

    constexpr Pid(Gains gains, Limits limits) : gains_(gains), limits_(limits) {}

    float update(float setpoint, float measurement, float measurement_rate, float dt_s){
        const float error = setpoint - measurement;
        const float p = gains_.kp * error;
        const float d = -gains_.kd * measurement_rate;

        float i = integral_ + gains_.ki * error * dt_s;
        const float unclamped = p + i + d;
        const bool pushing_past_max = unclamped > limits_.out_max && error > 0.0f;
        const bool pushing_past_min = unclamped < limits_.out_min && error < 0.0f;
        const bool outside_zone = limits_.integral_zone > 0.0f &&
                                  (error > limits_.integral_zone || error < -limits_.integral_zone);
        if(pushing_past_max || pushing_past_min || outside_zone){
            i = integral_;                                 // hold the integral
        }
        integral_ = clamp(i);

        terms_ = {p, integral_, d};
        return clamp(p + integral_ + d);
    }

    void set_gains(Gains gains){ gains_ = gains; }
    Gains gains() const { return gains_; }
    Terms terms() const { return terms_; }

    void reset(){
        integral_ = 0.0f;
        terms_ = {};
    }

private:
    float clamp(float x) const {
        return x < limits_.out_min ? limits_.out_min : (x > limits_.out_max ? limits_.out_max : x);
    }

    Gains  gains_;
    Limits limits_;
    float  integral_ = 0.0f;
    Terms  terms_{};
};
