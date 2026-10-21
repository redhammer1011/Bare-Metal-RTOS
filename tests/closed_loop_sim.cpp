// closed_loop_sim.cpp - host tool: run one closed-loop scenario, print CSV
//
//   closed_loop_sim [step_to_deg] [duration_s] [dt_s]
//
// Columns: t,setpoint,theta,estimate,bias_est,bias_true,motor1,motor2
// sim/plot_closed_loop.py runs this and plots the result.

#include "closed_loop.hpp"

#include <cstdio>
#include <cstdlib>

int main(int argc, char** argv){
    sim::Scenario sc;
    if(argc > 1){ sc.step_to = std::atof(argv[1]); }
    if(argc > 2){ sc.duration_s = std::atof(argv[2]); }
    if(argc > 3){ sc.dt = std::atof(argv[3]); }

    std::printf("t,setpoint,theta,estimate,bias_est,bias_true,motor1,motor2\n");
    for(const sim::Sample& s : sim::run_closed_loop(sc)){
        std::printf("%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    s.t, s.setpoint, s.theta, s.estimate, s.bias_est, s.bias_true, s.motor1, s.motor2);
    }
    const sim::StepMetrics m = sim::step_metrics(sim::run_closed_loop(sc), sc);
    std::fprintf(stderr, "overshoot %.1f %%, settling %.2f s, steady error %.3f deg, steady rms %.3f deg\n",
                 m.overshoot_pct, m.settling_s, m.steady_error_deg, m.steady_rms_deg);
    return 0;
}
