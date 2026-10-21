// scheduler.hpp - time-triggered cooperative scheduler (cyclic executive)
//
// Tasks are plain functions that run to completion and must never block.
// Each task runs every period_ms on a fixed timeline: its next due time
// advances by exactly one period, so running late once doesn't shift later
// runs. If a task falls behind by whole periods, the missed deadlines are
// counted as overruns and skipped rather than run back-to-back.
//
// Hardware-independent: the caller passes the current time into run(), so
// the same code runs on the target (run(millis())) and in host unit tests
// with a fake clock. Time comparisons are wrap-safe for a 32-bit ms counter.
#pragma once

#include <cstddef>
#include <cstdint>

template <std::size_t MaxTasks>
class Scheduler {
    static_assert(MaxTasks > 0, "Scheduler needs room for at least one task");

public:
    using TaskFn = void (*)();

    // Register a task. First run at now_ms + offset_ms, then every period_ms.
    // Returns false (and adds nothing) if fn is null, period_ms is 0, or the
    // table is full.
    bool add(TaskFn fn, std::uint32_t period_ms, std::uint32_t now_ms = 0, std::uint32_t offset_ms = 0){
        if(fn == nullptr || period_ms == 0 || count_ == MaxTasks){
            return false;
        }
        tasks_[count_] = Task{fn, period_ms, now_ms + offset_ms, 0, 0};
        ++count_;
        return true;
    }

    // Run every task that is due at now_ms, in the order they were added.
    // Call as often as possible; a task can only start when run() is called.
    void run(std::uint32_t now_ms){
        for(std::size_t i = 0; i < count_; ++i){
            Task& task = tasks_[i];
            if(!is_due(now_ms, task.next_run_ms)){
                continue;
            }

            task.fn();
            ++task.runs;
            task.next_run_ms += task.period_ms;          // fixed timeline: no drift

            if(is_due(now_ms, task.next_run_ms)){        // already past the next deadline too
                const std::uint32_t late   = now_ms - task.next_run_ms;
                const std::uint32_t missed = late / task.period_ms + 1;
                task.overruns    += missed;
                task.next_run_ms += missed * task.period_ms;   // skip ahead, no catch-up burst
            }
        }
    }

    // Diagnostics for task i, in the order added. Out-of-range i returns 0.
    std::uint32_t runs(std::size_t i) const     { return i < count_ ? tasks_[i].runs : 0; }
    std::uint32_t overruns(std::size_t i) const { return i < count_ ? tasks_[i].overruns : 0; }
    std::size_t   task_count() const            { return count_; }
    static constexpr std::size_t capacity()     { return MaxTasks; }

private:
    struct Task {
        TaskFn        fn;
        std::uint32_t period_ms;
        std::uint32_t next_run_ms;
        std::uint32_t runs;
        std::uint32_t overruns;     // deadlines that passed without a run
    };

    // True once now has reached deadline. The unsigned difference is correct
    // across a counter wrap; reading it as signed makes "slightly in the past"
    // non-negative and "slightly in the future" negative. Valid while tasks
    // are never more than ~24.8 days (2^31 ms) late.
    static bool is_due(std::uint32_t now_ms, std::uint32_t deadline_ms){
        return static_cast<std::int32_t>(now_ms - deadline_ms) >= 0;
    }

    Task        tasks_[MaxTasks]{};
    std::size_t count_ = 0;
};
