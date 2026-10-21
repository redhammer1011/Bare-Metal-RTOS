// test_scheduler.cpp - unit tests for core/scheduler.hpp, driven by a fake clock

#include "scheduler.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace {
    // Task functions must be plain function pointers, so they record into
    // file-scope state that each test resets.
    int calls_a = 0;
    int calls_b = 0;
    char order[8] = {};
    int order_len = 0;

    void task_a(){ ++calls_a; if(order_len < 8){ order[order_len++] = 'a'; } }
    void task_b(){ ++calls_b; if(order_len < 8){ order[order_len++] = 'b'; } }

    class SchedulerTest : public ::testing::Test {
    protected:
        void SetUp() override {
            calls_a = 0;
            calls_b = 0;
            order_len = 0;
        }
    };
}

TEST_F(SchedulerTest, RunsTaskOncePerPeriod){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10));

    for(std::uint32_t t = 0; t <= 100; ++t){ s.run(t); }

    EXPECT_EQ(calls_a, 11);           // t = 0, 10, 20, ..., 100
    EXPECT_EQ(s.overruns(0), 0u);
}

TEST_F(SchedulerTest, TasksRunAtIndependentRates){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10));
    ASSERT_TRUE(s.add(task_b, 25));

    for(std::uint32_t t = 0; t <= 100; ++t){ s.run(t); }

    EXPECT_EQ(calls_a, 11);
    EXPECT_EQ(calls_b, 5);            // t = 0, 25, 50, 75, 100
}

TEST_F(SchedulerTest, OffsetDelaysFirstRun){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 100, /*now*/ 0, /*offset*/ 25));

    for(std::uint32_t t = 0; t < 25; ++t){ s.run(t); }
    EXPECT_EQ(calls_a, 0);

    for(std::uint32_t t = 25; t <= 300; ++t){ s.run(t); }
    EXPECT_EQ(calls_a, 3);            // t = 25, 125, 225
}

TEST_F(SchedulerTest, TasksDueTogetherRunInAddOrder){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_b, 10));
    ASSERT_TRUE(s.add(task_a, 10));

    s.run(0);

    ASSERT_EQ(order_len, 2);
    EXPECT_EQ(order[0], 'b');
    EXPECT_EQ(order[1], 'a');
}

TEST_F(SchedulerTest, AddRejectsInvalidTasksAndFullTable){
    Scheduler<2> s;

    EXPECT_FALSE(s.add(nullptr, 10));
    EXPECT_FALSE(s.add(task_a, 0));
    EXPECT_EQ(s.task_count(), 0u);

    EXPECT_TRUE(s.add(task_a, 10));
    EXPECT_TRUE(s.add(task_b, 10));
    EXPECT_FALSE(s.add(task_a, 10));
    EXPECT_EQ(s.task_count(), 2u);
}

TEST_F(SchedulerTest, LateRunDoesNotShiftLaterRuns){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10));

    s.run(0);                         // on time
    s.run(13);                        // deadline 10, 3 ms late
    EXPECT_EQ(calls_a, 2);

    s.run(19);                        // next deadline is still 20, not 23
    EXPECT_EQ(calls_a, 2);
    s.run(20);
    EXPECT_EQ(calls_a, 3);
    EXPECT_EQ(s.overruns(0), 0u);     // late, but no deadline was skipped
}

TEST_F(SchedulerTest, FallingBehindRunsOnceAndCountsMissedDeadlines){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10));

    s.run(0);
    s.run(55);                        // deadlines 10..50 have all passed

    EXPECT_EQ(calls_a, 2);            // one run, not a burst of five
    EXPECT_EQ(s.overruns(0), 4u);     // 5 deadlines passed, 1 served by this run

    s.run(59);
    EXPECT_EQ(calls_a, 2);
    s.run(60);                        // back on the original 10 ms grid
    EXPECT_EQ(calls_a, 3);
}

TEST_F(SchedulerTest, KeepsScheduleAcrossMillisWrap){
    const std::uint32_t start = 0xFFFFFF00u;   // 256 ms before the 32-bit wrap
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10, start));

    for(std::uint32_t i = 0; i < 1000; ++i){ s.run(start + i); }   // start + i wraps to 0 midway

    EXPECT_EQ(calls_a, 100);          // i = 0, 10, ..., 990
    EXPECT_EQ(s.overruns(0), 0u);
}

TEST_F(SchedulerTest, DiagnosticsForUnknownTaskAreZero){
    Scheduler<4> s;
    ASSERT_TRUE(s.add(task_a, 10));
    s.run(0);

    EXPECT_EQ(s.runs(0), 1u);
    EXPECT_EQ(s.runs(5), 0u);
    EXPECT_EQ(s.overruns(5), 0u);
}
