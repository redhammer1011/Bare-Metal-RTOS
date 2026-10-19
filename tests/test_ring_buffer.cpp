// test_ring_buffer.cpp - unit tests for core/ring_buffer.hpp (runs on the laptop)

#include "ring_buffer.hpp"

#include <gtest/gtest.h>

TEST(RingBuffer, StartsEmpty){
    RingBuffer<int, 8> rb;

    EXPECT_TRUE(rb.empty());
    EXPECT_FALSE(rb.full());
    EXPECT_EQ(rb.size(), 0u);
}

TEST(RingBuffer, PopFromEmptyFailsAndLeavesOutputUntouched){
    RingBuffer<int, 8> rb;
    int out = 42;

    EXPECT_FALSE(rb.pop(out));
    EXPECT_EQ(out, 42);
}

TEST(RingBuffer, CapacityIsOneLessThanSlotCount){
    RingBuffer<int, 8> rb;
    static_assert(RingBuffer<int, 8>::capacity() == 7);

    for(int i = 0; i < 7; ++i){
        EXPECT_TRUE(rb.push(i)) << "push #" << i << " should fit";
    }
    EXPECT_TRUE(rb.full());
    EXPECT_EQ(rb.size(), 7u);
}

TEST(RingBuffer, PushWhenFullFailsAndKeepsContents){
    RingBuffer<int, 4> rb;
    ASSERT_TRUE(rb.push(1));
    ASSERT_TRUE(rb.push(2));
    ASSERT_TRUE(rb.push(3));

    EXPECT_FALSE(rb.push(99));

    int v = 0;
    ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 1);
    ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 2);
    ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 3);
    EXPECT_FALSE(rb.pop(v));                   // 99 was never stored
}

TEST(RingBuffer, PopsInFifoOrder){
    RingBuffer<int, 8> rb;
    for(int i = 1; i <= 5; ++i){
        ASSERT_TRUE(rb.push(i));
    }

    for(int expected = 1; expected <= 5; ++expected){
        int v = 0;
        ASSERT_TRUE(rb.pop(v));
        EXPECT_EQ(v, expected);
    }
}

TEST(RingBuffer, FifoOrderHoldsAcrossWrapAround){
    RingBuffer<int, 4> rb;   // indices wrap after slot 3
    int next_in = 0;
    int next_out = 0;

    // Many more items than slots, pushed and popped in uneven bursts, so head
    // and tail each wrap several times at different moments.
    for(int round = 0; round < 10; ++round){
        // Bounded so a broken full check fails the test instead of hanging it.
        for(int attempt = 0; attempt < 4 && rb.push(next_in); ++attempt){
            ++next_in;
        }
        ASSERT_TRUE(rb.full()) << "round " << round;
        const int to_pop = (round % 2 == 0) ? 2 : 3;
        for(int i = 0; i < to_pop; ++i){
            int v = -1;
            ASSERT_TRUE(rb.pop(v));
            EXPECT_EQ(v, next_out) << "round " << round;
            ++next_out;
        }
    }
    EXPECT_GT(next_in, 12);  // sanity: we really did go around the ring
}

TEST(RingBuffer, ReusableAfterDraining){
    RingBuffer<int, 4> rb;
    int v = 0;

    for(int cycle = 0; cycle < 3; ++cycle){
        ASSERT_TRUE(rb.push(10 * cycle + 1));
        ASSERT_TRUE(rb.push(10 * cycle + 2));
        ASSERT_TRUE(rb.push(10 * cycle + 3));
        ASSERT_TRUE(rb.full());

        ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 10 * cycle + 1);
        ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 10 * cycle + 2);
        ASSERT_TRUE(rb.pop(v));  EXPECT_EQ(v, 10 * cycle + 3);
        EXPECT_TRUE(rb.empty());
        EXPECT_FALSE(rb.pop(v));
    }
}

TEST(RingBuffer, SizeIsCorrectWhenHeadHasWrappedButTailHasNot){
    RingBuffer<int, 8> rb;
    int v = 0;

    for(int i = 0; i < 6; ++i){ ASSERT_TRUE(rb.push(i)); }   // head = 6
    for(int i = 0; i < 5; ++i){ ASSERT_TRUE(rb.pop(v)); }    // tail = 5
    for(int i = 0; i < 4; ++i){ ASSERT_TRUE(rb.push(i)); }   // head = 10 % 8 = 2

    // head (2) < tail (5): naive head - tail would underflow.
    EXPECT_EQ(rb.size(), 5u);
    EXPECT_FALSE(rb.empty());
    EXPECT_FALSE(rb.full());
}

// With a power-of-two N, unsigned wraparound makes (head - tail) % N happen to
// give the right answer, hiding a missing "+ N". A non-power-of-two N exposes it.
TEST(RingBuffer, SizeIsCorrectForNonPowerOfTwoSlotCount){
    RingBuffer<int, 5> rb;
    int v = 0;

    for(int i = 0; i < 4; ++i){ ASSERT_TRUE(rb.push(i)); }   // head = 4
    for(int i = 0; i < 3; ++i){ ASSERT_TRUE(rb.pop(v)); }    // tail = 3
    for(int i = 0; i < 3; ++i){ ASSERT_TRUE(rb.push(i)); }   // head = 7 % 5 = 2

    EXPECT_EQ(rb.size(), 4u);
    EXPECT_TRUE(rb.full());
}

TEST(RingBuffer, WorksWithStructType){
    struct Sample {
        int   id;
        float value;
    };
    RingBuffer<Sample, 4> rb;

    ASSERT_TRUE(rb.push(Sample{7, 1.5f}));
    ASSERT_TRUE(rb.push(Sample{8, -2.25f}));

    Sample s{};
    ASSERT_TRUE(rb.pop(s));
    EXPECT_EQ(s.id, 7);
    EXPECT_FLOAT_EQ(s.value, 1.5f);
    ASSERT_TRUE(rb.pop(s));
    EXPECT_EQ(s.id, 8);
    EXPECT_FLOAT_EQ(s.value, -2.25f);
}

TEST(RingBuffer, SmallestBufferHoldsOneItem){
    RingBuffer<char, 2> rb;
    char c = 0;

    EXPECT_TRUE(rb.push('a'));
    EXPECT_FALSE(rb.push('b'));
    ASSERT_TRUE(rb.pop(c));
    EXPECT_EQ(c, 'a');
    EXPECT_TRUE(rb.push('b'));   // slot reused after wrap
    ASSERT_TRUE(rb.pop(c));
    EXPECT_EQ(c, 'b');
}
