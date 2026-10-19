// ring_buffer.hpp - fixed-capacity single-producer/single-consumer FIFO
//
// No heap: storage is a plain array sized at compile time. Safe to share
// between exactly one producer and one consumer (e.g. a UART ISR and main())
// without disabling interrupts, because each index has a single writer:
// head_ is written only by push(), tail_ only by pop().
//
// One slot is always left empty so "full" and "empty" can be told apart,
// so a RingBuffer<T, N> holds at most N - 1 items.
#pragma once

#include <atomic>
#include <cstddef>

template <typename T, std::size_t N>
class RingBuffer {
    static_assert(N >= 2, "RingBuffer needs at least 2 slots (one is always kept empty)");
    static_assert(std::atomic<std::size_t>::is_always_lock_free,
                  "indices must be lock-free so an ISR can never block on them");

public:
    // Producer only. Returns false (and stores nothing) if the buffer is full.
    bool push(const T& item){
        const std::size_t head = head_.load(std::memory_order_relaxed);   // we own head_
        const std::size_t next = increment(head);
        if(next == tail_.load(std::memory_order_acquire)){
            return false;                                                  // full
        }
        data_[head] = item;
        head_.store(next, std::memory_order_release);                     // publish: item written first
        return true;
    }

    // Consumer only. Returns false if the buffer is empty; otherwise moves the
    // oldest item into 'out'.
    bool pop(T& out){
        const std::size_t tail = tail_.load(std::memory_order_relaxed);   // we own tail_
        if(tail == head_.load(std::memory_order_acquire)){
            return false;                                                  // empty
        }
        out = data_[tail];
        tail_.store(increment(tail), std::memory_order_release);          // free slot: item read first
        return true;
    }

    // Snapshots: exact when called by the owning side, possibly stale when
    // called while the other side is running.
    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    bool full() const {
        return increment(head_.load(std::memory_order_acquire)) == tail_.load(std::memory_order_acquire);
    }

    std::size_t size() const {
        const std::size_t head = head_.load(std::memory_order_acquire);
        const std::size_t tail = tail_.load(std::memory_order_acquire);
        return (head + N - tail) % N;
    }

    static constexpr std::size_t capacity(){ return N - 1; }

private:
    static constexpr std::size_t increment(std::size_t i){ return (i + 1) % N; }

    T data_[N]{};
    std::atomic<std::size_t> head_{0};   // next slot push() writes
    std::atomic<std::size_t> tail_{0};   // next slot pop() reads
};
