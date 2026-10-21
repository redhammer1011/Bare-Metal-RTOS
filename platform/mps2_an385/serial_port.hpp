// serial_port.hpp - interrupt-driven CMSDK UART driver, one object per UART
//
// Both directions are buffered so no caller ever busy-waits on the hardware:
//   RX: the RX interrupt pushes each byte into rx_; read() pops.
//   TX: write() pushes into tx_ and starts the transmitter if idle; the TX
//       interrupt (fired as each byte finishes) sends the next byte.
//
// The ring buffers are single-producer / single-consumer, so each SerialPort
// supports exactly one reading task and one writing task (they may be the
// same task). The objects are constant-initialized, so they are valid before
// any constructor runs, but start() must be called from a task: the RX
// callback may wake tasks.
#pragma once

#include "cmsdk_uart_regs.hpp"
#include "ring_buffer.hpp"

#include <cstddef>
#include <cstdint>

class SerialPort {
public:
    struct Config {
        std::uintptr_t base;    // register block address
        unsigned       rx_irq;
        unsigned       tx_irq;
    };

    // Called from the RX interrupt after each byte is queued. Interrupt
    // context: keep it short, use only ...FromISR() kernel calls.
    using RxCallback = void (*)();

    constexpr explicit SerialPort(Config config) : config_(config) {}

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Configure the UART and enable its RX and TX interrupts at irq_priority
    // (full 8-bit NVIC value; >= configMAX_SYSCALL_INTERRUPT_PRIORITY if the
    // callback calls FreeRTOS).
    void start(std::uint8_t irq_priority, RxCallback on_rx);

    // Consumer side of RX. Returns false if no byte is waiting.
    bool read(std::uint8_t& byte);

    // Producer side of TX. Queues as many bytes as fit and returns how many;
    // the caller retries the rest later. Never blocks.
    std::size_t write(const std::uint8_t* data, std::size_t len);

    std::uint32_t rx_dropped() const { return rx_dropped_; }   // bytes lost to a full rx_

    // Called by this UART's interrupt handlers.
    void handle_rx_interrupt();
    void handle_tx_interrupt();

private:
    volatile cmsdk_uart::Regs& regs() const { return cmsdk_uart::at(config_.base); }
    void start_tx_if_idle();

    Config config_;
    RingBuffer<std::uint8_t, 128> rx_{};
    RingBuffer<std::uint8_t, 128> tx_{};
    volatile bool          tx_busy_ = false;     // a byte is in the transmitter; its TX interrupt will follow
    volatile std::uint32_t rx_dropped_ = 0;
    RxCallback             on_rx_ = nullptr;
};

// UART1: binary packet link to the plant simulation (QEMU's second -serial).
SerialPort& link_port();
