// cmsdk_uart_regs.hpp - register layout of the Arm CMSDK APB UART (MPS2-AN385)
//
// Shared by the polled console driver (uart.cpp) and the interrupt-driven
// SerialPort (serial_port.cpp). Bit meanings from the CMSDK TRM, confirmed
// against QEMU's hw/char/cmsdk-apb-uart.c.
#pragma once

#include <cstddef>
#include <cstdint>

namespace cmsdk_uart {

struct Regs {
    volatile std::uint32_t DATA;      // 0x00 write: transmit a byte; read: received byte
    volatile std::uint32_t STATE;     // 0x04
    volatile std::uint32_t CTRL;      // 0x08
    volatile std::uint32_t INTSTATUS; // 0x0C write 1 to clear
    volatile std::uint32_t BAUDDIV;   // 0x10
};

static_assert(offsetof(Regs, DATA) == 0x00);
static_assert(offsetof(Regs, STATE) == 0x04);
static_assert(offsetof(Regs, CTRL) == 0x08);
static_assert(offsetof(Regs, INTSTATUS) == 0x0C);
static_assert(offsetof(Regs, BAUDDIV) == 0x10);

constexpr std::uint32_t STATE_TX_FULL  = 1u << 0;
constexpr std::uint32_t STATE_RX_FULL  = 1u << 1;

constexpr std::uint32_t CTRL_TX_EN     = 1u << 0;
constexpr std::uint32_t CTRL_RX_EN     = 1u << 1;
constexpr std::uint32_t CTRL_TX_INTEN  = 1u << 2;   // interrupt each time a byte finishes sending
constexpr std::uint32_t CTRL_RX_INTEN  = 1u << 3;   // interrupt each time a byte arrives

constexpr std::uint32_t INTSTATUS_TX   = 1u << 0;
constexpr std::uint32_t INTSTATUS_RX   = 1u << 1;

constexpr std::uint32_t BAUDDIV_MIN    = 16;

constexpr std::uintptr_t UART0_BASE    = 0x40004000;
constexpr std::uintptr_t UART1_BASE    = 0x40005000;

// IRQ numbers per QEMU hw/arm/mps2.c: UART n RX = 2n, TX = 2n + 1 (n = 0..2).
constexpr unsigned UART0_RX_IRQ = 0;
constexpr unsigned UART1_RX_IRQ = 2;
constexpr unsigned UART1_TX_IRQ = 3;

inline volatile Regs& at(std::uintptr_t base){
    return *reinterpret_cast<volatile Regs*>(base);
}

}  // namespace cmsdk_uart
