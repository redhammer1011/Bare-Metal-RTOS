// serial_port.cpp - interrupt-driven CMSDK UART driver

#include "serial_port.hpp"

#include "critical_section.hpp"
#include "nvic.hpp"

using namespace cmsdk_uart;

namespace {
    // constexpr constructor: built at compile time, no .init_array entry.
    SerialPort uart1_port{SerialPort::Config{UART1_BASE, UART1_RX_IRQ, UART1_TX_IRQ}};
}

SerialPort& link_port(){
    return uart1_port;
}

void SerialPort::start(std::uint8_t irq_priority, RxCallback on_rx){
    on_rx_ = on_rx;                              // set before the RX interrupt can fire

    regs().BAUDDIV = BAUDDIV_MIN;
    regs().CTRL = CTRL_TX_EN | CTRL_RX_EN | CTRL_TX_INTEN | CTRL_RX_INTEN;

    nvic_set_priority(config_.rx_irq, irq_priority);
    nvic_set_priority(config_.tx_irq, irq_priority);
    nvic_enable_irq(config_.rx_irq);
    nvic_enable_irq(config_.tx_irq);
}

bool SerialPort::read(std::uint8_t& byte){
    return rx_.pop(byte);
}

std::size_t SerialPort::write(const std::uint8_t* data, std::size_t len){
    std::size_t queued = 0;
    while(queued < len && tx_.push(data[queued])){
        ++queued;
    }
    start_tx_if_idle();                          // after queueing, so the ISR also sees these bytes
    return queued;
}

// The transmitter only interrupts when a byte *finishes*, so an idle
// transmitter must be handed its first byte here ("kick"); the TX interrupt
// sends the rest. Interrupts are masked so the ISR can't run between the
// busy check and the DATA write, and so only one side pops tx_ at a time.
void SerialPort::start_tx_if_idle(){
    CriticalSection cs;
    if(tx_busy_){
        return;                                  // a byte is in flight: its interrupt will continue the chain
    }
    std::uint8_t byte;
    if(!tx_.pop(byte)){
        return;
    }
    tx_busy_ = true;                             // BEFORE the DATA write: QEMU finishes the byte and raises
    regs().DATA = byte;                          // the TX interrupt inside this store (it runs once cs ends)
}

void SerialPort::handle_tx_interrupt(){
    regs().INTSTATUS = INTSTATUS_TX;             // W1C: clear first, as with RX
    std::uint8_t byte;
    if(tx_.pop(byte)){
        regs().DATA = byte;                      // next byte; its completion interrupts again
    } else {
        tx_busy_ = false;                        // nothing left: the next write() must kick
    }
}

void SerialPort::handle_rx_interrupt(){
    regs().INTSTATUS = INTSTATUS_RX;
    const auto byte = static_cast<std::uint8_t>(regs().DATA);
    if(!rx_.push(byte)){
        rx_dropped_ = rx_dropped_ + 1;
    }
    if(on_rx_ != nullptr){
        on_rx_();
    }
}

extern "C" void UART1RX_Handler(){ uart1_port.handle_rx_interrupt(); }
extern "C" void UART1TX_Handler(){ uart1_port.handle_tx_interrupt(); }
