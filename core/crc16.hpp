// crc16.hpp - CRC-16/CCITT-FALSE
//
// Polynomial 0x1021 (x^16 + x^12 + x^5 + 1), initial value 0xFFFF, no bit
// reflection, no final XOR. Check value: CRC of ASCII "123456789" = 0x29B1.
// Detects every single-bit, double-bit and odd-count bit error, and every
// burst of up to 16 bits, in frames of our size.
//
// Bitwise rather than table-driven: 8 shift/XOR steps per byte, no 512-byte
// lookup table in flash. Fast enough for short packets at a few hundred Hz.
#pragma once

#include <cstddef>
#include <cstdint>

inline std::uint16_t crc16_ccitt(const std::uint8_t* data, std::size_t len,
                                 std::uint16_t crc = 0xFFFF){
    for(std::size_t i = 0; i < len; ++i){
        crc = static_cast<std::uint16_t>(crc ^ (static_cast<std::uint16_t>(data[i]) << 8));
        for(int bit = 0; bit < 8; ++bit){
            if(crc & 0x8000u){
                crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021u);
            } else {
                crc = static_cast<std::uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}
