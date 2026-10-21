// cobs.hpp - Consistent Overhead Byte Stuffing
//
// Encoding removes every 0x00 from a buffer so 0x00 can mark the end of a
// frame on a byte stream. Each run of non-zero bytes is prefixed by a code
// byte: code N means "copy the next N-1 bytes, then a 0x00 was here", except
// code 0xFF, which means "254 bytes follow and no 0x00". The zero implied by
// the last code of a frame is not part of the data.
//
// Overhead is at most one byte per 254, so buffer sizes are known up front.
// The caller appends the 0x00 delimiter after encoding and strips it before
// decoding.
#pragma once

#include <cstddef>
#include <cstdint>

// Worst-case encoded size of n input bytes (delimiter not included).
constexpr std::size_t cobs_max_encoded_size(std::size_t n){
    return n + n / 254 + 1;
}

// Encode src[0, len) into dst. Returns the encoded length, or 0 if dst_cap is
// smaller than cobs_max_encoded_size(len) (a valid encoding is never empty).
inline std::size_t cobs_encode(const std::uint8_t* src, std::size_t len,
                               std::uint8_t* dst, std::size_t dst_cap){
    if(dst_cap < cobs_max_encoded_size(len)){
        return 0;
    }

    std::size_t code_index = 0;   // where the current run's code byte will go
    std::size_t out = 1;          // next free byte in dst (slot 0 is the first code)
    std::uint8_t code = 1;        // 1 + number of data bytes in the current run

    for(std::size_t i = 0; i < len; ++i){
        if(src[i] == 0){
            dst[code_index] = code;          // close the run: the zero is implied by its code
            code_index = out++;
            code = 1;
        } else {
            dst[out++] = src[i];
            ++code;
            if(code == 0xFF){                // 254 data bytes: maximum run, no implied zero
                dst[code_index] = code;
                code_index = out++;
                code = 1;
            }
        }
    }
    dst[code_index] = code;                  // close the final run
    return out;
}

// Decode src[0, len) (delimiter already stripped) into dst. Returns false if
// the input is not valid COBS (contains 0x00, or a code runs past the end) or
// if the result would not fit in dst_cap; dst contents are then unspecified.
inline bool cobs_decode(const std::uint8_t* src, std::size_t len,
                        std::uint8_t* dst, std::size_t dst_cap, std::size_t& out_len){
    std::size_t in = 0;
    std::size_t out = 0;

    while(in < len){
        const std::uint8_t code = src[in++];
        if(code == 0){
            return false;                    // 0x00 can't appear inside an encoded frame
        }

        const std::size_t run = code - 1u;
        if(run > len - in || run > dst_cap - out){
            return false;                    // code points past the input, or output full
        }
        for(std::size_t k = 0; k < run; ++k){
            if(src[in] == 0){
                return false;
            }
            dst[out++] = src[in++];
        }

        if(code != 0xFF && in < len){        // implied zero, except after the final run
            if(out == dst_cap){
                return false;
            }
            dst[out++] = 0;
        }
    }

    out_len = out;
    return true;
}
