// frame_parser.hpp - byte-at-a-time receiver for COBS/CRC-16 frames
//
// Feed it every byte from the link, in order. It collects bytes up to each
// 0x00 delimiter, then checks the frame: COBS valid, long enough, CRC matches,
// known type, payload length right for that type. Only a frame that passes
// every check is delivered; each kind of rejection has its own counter.
//
// Recovery is automatic: whatever goes wrong, the next 0x00 starts a fresh
// frame. A frame that grows past the largest possible encoding means a
// delimiter was lost, so the rest of it is discarded up to the next 0x00.
//
// No heap, fixed buffers, bounded work per byte (a full check runs only on
// the delimiter byte). Not thread-safe: use from one task.
#pragma once

#include "cobs.hpp"
#include "crc16.hpp"
#include "packet.hpp"

#include <cstddef>
#include <cstdint>

namespace proto {

class FrameParser {
public:
    struct Stats {
        std::uint32_t frames_ok;
        std::uint32_t cobs_errors;     // not valid COBS
        std::uint32_t length_errors;   // too short, or wrong payload length for its type
        std::uint32_t crc_errors;
        std::uint32_t type_errors;     // unknown packet type
        std::uint32_t overflows;       // frame longer than any valid frame (lost delimiter)
    };

    // Returns true when this byte completed a valid packet, now in packet().
    bool feed(std::uint8_t byte){
        if(byte == 0x00){
            if(state_ == State::Discarding){
                state_ = State::Collecting;          // resynchronized
                len_ = 0;
                return false;
            }
            const bool ok = (len_ > 0) && finish_frame();   // empty frame: just a spare delimiter
            len_ = 0;
            return ok;
        }

        if(state_ == State::Discarding){
            return false;
        }
        if(len_ == sizeof buf_){
            ++stats_.overflows;
            state_ = State::Discarding;
            len_ = 0;
            return false;
        }
        buf_[len_++] = byte;
        return false;
    }

    const Packet& packet() const { return packet_; }
    const Stats&  stats()  const { return stats_; }

private:
    enum class State { Collecting, Discarding };

    bool finish_frame(){
        std::uint8_t raw[kMaxRaw];
        std::size_t raw_len = 0;
        if(!cobs_decode(buf_, len_, raw, sizeof raw, raw_len)){
            ++stats_.cobs_errors;
            return false;
        }
        if(raw_len < kHeaderSize + kCrcSize){
            ++stats_.length_errors;
            return false;
        }

        const std::size_t crc_offset = raw_len - kCrcSize;
        const std::uint16_t received = static_cast<std::uint16_t>(raw[crc_offset] | (raw[crc_offset + 1] << 8));
        if(crc16_ccitt(raw, crc_offset) != received){
            ++stats_.crc_errors;
            return false;
        }

        const std::size_t expected = payload_size(raw[0]);
        if(expected == 0){
            ++stats_.type_errors;
            return false;
        }
        const std::size_t payload_len = crc_offset - kHeaderSize;
        if(payload_len != expected){
            ++stats_.length_errors;
            return false;
        }

        packet_.type = static_cast<PacketType>(raw[0]);
        packet_.seq = raw[1];
        for(std::size_t i = 0; i < payload_len; ++i){
            packet_.payload[i] = raw[kHeaderSize + i];
        }
        packet_.payload_len = payload_len;
        ++stats_.frames_ok;
        return true;
    }

    State        state_ = State::Collecting;
    std::uint8_t buf_[kMaxEncoded]{};   // encoded bytes of the current frame, delimiter excluded
    std::size_t  len_ = 0;
    Packet       packet_{};
    Stats        stats_{};
};

}  // namespace proto
