// packet.hpp - packet types and framing for the plant/firmware serial link
//
// Raw packet:  [type][seq][payload ...][crc_lo][crc_hi]
//              CRC-16/CCITT over type, seq and payload.
// On the wire: COBS(raw packet) followed by a 0x00 delimiter.
//
// Multi-byte fields are little-endian. Floats are IEEE-754 binary32. Payloads
// are written field by field, never by copying a struct's memory: struct
// padding and byte order are compiler- and CPU-specific, the wire format must
// not be. The Python side packs the same fields with struct.pack('<f...').
#pragma once

#include "cobs.hpp"
#include "crc16.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace proto {

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "wire format assumes IEEE-754 binary32 floats");

enum class PacketType : std::uint8_t {
    Sensor      = 1,   // plant -> firmware
    Actuator    = 2,   // firmware -> plant
    Telemetry   = 3,   // firmware -> host
    SetSetpoint = 4,   // host -> firmware
    SetGains    = 5,   // host -> firmware
};

constexpr std::size_t kHeaderSize  = 2;   // type + seq
constexpr std::size_t kCrcSize     = 2;
constexpr std::size_t kMaxPayload  = 32;
constexpr std::size_t kMaxRaw      = kHeaderSize + kMaxPayload + kCrcSize;
constexpr std::size_t kMaxEncoded  = cobs_max_encoded_size(kMaxRaw);
constexpr std::size_t kMaxFrame    = kMaxEncoded + 1;   // + 0x00 delimiter

// Payload length each packet type must have; 0 for an unknown type.
constexpr std::size_t payload_size(std::uint8_t type){
    switch(static_cast<PacketType>(type)){
        case PacketType::Sensor:      return 12;
        case PacketType::Actuator:    return 8;
        case PacketType::Telemetry:   return 16;
        case PacketType::SetSetpoint: return 4;
        case PacketType::SetGains:    return 12;
    }
    return 0;
}

// ------------------------------------------------------------------ payloads
struct SensorData    { float gyro_rate_dps; float accel_x_g; float accel_z_g; };
struct ActuatorData  { float motor1; float motor2; };
struct TelemetryData { float setpoint_deg; float estimate_deg; float output; std::uint32_t loop_time_us; };
struct SetpointData  { float setpoint_deg; };
struct GainsData     { float kp; float ki; float kd; };

// --------------------------------------------------- little-endian accessors
inline void put_u32(std::uint8_t* p, std::uint32_t v){
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

inline std::uint32_t get_u32(const std::uint8_t* p){
    return static_cast<std::uint32_t>(p[0])
         | static_cast<std::uint32_t>(p[1]) << 8
         | static_cast<std::uint32_t>(p[2]) << 16
         | static_cast<std::uint32_t>(p[3]) << 24;
}

// memcpy is the well-defined way to reinterpret a float's bits as an integer;
// the compiler turns a 4-byte memcpy into a single register move.
inline void put_f32(std::uint8_t* p, float f){
    std::uint32_t bits;
    std::memcpy(&bits, &f, sizeof bits);
    put_u32(p, bits);
}

inline float get_f32(const std::uint8_t* p){
    const std::uint32_t bits = get_u32(p);
    float f;
    std::memcpy(&f, &bits, sizeof f);
    return f;
}

// -------------------------------------------------- per-type (de)serializers
// encode_payload writes the payload and returns its length; type_of names the
// packet type for a payload struct; decode_payload reads it back.
inline PacketType type_of(const SensorData&)    { return PacketType::Sensor; }
inline PacketType type_of(const ActuatorData&)  { return PacketType::Actuator; }
inline PacketType type_of(const TelemetryData&) { return PacketType::Telemetry; }
inline PacketType type_of(const SetpointData&)  { return PacketType::SetSetpoint; }
inline PacketType type_of(const GainsData&)     { return PacketType::SetGains; }

inline std::size_t encode_payload(const SensorData& d, std::uint8_t* p){
    put_f32(p + 0, d.gyro_rate_dps); put_f32(p + 4, d.accel_x_g); put_f32(p + 8, d.accel_z_g);
    return 12;
}
inline std::size_t encode_payload(const ActuatorData& d, std::uint8_t* p){
    put_f32(p + 0, d.motor1); put_f32(p + 4, d.motor2);
    return 8;
}
inline std::size_t encode_payload(const TelemetryData& d, std::uint8_t* p){
    put_f32(p + 0, d.setpoint_deg); put_f32(p + 4, d.estimate_deg); put_f32(p + 8, d.output);
    put_u32(p + 12, d.loop_time_us);
    return 16;
}
inline std::size_t encode_payload(const SetpointData& d, std::uint8_t* p){
    put_f32(p + 0, d.setpoint_deg);
    return 4;
}
inline std::size_t encode_payload(const GainsData& d, std::uint8_t* p){
    put_f32(p + 0, d.kp); put_f32(p + 4, d.ki); put_f32(p + 8, d.kd);
    return 12;
}

// A packet that passed every check in FrameParser.
struct Packet {
    PacketType   type;
    std::uint8_t seq;
    std::uint8_t payload[kMaxPayload];
    std::size_t  payload_len;
};

// Each returns false (leaving d untouched) if the packet is a different type.
inline bool decode_payload(const Packet& pk, SensorData& d){
    if(pk.type != PacketType::Sensor) return false;
    d = {get_f32(pk.payload + 0), get_f32(pk.payload + 4), get_f32(pk.payload + 8)};
    return true;
}
inline bool decode_payload(const Packet& pk, ActuatorData& d){
    if(pk.type != PacketType::Actuator) return false;
    d = {get_f32(pk.payload + 0), get_f32(pk.payload + 4)};
    return true;
}
inline bool decode_payload(const Packet& pk, TelemetryData& d){
    if(pk.type != PacketType::Telemetry) return false;
    d = {get_f32(pk.payload + 0), get_f32(pk.payload + 4), get_f32(pk.payload + 8), get_u32(pk.payload + 12)};
    return true;
}
inline bool decode_payload(const Packet& pk, SetpointData& d){
    if(pk.type != PacketType::SetSetpoint) return false;
    d = {get_f32(pk.payload + 0)};
    return true;
}
inline bool decode_payload(const Packet& pk, GainsData& d){
    if(pk.type != PacketType::SetGains) return false;
    d = {get_f32(pk.payload + 0), get_f32(pk.payload + 4), get_f32(pk.payload + 8)};
    return true;
}

// ------------------------------------------------------------- frame builder
// Builds a complete wire frame (COBS-encoded, 0x00-terminated) into frame.
// Returns its length, or 0 if the type is unknown, payload_len doesn't match
// the type, or frame_cap is too small.
inline std::size_t build_frame(PacketType type, std::uint8_t seq,
                               const std::uint8_t* payload, std::size_t payload_len,
                               std::uint8_t* frame, std::size_t frame_cap){
    const std::uint8_t type_byte = static_cast<std::uint8_t>(type);
    if(payload_size(type_byte) == 0 || payload_len != payload_size(type_byte)){
        return 0;
    }

    std::uint8_t raw[kMaxRaw];
    raw[0] = type_byte;
    raw[1] = seq;
    std::memcpy(raw + kHeaderSize, payload, payload_len);
    const std::size_t crc_offset = kHeaderSize + payload_len;
    const std::uint16_t crc = crc16_ccitt(raw, crc_offset);
    raw[crc_offset]     = static_cast<std::uint8_t>(crc);
    raw[crc_offset + 1] = static_cast<std::uint8_t>(crc >> 8);
    const std::size_t raw_len = crc_offset + kCrcSize;

    if(frame_cap < cobs_max_encoded_size(raw_len) + 1){
        return 0;
    }
    const std::size_t encoded_len = cobs_encode(raw, raw_len, frame, frame_cap - 1);
    frame[encoded_len] = 0x00;                                   // delimiter
    return encoded_len + 1;
}

// Typed convenience: build_frame(seq, SensorData{...}, frame, cap).
template <typename Payload>
std::size_t build_frame(std::uint8_t seq, const Payload& data,
                        std::uint8_t* frame, std::size_t frame_cap){
    std::uint8_t payload[kMaxPayload];
    const std::size_t len = encode_payload(data, payload);
    return build_frame(type_of(data), seq, payload, len, frame, frame_cap);
}

}  // namespace proto
