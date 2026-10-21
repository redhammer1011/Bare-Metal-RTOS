// test_protocol.cpp - COBS, CRC-16, packet framing and the frame parser

#include "cobs.hpp"
#include "crc16.hpp"
#include "frame_parser.hpp"
#include "packet.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

using Bytes = std::vector<std::uint8_t>;

namespace {
    Bytes encode(const Bytes& in){
        Bytes out(cobs_max_encoded_size(in.size()));
        const std::size_t n = cobs_encode(in.data(), in.size(), out.data(), out.size());
        out.resize(n);
        return out;
    }

    bool decode(const Bytes& in, Bytes& out){
        out.assign(in.size() + 1, 0);
        std::size_t n = 0;
        if(!cobs_decode(in.data(), in.size(), out.data(), out.size(), n)){
            return false;
        }
        out.resize(n);
        return true;
    }

    Bytes frame_for(std::uint8_t seq, const proto::SensorData& d){
        Bytes f(proto::kMaxFrame);
        f.resize(proto::build_frame(seq, d, f.data(), f.size()));
        return f;
    }

    // Feeds bytes, collecting every delivered packet.
    std::vector<proto::Packet> feed_all(proto::FrameParser& p, const Bytes& bytes){
        std::vector<proto::Packet> got;
        for(std::uint8_t b : bytes){
            if(p.feed(b)){
                got.push_back(p.packet());
            }
        }
        return got;
    }

    // Sensor values derived from seq, so any delivered packet can be checked.
    proto::SensorData sensor_for(std::uint32_t i){
        return {static_cast<float>(i) * 0.5f, static_cast<float>(i % 7) - 3.0f, 1.0f / static_cast<float>(i + 1)};
    }
}

// =================================================================== CRC-16

TEST(Crc16, MatchesPublishedCheckValue){
    const char* s = "123456789";
    EXPECT_EQ(crc16_ccitt(reinterpret_cast<const std::uint8_t*>(s), 9), 0x29B1);
}

TEST(Crc16, EmptyInputIsInitialValue){
    EXPECT_EQ(crc16_ccitt(nullptr, 0), 0xFFFF);
}

TEST(Crc16, DetectsEverySingleBitFlip){
    Bytes data = {0x01, 0x10, 0x7F, 0x80, 0xFF, 0x00, 0x42, 0x99};
    const std::uint16_t good = crc16_ccitt(data.data(), data.size());
    for(std::size_t byte = 0; byte < data.size(); ++byte){
        for(int bit = 0; bit < 8; ++bit){
            data[byte] ^= static_cast<std::uint8_t>(1u << bit);
            EXPECT_NE(crc16_ccitt(data.data(), data.size()), good) << "byte " << byte << " bit " << bit;
            data[byte] ^= static_cast<std::uint8_t>(1u << bit);
        }
    }
}

TEST(Crc16, DetectsSwappedBytesThatASumWouldMiss){
    const Bytes a = {0x01, 0x02, 0x03};
    const Bytes b = {0x02, 0x01, 0x03};   // same byte sum
    EXPECT_NE(crc16_ccitt(a.data(), a.size()), crc16_ccitt(b.data(), b.size()));
}

// ===================================================================== COBS

TEST(Cobs, EncodesKnownVectors){
    EXPECT_EQ(encode({}),                       (Bytes{0x01}));
    EXPECT_EQ(encode({0x00}),                   (Bytes{0x01, 0x01}));
    EXPECT_EQ(encode({0x00, 0x00}),             (Bytes{0x01, 0x01, 0x01}));
    EXPECT_EQ(encode({0x11, 0x22, 0x00, 0x33}), (Bytes{0x03, 0x11, 0x22, 0x02, 0x33}));
    EXPECT_EQ(encode({0x11, 0x00, 0x00, 0x22}), (Bytes{0x02, 0x11, 0x01, 0x02, 0x22}));
    EXPECT_EQ(encode({0x11, 0x22, 0x33}),       (Bytes{0x04, 0x11, 0x22, 0x33}));
}

TEST(Cobs, HandlesMaximumLengthRuns){
    Bytes run254(254, 0xAB);
    Bytes enc = encode(run254);
    ASSERT_EQ(enc.size(), 256u);              // FF + 254 data + final empty run
    EXPECT_EQ(enc.front(), 0xFF);
    EXPECT_EQ(enc.back(), 0x01);

    Bytes run255(255, 0xAB);
    enc = encode(run255);
    ASSERT_EQ(enc.size(), 257u);              // FF + 254 data, then 02 + 1 data
    EXPECT_EQ(enc[255], 0x02);

    Bytes out;
    ASSERT_TRUE(decode(encode(run254), out));
    EXPECT_EQ(out, run254);
    ASSERT_TRUE(decode(encode(run255), out));
    EXPECT_EQ(out, run255);
}

TEST(Cobs, RoundTripsRandomDataAndNeverEmitsZero){
    std::mt19937 rng(12345);
    for(int trial = 0; trial < 2000; ++trial){
        Bytes data(rng() % 600);
        const bool zero_heavy = trial % 3 == 0;
        for(auto& b : data){
            b = zero_heavy ? static_cast<std::uint8_t>(rng() % 4 == 0 ? 0 : rng()) : static_cast<std::uint8_t>(rng());
        }
        const Bytes enc = encode(data);
        ASSERT_LE(enc.size(), cobs_max_encoded_size(data.size()));
        for(std::uint8_t b : enc){
            ASSERT_NE(b, 0x00) << "trial " << trial;
        }
        Bytes out;
        ASSERT_TRUE(decode(enc, out)) << "trial " << trial;
        ASSERT_EQ(out, data) << "trial " << trial;
    }
}

TEST(Cobs, EncodeRejectsTooSmallBuffer){
    const Bytes data = {1, 2, 3};
    std::uint8_t out[3];
    EXPECT_EQ(cobs_encode(data.data(), data.size(), out, sizeof out), 0u);
}

TEST(Cobs, DecodeRejectsInvalidInput){
    Bytes out;
    EXPECT_FALSE(decode({0x03, 0x11, 0x00}, out));     // zero inside the frame
    EXPECT_FALSE(decode({0x00, 0x11}, out));           // zero code
    EXPECT_FALSE(decode({0x05, 0x11, 0x22}, out));     // code runs past the end

    std::uint8_t small[2];
    std::size_t n = 0;
    const Bytes enc = encode({1, 2, 3});
    EXPECT_FALSE(cobs_decode(enc.data(), enc.size(), small, sizeof small, n));   // output too small
}

// ================================================================== packets

TEST(Packet, LittleEndianAccessorsRoundTrip){
    std::uint8_t buf[4];
    proto::put_u32(buf, 0x12345678u);
    EXPECT_EQ(buf[0], 0x78);                  // least significant byte first
    EXPECT_EQ(buf[3], 0x12);
    EXPECT_EQ(proto::get_u32(buf), 0x12345678u);

    proto::put_f32(buf, 1.0f);                // IEEE-754: 1.0f = 0x3F800000
    EXPECT_EQ(proto::get_u32(buf), 0x3F800000u);
    EXPECT_FLOAT_EQ(proto::get_f32(buf), 1.0f);
}

TEST(Packet, FrameEndsWithSingleDelimiter){
    const Bytes f = frame_for(0, {0.0f, 0.0f, 0.0f});   // payload full of zero bytes
    ASSERT_FALSE(f.empty());
    EXPECT_EQ(f.back(), 0x00);
    for(std::size_t i = 0; i + 1 < f.size(); ++i){
        EXPECT_NE(f[i], 0x00) << "index " << i;
    }
    EXPECT_LE(f.size(), proto::kMaxFrame);
}

TEST(Packet, BuildRejectsWrongPayloadLength){
    std::uint8_t payload[4] = {};
    std::uint8_t frame[proto::kMaxFrame];
    EXPECT_EQ(proto::build_frame(proto::PacketType::Sensor, 0, payload, 4, frame, sizeof frame), 0u);
    EXPECT_EQ(proto::build_frame(static_cast<proto::PacketType>(99), 0, payload, 4, frame, sizeof frame), 0u);
}

TEST(Packet, EveryTypeRoundTripsThroughTheParser){
    proto::FrameParser p;
    std::uint8_t f[proto::kMaxFrame];
    std::size_t n;

    n = proto::build_frame(1, proto::SensorData{12.5f, -0.25f, 0.97f}, f, sizeof f);
    ASSERT_EQ(feed_all(p, Bytes(f, f + n)).size(), 1u);
    proto::SensorData s{};
    ASSERT_TRUE(proto::decode_payload(p.packet(), s));
    EXPECT_EQ(p.packet().seq, 1);
    EXPECT_FLOAT_EQ(s.gyro_rate_dps, 12.5f);
    EXPECT_FLOAT_EQ(s.accel_x_g, -0.25f);
    EXPECT_FLOAT_EQ(s.accel_z_g, 0.97f);

    n = proto::build_frame(2, proto::ActuatorData{0.4f, 0.6f}, f, sizeof f);
    ASSERT_EQ(feed_all(p, Bytes(f, f + n)).size(), 1u);
    proto::ActuatorData a{};
    ASSERT_TRUE(proto::decode_payload(p.packet(), a));
    EXPECT_FLOAT_EQ(a.motor2, 0.6f);
    EXPECT_FALSE(proto::decode_payload(p.packet(), s));   // wrong type: refused

    n = proto::build_frame(3, proto::TelemetryData{10.0f, 9.5f, -0.1f, 1234u}, f, sizeof f);
    ASSERT_EQ(feed_all(p, Bytes(f, f + n)).size(), 1u);
    proto::TelemetryData t{};
    ASSERT_TRUE(proto::decode_payload(p.packet(), t));
    EXPECT_EQ(t.loop_time_us, 1234u);

    n = proto::build_frame(4, proto::SetpointData{-15.0f}, f, sizeof f);
    ASSERT_EQ(feed_all(p, Bytes(f, f + n)).size(), 1u);
    proto::SetpointData sp{};
    ASSERT_TRUE(proto::decode_payload(p.packet(), sp));
    EXPECT_FLOAT_EQ(sp.setpoint_deg, -15.0f);

    n = proto::build_frame(5, proto::GainsData{1.5f, 0.2f, 0.05f}, f, sizeof f);
    ASSERT_EQ(feed_all(p, Bytes(f, f + n)).size(), 1u);
    proto::GainsData g{};
    ASSERT_TRUE(proto::decode_payload(p.packet(), g));
    EXPECT_FLOAT_EQ(g.kd, 0.05f);

    EXPECT_EQ(p.stats().frames_ok, 5u);
}

// Exact wire bytes, also asserted by sim/test_protocol.py: pins the format so
// the C++ and Python implementations can't drift apart while each still
// agrees with itself (a consistently wrong CRC passes every round-trip test).
TEST(Packet, MatchesGoldenWireBytesSharedWithPython){
    std::uint8_t f[proto::kMaxFrame];
    std::size_t n = proto::build_frame(1, proto::SensorData{12.5f, -0.25f, 0.97f}, f, sizeof f);
    EXPECT_EQ(Bytes(f, f + n), (Bytes{0x03, 0x01, 0x01, 0x01, 0x03, 0x48, 0x41, 0x01, 0x09, 0x80, 0xbe,
                                      0xec, 0x51, 0x78, 0x3f, 0x1f, 0xca, 0x00}));
    n = proto::build_frame(2, proto::ActuatorData{0.4f, 0.6f}, f, sizeof f);
    EXPECT_EQ(Bytes(f, f + n), (Bytes{0x0d, 0x02, 0x02, 0xcd, 0xcc, 0xcc, 0x3e, 0x9a, 0x99, 0x19, 0x3f,
                                      0xc1, 0xc0, 0x00}));
    n = proto::build_frame(255, proto::TelemetryData{10.0f, 9.5f, -0.1f, 1234u}, f, sizeof f);
    EXPECT_EQ(Bytes(f, f + n), (Bytes{0x03, 0x03, 0xff, 0x01, 0x03, 0x20, 0x41, 0x01, 0x09, 0x18, 0x41,
                                      0xcd, 0xcc, 0xcc, 0xbd, 0xd2, 0x04, 0x01, 0x03, 0x44, 0xa8, 0x00}));
    n = proto::build_frame(0, proto::SensorData{0.0f, 0.0f, 0.0f}, f, sizeof f);
    EXPECT_EQ(Bytes(f, f + n), (Bytes{0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                      0x01, 0x01, 0x01, 0x03, 0x0b, 0xd2, 0x00}));
}

// ============================================================ frame parser

TEST(FrameParser, ParsesBackToBackFramesAndIgnoresSpareDelimiters){
    proto::FrameParser p;
    Bytes stream = {0x00, 0x00};                       // spare delimiters: harmless
    for(std::uint8_t seq = 0; seq < 5; ++seq){
        const Bytes f = frame_for(seq, sensor_for(seq));
        stream.insert(stream.end(), f.begin(), f.end());
    }
    const auto got = feed_all(p, stream);
    ASSERT_EQ(got.size(), 5u);
    for(std::uint8_t seq = 0; seq < 5; ++seq){
        EXPECT_EQ(got[seq].seq, seq);
    }
    const auto& st = p.stats();
    EXPECT_EQ(st.cobs_errors + st.length_errors + st.crc_errors + st.type_errors + st.overflows, 0u);
}

TEST(FrameParser, RecoversWhenJoiningMidFrame){
    proto::FrameParser p;
    const Bytes first = frame_for(1, sensor_for(1));
    const Bytes second = frame_for(2, sensor_for(2));
    Bytes stream(first.begin() + 3, first.end());      // missed the first 3 bytes
    stream.insert(stream.end(), second.begin(), second.end());

    const auto got = feed_all(p, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 2);                          // partial frame rejected, next one fine
    EXPECT_EQ(p.stats().frames_ok, 1u);
}

TEST(FrameParser, RejectsCorruptedPayloadWithCrcError){
    proto::FrameParser p;
    Bytes f = frame_for(7, sensor_for(7));
    f[5] ^= 0x04;                                      // flip one bit inside the frame
    if(f[5] == 0x00){ f[5] ^= 0x08; }                  // keep the corruption from becoming a delimiter
    EXPECT_TRUE(feed_all(p, f).empty());
    const auto& st = p.stats();
    EXPECT_EQ(st.frames_ok, 0u);
    EXPECT_EQ(st.crc_errors + st.cobs_errors, 1u);
}

TEST(FrameParser, RejectsUnknownTypeAndWrongLength){
    proto::FrameParser p;
    std::uint8_t raw[8] = {99, 0, 1, 2, 3, 4};         // type 99, 4-byte payload
    std::uint16_t crc = crc16_ccitt(raw, 6);
    raw[6] = static_cast<std::uint8_t>(crc); raw[7] = static_cast<std::uint8_t>(crc >> 8);
    Bytes f = encode(Bytes(raw, raw + 8)); f.push_back(0x00);
    EXPECT_TRUE(feed_all(p, f).empty());
    EXPECT_EQ(p.stats().type_errors, 1u);

    raw[0] = static_cast<std::uint8_t>(proto::PacketType::Sensor);   // Sensor needs 12 bytes, not 4
    crc = crc16_ccitt(raw, 6);
    raw[6] = static_cast<std::uint8_t>(crc); raw[7] = static_cast<std::uint8_t>(crc >> 8);
    f = encode(Bytes(raw, raw + 8)); f.push_back(0x00);
    EXPECT_TRUE(feed_all(p, f).empty());
    EXPECT_EQ(p.stats().length_errors, 1u);
}

TEST(FrameParser, DiscardsOverlongFrameThenResyncs){
    proto::FrameParser p;
    Bytes stream(100, 0x55);                           // a lost delimiter: no 0x00 for 100 bytes
    stream.push_back(0x00);
    const Bytes good = frame_for(9, sensor_for(9));
    stream.insert(stream.end(), good.begin(), good.end());

    const auto got = feed_all(p, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].seq, 9);
    EXPECT_EQ(p.stats().overflows, 1u);
}

// ==================================================================== fuzz

TEST(FrameParserFuzz, RandomBytesNeverCrashAndAccountingAddsUp){
    proto::FrameParser p;
    std::mt19937 rng(2026);
    std::uint32_t delimiters = 0;
    for(int i = 0; i < 1'000'000; ++i){
        // ~1 in 20 bytes is a delimiter, so frames of every length get tried.
        const std::uint8_t b = (rng() % 20 == 0) ? 0x00 : static_cast<std::uint8_t>(rng());
        if(b == 0x00){ ++delimiters; }
        p.feed(b);
    }
    const auto& st = p.stats();
    const std::uint32_t judged = st.frames_ok + st.cobs_errors + st.length_errors + st.crc_errors + st.type_errors;
    EXPECT_LE(judged, delimiters);                     // at most one verdict per delimiter
    EXPECT_GT(st.cobs_errors + st.crc_errors, 0u);     // the checks really ran
}

TEST(FrameParserFuzz, CorruptedStreamNeverDeliversAWrongPacket){
    proto::FrameParser p;
    std::mt19937 rng(7);
    std::uint32_t sent = 0;
    std::uint32_t corrupted_frames = 0;
    std::uint32_t delivered = 0;

    for(std::uint32_t i = 0; i < 20'000; ++i){
        Bytes f = frame_for(static_cast<std::uint8_t>(i), sensor_for(i));
        ++sent;
        if(rng() % 10 == 0){                           // corrupt ~10% of frames
            ++corrupted_frames;
            const std::size_t pos = rng() % f.size();
            switch(rng() % 3){
                case 0: f[pos] ^= static_cast<std::uint8_t>(1u << (rng() % 8)); break;   // bit flip
                case 1: f.erase(f.begin() + static_cast<std::ptrdiff_t>(pos)); break;    // dropped byte
                case 2: f.insert(f.begin() + static_cast<std::ptrdiff_t>(pos), static_cast<std::uint8_t>(rng())); break;  // extra byte
            }
        }
        for(std::uint8_t b : f){
            if(!p.feed(b)){
                continue;
            }
            ++delivered;
            // Anything delivered must be exactly one of the frames we sent.
            proto::SensorData got{};
            ASSERT_TRUE(proto::decode_payload(p.packet(), got));
            const std::uint8_t seq = p.packet().seq;
            bool matches_a_sent_frame = false;
            for(std::uint32_t k = seq; k <= i; k += 256){     // same seq byte, any lap of the counter
                const proto::SensorData want = sensor_for(k);
                if(got.gyro_rate_dps == want.gyro_rate_dps && got.accel_x_g == want.accel_x_g && got.accel_z_g == want.accel_z_g){
                    matches_a_sent_frame = true;
                    break;
                }
            }
            ASSERT_TRUE(matches_a_sent_frame) << "corrupted packet accepted, seq " << int(seq);
        }
    }
    EXPECT_GE(delivered, sent - 2 * corrupted_frames);  // a corruption can cost at most its own frame and a neighbour
    EXPECT_LE(delivered, sent);
}
