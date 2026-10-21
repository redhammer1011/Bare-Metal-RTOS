"""test_protocol.py - Python protocol tests, including C++ cross-check vectors.

Run from the repo root:  python -m unittest discover -s sim
"""

import random
import struct
import unittest

from protocol import (
    FrameParser, PacketType, build_frame, cobs_decode, cobs_encode, crc16_ccitt,
)

# Exact frames produced by the C++ implementation (core/packet.hpp); the same
# bytes are asserted in tests/test_protocol.cpp. If either side changes its
# wire format, one of the two test suites fails.
GOLDEN = {
    "sensor":    (PacketType.SENSOR, 1, (12.5, -0.25, 0.97),
                  "03 01 01 01 03 48 41 01 09 80 be ec 51 78 3f 1f ca 00"),
    "actuator":  (PacketType.ACTUATOR, 2, (0.4, 0.6),
                  "0d 02 02 cd cc cc 3e 9a 99 19 3f c1 c0 00"),
    "telemetry": (PacketType.TELEMETRY, 255, (10.0, 9.5, -0.1, 1234),
                  "03 03 ff 01 03 20 41 01 09 18 41 cd cc cc bd d2 04 01 03 44 a8 00"),
    "zeros":     (PacketType.SENSOR, 0, (0.0, 0.0, 0.0),
                  "02 01 01 01 01 01 01 01 01 01 01 01 01 01 03 0b d2 00"),
}


def f32(x):
    """Round a Python float (64-bit) to the nearest 32-bit float, as the firmware stores it."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


class TestCrc(unittest.TestCase):
    def test_published_check_value(self):
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)


class TestCobs(unittest.TestCase):
    def test_known_vectors(self):
        self.assertEqual(cobs_encode(b""), b"\x01")
        self.assertEqual(cobs_encode(b"\x00"), b"\x01\x01")
        self.assertEqual(cobs_encode(bytes([0x11, 0x22, 0x00, 0x33])), bytes([0x03, 0x11, 0x22, 0x02, 0x33]))
        self.assertEqual(cobs_encode(bytes(254 * [0xAB]))[0], 0xFF)

    def test_round_trip_random(self):
        rng = random.Random(12345)
        for _ in range(500):
            data = bytes(rng.choice([0, rng.randrange(256)]) for _ in range(rng.randrange(600)))
            enc = cobs_encode(data)
            self.assertNotIn(0, enc)
            self.assertEqual(cobs_decode(enc), data)


class TestCrossCheckWithCpp(unittest.TestCase):
    def test_python_builds_identical_bytes(self):
        for name, (ptype, seq, values, hex_bytes) in GOLDEN.items():
            with self.subTest(name):
                self.assertEqual(build_frame(ptype, seq, *values), bytes.fromhex(hex_bytes))

    def test_python_parses_cpp_frames(self):
        for name, (ptype, seq, values, hex_bytes) in GOLDEN.items():
            with self.subTest(name):
                parser = FrameParser()
                packets = parser.feed(bytes.fromhex(hex_bytes))
                self.assertEqual(len(packets), 1)
                self.assertEqual(packets[0].type, ptype)
                self.assertEqual(packets[0].seq, seq)
                self.assertEqual(packets[0].values, tuple(f32(v) if isinstance(v, float) else v for v in values))


class TestParser(unittest.TestCase):
    def test_rejects_corruption_and_resyncs(self):
        good = build_frame(PacketType.ACTUATOR, 7, 0.1, 0.2)
        bad = bytearray(good)
        bad[4] ^= 0x01
        parser = FrameParser()
        packets = parser.feed(bytes(bad) + good)
        self.assertEqual([p.seq for p in packets], [7])
        self.assertEqual(parser.stats.crc_errors + parser.stats.cobs_errors, 1)

    def test_discards_overlong_frame(self):
        parser = FrameParser()
        packets = parser.feed(b"\x55" * 100 + b"\x00" + build_frame(PacketType.SET_SETPOINT, 3, 5.0))
        self.assertEqual([p.seq for p in packets], [3])
        self.assertEqual(parser.stats.overflows, 1)


if __name__ == "__main__":
    unittest.main()
