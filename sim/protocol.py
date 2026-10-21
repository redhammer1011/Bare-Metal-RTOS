"""protocol.py - Python side of the plant/firmware serial link.

Mirrors core/cobs.hpp, core/crc16.hpp, core/packet.hpp and
core/frame_parser.hpp byte for byte:

    raw packet:  [type][seq][payload ...][crc_lo][crc_hi]   CRC-16/CCITT-FALSE
    on the wire: COBS(raw packet) + b"\\x00"

Multi-byte fields are little-endian; floats are IEEE-754 binary32.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum


class PacketType(IntEnum):
    SENSOR = 1        # plant -> firmware
    ACTUATOR = 2      # firmware -> plant
    TELEMETRY = 3     # firmware -> host
    SET_SETPOINT = 4  # host -> firmware
    SET_GAINS = 5     # host -> firmware


# struct format of each payload ('<' = little-endian, f = float32, I = uint32)
PAYLOAD_FORMAT = {
    PacketType.SENSOR: "<fff",        # gyro_rate_dps, accel_x_g, accel_z_g
    PacketType.ACTUATOR: "<ff",       # motor1, motor2
    PacketType.TELEMETRY: "<fffI",    # setpoint_deg, estimate_deg, output, loop_time_us
    PacketType.SET_SETPOINT: "<f",    # setpoint_deg
    PacketType.SET_GAINS: "<fff",     # kp, ki, kd
}

HEADER_SIZE = 2
CRC_SIZE = 2
MAX_PAYLOAD = 32
MAX_RAW = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE
MAX_ENCODED = MAX_RAW + MAX_RAW // 254 + 1


# ---------------------------------------------------------------------- CRC
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF. crc16_ccitt(b"123456789") == 0x29B1."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


# --------------------------------------------------------------------- COBS
def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])          # placeholder for the first code byte
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """Returns the decoded bytes, or None if data is not valid COBS."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0 or i + code - 1 > len(data):
            return None
        run = data[i:i + code - 1]
        if 0 in run:
            return None
        out += run
        i += code - 1
        if code != 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


# ------------------------------------------------------------------ packets
@dataclass
class Packet:
    type: PacketType
    seq: int
    values: tuple


def build_frame(ptype: PacketType, seq: int, *values) -> bytes:
    """Complete wire frame for one packet, e.g. build_frame(PacketType.ACTUATOR, 7, 0.4, 0.6)."""
    payload = struct.pack(PAYLOAD_FORMAT[ptype], *values)
    raw = bytes([int(ptype), seq & 0xFF]) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + b"\x00"


@dataclass
class ParserStats:
    frames_ok: int = 0
    cobs_errors: int = 0
    length_errors: int = 0
    crc_errors: int = 0
    type_errors: int = 0
    overflows: int = 0


@dataclass
class FrameParser:
    """Byte-stream receiver with the same checks and counters as the C++ FrameParser."""
    stats: ParserStats = field(default_factory=ParserStats)
    _buf: bytearray = field(default_factory=bytearray)
    _discarding: bool = False

    def feed(self, data: bytes) -> list[Packet]:
        """Feed any number of bytes; returns the valid packets they completed."""
        packets = []
        for byte in data:
            if byte == 0:
                if self._discarding:
                    self._discarding = False
                elif self._buf:
                    pkt = self._finish()
                    if pkt is not None:
                        packets.append(pkt)
                self._buf.clear()
            elif not self._discarding:
                if len(self._buf) == MAX_ENCODED:
                    self.stats.overflows += 1
                    self._discarding = True
                    self._buf.clear()
                else:
                    self._buf.append(byte)
        return packets

    def _finish(self) -> Packet | None:
        raw = cobs_decode(bytes(self._buf))
        if raw is None or len(raw) > MAX_RAW:
            self.stats.cobs_errors += 1
            return None
        if len(raw) < HEADER_SIZE + CRC_SIZE:
            self.stats.length_errors += 1
            return None
        body, (received_crc,) = raw[:-CRC_SIZE], struct.unpack("<H", raw[-CRC_SIZE:])
        if crc16_ccitt(body) != received_crc:
            self.stats.crc_errors += 1
            return None
        try:
            ptype = PacketType(body[0])
        except ValueError:
            self.stats.type_errors += 1
            return None
        payload = body[HEADER_SIZE:]
        fmt = PAYLOAD_FORMAT[ptype]
        if len(payload) != struct.calcsize(fmt):
            self.stats.length_errors += 1
            return None
        self.stats.frames_ok += 1
        return Packet(ptype, body[1], struct.unpack(fmt, payload))
