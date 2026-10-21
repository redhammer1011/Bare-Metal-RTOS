"""link_test.py - loopback test of the firmware's UART1 packet link.

With the firmware running (scripts/run_qemu.ps1), this connects to UART1 over
TCP and checks the whole path: host -> UART1 RX ISR -> ring buffer -> FrameParser
-> build_frame -> ring buffer -> UART1 TX ISR -> host.

  1. Round trips: send one random valid packet at a time and wait for its echo.
     Every echo must be byte-identical; reports round-trip latency.
  2. Corruption: interleave valid packets with corrupted copies (bit flips,
     dropped bytes, inserted bytes). The firmware must echo every valid packet
     and none of the corrupted ones.

Usage:  python sim/link_test.py [--port 5560] [--count 500]
"""

from __future__ import annotations

import argparse
import random
import socket
import statistics
import sys
import time

from protocol import PAYLOAD_FORMAT, FrameParser, Packet, PacketType, build_frame


def random_packet(rng: random.Random, seq: int) -> tuple[bytes, Packet]:
    """A valid frame with random values, and the Packet the firmware should echo."""
    ptype = rng.choice(list(PacketType))
    values = []
    for code in PAYLOAD_FORMAT[ptype][1:]:
        values.append(rng.randrange(0, 2**32) if code == "I" else rng.uniform(-1000.0, 1000.0))
    frame = build_frame(ptype, seq, *values)
    # The expected values are what survives float32 rounding, so parse our own frame.
    return frame, FrameParser().feed(frame)[0]


def corrupt(rng: random.Random, frame: bytes) -> bytes:
    """Damage a frame the way a noisy line would, keeping its final delimiter."""
    body = bytearray(frame[:-1])
    pos = rng.randrange(len(body))
    kind = rng.randrange(3)
    if kind == 0:
        body[pos] ^= 1 << rng.randrange(8)              # bit flip
        if body[pos] == 0:
            body[pos] = 0x55                            # a new 0x00 would just split the frame
    elif kind == 1:
        del body[pos]                                   # dropped byte
    else:
        body.insert(pos, rng.randrange(1, 256))         # extra byte
    return bytes(body) + b"\x00"


class Link:
    def __init__(self, port: int):
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.parser = FrameParser()
        self.pending: list[Packet] = []

    def send(self, data: bytes) -> None:
        self.sock.sendall(data)

    def receive(self, timeout: float) -> Packet | None:
        deadline = time.perf_counter() + timeout
        while not self.pending:
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionError("firmware closed the link")
            self.pending.extend(self.parser.feed(chunk))
        return self.pending.pop(0)


def test_round_trips(link: Link, rng: random.Random, count: int) -> bool:
    latencies_ms = []
    mismatches = timeouts = 0
    for i in range(count):
        frame, expected = random_packet(rng, i & 0xFF)
        start = time.perf_counter()
        link.send(frame)
        echo = link.receive(timeout=1.0)
        if echo is None:
            timeouts += 1
            continue
        latencies_ms.append((time.perf_counter() - start) * 1000)
        if echo != expected:
            mismatches += 1
    ok = mismatches == 0 and timeouts == 0
    lat = sorted(latencies_ms)
    print(f"[round trips] sent {count}, echoed {len(lat)}, mismatches {mismatches}, timeouts {timeouts}")
    if lat:
        print(f"              latency ms: median {statistics.median(lat):.2f}, "
              f"p99 {lat[int(len(lat) * 0.99) - 1]:.2f}, max {lat[-1]:.2f}")
    return ok


def test_corruption(link: Link, rng: random.Random, count: int) -> bool:
    sent_valid: list[Packet] = []
    corrupted = 0
    for i in range(count):
        frame, expected = random_packet(rng, i & 0xFF)
        if rng.random() < 0.3:
            link.send(corrupt(rng, frame))
            corrupted += 1
        else:
            link.send(frame)
            sent_valid.append(expected)
        time.sleep(0.002)                               # don't outrun a 128-byte RX buffer

    echoes = []
    while (pkt := link.receive(timeout=1.0)) is not None:
        echoes.append(pkt)

    unexpected = [e for e in echoes if e not in sent_valid]
    missing = len(sent_valid) - (len(echoes) - len(unexpected))
    print(f"[corruption]  sent {len(sent_valid)} valid + {corrupted} corrupted; "
          f"echoed {len(echoes)}, corrupted accepted {len(unexpected)}, valid missing {missing}")
    # A corrupted frame can swallow its valid neighbour only by eating a delimiter,
    # which corrupt() never does, so every valid frame must come back.
    return not unexpected and missing == 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5560)
    ap.add_argument("--count", type=int, default=500)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rng = random.Random(args.seed)
    link = Link(args.port)
    passed = test_round_trips(link, rng, args.count) and test_corruption(link, rng, args.count)
    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
