"""Raspberry Pi <-> ATmega I2C frame format (v2).

The thesis sent raw arrays of scaled integers and observed corrupted/random
values on the bus (Section 4.3.3).  v2 keeps the thesis x100 fixed-point
scaling but adds a fixed frame layout, a sequence counter and a CRC-8 so a
corrupted frame is rejected instead of driving a motor.  The C++ hardware
interface (robocraft_hardware) and the firmware implement the same layout.

Command frame  (master -> slave, 9 bytes)
    0  0xA5 header
    1  seq
    2-3 q1 target  int16, centi-degrees (thesis x100 scaling), big endian
    4-5 q2 target  int16, centi-degrees
    6  gripper servo angle 0..180
    7  flags  bit0 enable, bit1 platform brake (lock), bit2 clear fault
    8  CRC-8 (poly 0x07) over bytes 0..7

State frame  (slave -> master, 8 bytes)
    0  0x5A header
    1  seq echo
    2-3 q1 measured int16 centi-degrees
    4-5 q2 measured int16 centi-degrees
    6  status  bit0 enabled, bit1 fault, bit2 watchdog tripped, bit3 at target
    7  CRC-8 over bytes 0..6
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass

CMD_HEADER = 0xA5
STATE_HEADER = 0x5A
CMD_LEN = 9
STATE_LEN = 8


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def rad_to_centideg(a: float) -> int:
    v = int(round(math.degrees(a) * 100.0))
    return max(-32768, min(32767, v))


def centideg_to_rad(v: int) -> float:
    return math.radians(v / 100.0)


@dataclass
class Command:
    seq: int
    q1: float
    q2: float
    gripper_deg: int = 0
    enable: bool = True
    brake: bool = False
    clear_fault: bool = False

    def encode(self) -> bytes:
        flags = (1 if self.enable else 0) | (2 if self.brake else 0) | (4 if self.clear_fault else 0)
        body = struct.pack(">BBhhBB", CMD_HEADER, self.seq & 0xFF, rad_to_centideg(self.q1),
                           rad_to_centideg(self.q2), max(0, min(180, self.gripper_deg)), flags)
        return body + bytes([crc8(body)])

    @staticmethod
    def decode(frame: bytes) -> "Command":
        if len(frame) != CMD_LEN or frame[0] != CMD_HEADER or crc8(frame[:-1]) != frame[-1]:
            raise ValueError("invalid command frame")
        _, seq, q1, q2, grip, flags = struct.unpack(">BBhhBB", frame[:-1])
        return Command(seq, centideg_to_rad(q1), centideg_to_rad(q2), grip,
                       bool(flags & 1), bool(flags & 2), bool(flags & 4))


@dataclass
class State:
    seq: int
    q1: float
    q2: float
    status: int = 0

    def encode(self) -> bytes:
        body = struct.pack(">BBhhB", STATE_HEADER, self.seq & 0xFF, rad_to_centideg(self.q1),
                           rad_to_centideg(self.q2), self.status & 0xFF)
        return body + bytes([crc8(body)])

    @staticmethod
    def decode(frame: bytes) -> "State":
        if len(frame) != STATE_LEN or frame[0] != STATE_HEADER or crc8(frame[:-1]) != frame[-1]:
            raise ValueError("invalid state frame")
        _, seq, q1, q2, status = struct.unpack(">BBhhB", frame[:-1])
        return State(seq, centideg_to_rad(q1), centideg_to_rad(q2), status)
