"""Native state decoder extracted from the MIT-licensed native bridge.

Modified: receive-only subset; no command encoder or navigation estimator.
See recovery/licenses/LICENSE-MIT and THIRD_PARTY_NOTICES.md.
"""

from dataclasses import dataclass

import math

import struct

HEADER = struct.Struct('<IiI')

STATE = struct.Struct('<ii18dI?3xIidi??2x2d')

assert STATE.size == 200

@dataclass(frozen=True)
class NativeState:
    basic: int
    gait: int
    rpy_deg: tuple
    angular_world: tuple
    acceleration_world: tuple
    position_raw: tuple
    velocity_world: tuple
    velocity_body: tuple
    motion: int
    battery: float
    balance_adjusting: bool
    zeroed: bool
    ultrasound: tuple

def decode_state(packet):
    if len(packet) != HEADER.size + STATE.size:
        raise ValueError('Expected 212-byte documented native state packet')
    code, size, kind = HEADER.unpack_from(packet)
    if code != 0x0901 or kind != 1 or size != STATE.size:
        raise ValueError('Wrong opcode, payload size or command type')
    # Reject malformed bools rather than coercing arbitrary ABI bytes to True.
    if any(packet[12 + offset] not in (0, 1) for offset in (156, 180, 181)):
        raise ValueError('Invalid C++ bool / incompatible ABI')
    v = STATE.unpack_from(packet, HEADER.size)
    # A range channel can be invalid without corrupting the independent pose.
    # Preserve its raw value; the ROS adapter must publish invalid Range and
    # fail motion health, never clamp it into a believable free-space reading.
    numbers = (*v[2:20], v[24])
    if not all(math.isfinite(x) for x in numbers):
        raise ValueError('Nonfinite native telemetry')
    # Installed jy_exe 2.0.153 reports 93.0, not the documented 0.93.
    # Preserve raw battery units; never silently normalize a firmware variant.
    if not 0 <= v[24] <= 100:
        raise ValueError('Battery outside documented units/range: '+repr(v[24]))
    return NativeState(v[0], v[1], tuple(v[2:5]), tuple(v[5:8]),
                       tuple(v[8:11]), tuple(v[11:14]), tuple(v[14:17]),
                       tuple(v[17:20]), v[23], v[24], v[26], v[27], tuple(v[28:30]))
