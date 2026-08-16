"""PC-side parser for the K230 ball-position UART protocol.

This is the host-side companion of k230_ball_vision.py.  It builds and
validates the exact 10-byte frames the K230 sends, so you can capture serial
data from the controller RX line and decode it on a PC.

Frame layout (10 bytes, little-endian):
    0       0xAA
    1       0x55
    2       sequence number
    3..4    ball position in mm from the LEFT bar end, signed int16 x10
    5..6    ball velocity in mm/s, signed int16 (always 0 from the K230)
    7       confidence, 0..255
    8       flags (bit0=valid, bit1=calibrated, bit2=held after dropout)
    9       CRC-8/ATM over bytes 0..8

Quick self-test:
    python frame_parser.py

Usage in your own code:
    from frame_parser import parse_position_frame
    frame = bytes.fromhex("AA5502E204000000FF050E")
    info  = parse_position_frame(frame)   # raises on bad header/CRC/length
"""

HEADER_0 = 0xAA
HEADER_1 = 0x55
FRAME_LENGTH = 10

FLAG_POSITION_VALID = 0x01
FLAG_CALIBRATED = 0x02
FLAG_FILTER_INITIALIZED = 0x04

INVALID_POSITION_MM = -32768


def clamp_int(value, low, high):
    value = int(round(value))
    if value < low:
        return low
    if value > high:
        return high
    return value


def crc8_atm(data):
    """Return CRC-8/ATM (polynomial 0x07, init 0x00)."""
    crc = 0
    for value in data:
        crc ^= int(value) & 0xFF
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _append_int16_le(buffer, value):
    value = clamp_int(value, -32768, 32767)
    if value < 0:
        value += 65536
    buffer.append(value & 0xFF)
    buffer.append((value >> 8) & 0xFF)


def _read_int16_le(low, high):
    value = (int(low) & 0xFF) | ((int(high) & 0xFF) << 8)
    if value >= 32768:
        value -= 65536
    return value


def build_position_frame(seq, position_mm, velocity_mm_s, confidence, flags):
    frame = bytearray()
    frame.append(HEADER_0)
    frame.append(HEADER_1)
    frame.append(int(seq) & 0xFF)
    _append_int16_le(frame, position_mm)
    _append_int16_le(frame, velocity_mm_s)
    frame.append(clamp_int(confidence, 0, 255))
    frame.append(int(flags) & 0xFF)
    frame.append(crc8_atm(frame))
    return bytes(frame)


def parse_position_frame(frame):
    """Validate and decode one frame.

    This helper is intended for host-side serial debugging. It only uses
    language features also available in MicroPython.
    """
    if len(frame) != FRAME_LENGTH:
        raise ValueError("position frame must contain exactly 10 bytes")
    if frame[0] != HEADER_0 or frame[1] != HEADER_1:
        raise ValueError("position frame header mismatch")
    if crc8_atm(frame[:-1]) != frame[-1]:
        raise ValueError("position frame CRC mismatch")
    return {
        "seq": int(frame[2]),
        "position_mm": _read_int16_le(frame[3], frame[4]),
        "velocity_mm_s": _read_int16_le(frame[5], frame[6]),
        "confidence": int(frame[7]),
        "flags": int(frame[8]),
    }


def _selftest():
    """Round-trip a few frames through build -> parse and print the results."""
    cases = [
        (0, 1250, 0, 255, 0x03),      # 125.0 mm, valid + calibrated
        (1, 2500, 0, 255, 0x07),      # 250.0 mm, valid + calibrated + held
        (2, -32768, 0, 0, 0x02),      # invalid position, calibrated only
    ]
    for seq, pos, vel, conf, flags in cases:
        frame = build_position_frame(seq, pos, vel, conf, flags)
        parsed = parse_position_frame(frame)
        assert parsed["seq"] == (seq & 0xFF)
        assert parsed["position_mm"] == pos
        assert parsed["velocity_mm_s"] == vel
        assert parsed["confidence"] == conf
        assert parsed["flags"] == flags
        print("OK  %s  ->  %s" % (frame.hex().upper(), parsed))

    # A corrupted byte must be rejected.
    bad = bytearray(build_position_frame(9, 1250, 0, 255, 0x03))
    bad[3] ^= 0xFF
    try:
        parse_position_frame(bytes(bad))
        raise SystemExit("FAIL: corrupted frame was accepted")
    except ValueError:
        print("OK  corrupted frame rejected")
    print("all protocol self-tests passed")


if __name__ == "__main__":
    _selftest()
