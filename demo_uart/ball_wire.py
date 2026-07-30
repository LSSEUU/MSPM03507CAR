"""Minimal fixed-width UART codec for H3 ball measurements."""

FRAME_SIZE = 9
HEADER_0 = 0xA5
HEADER_1 = 0x5A
FLAG_VALID = 0x01


def crc8_atm(data):
    """Return CRC-8/ATM (poly=0x07, init=0, refin=false, xorout=0)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def _check_u8(name, value):
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError("{} must be an integer".format(name))
    if not 0 <= value <= 0xFF:
        raise ValueError("{} must fit u8".format(name))


def _check_i16(name, value):
    if not isinstance(value, int) or isinstance(value, bool):
        raise TypeError("{} must be an integer".format(name))
    if not -32768 <= value <= 32767:
        raise ValueError("{} must fit i16".format(name))


def _put_i16_le(frame, offset, value):
    encoded = value & 0xFFFF
    frame[offset] = encoded & 0xFF
    frame[offset + 1] = (encoded >> 8) & 0xFF


def _get_i16_le(frame, offset):
    value = frame[offset] | (frame[offset + 1] << 8)
    return value - 0x10000 if value & 0x8000 else value


def encode_frame(seq, valid, x_mm_x10=0, vx_mm_s=0):
    """Encode one 9-byte ball-measurement frame.

    Invalid frames always carry zero position and velocity so stale values can
    never be mistaken for a current measurement.
    """
    _check_u8("seq", seq)
    if not isinstance(valid, bool):
        raise TypeError("valid must be bool")

    if not valid:
        x_mm_x10 = 0
        vx_mm_s = 0
    _check_i16("x_mm_x10", x_mm_x10)
    _check_i16("vx_mm_s", vx_mm_s)

    frame = bytearray(FRAME_SIZE)
    frame[0] = HEADER_0
    frame[1] = HEADER_1
    frame[2] = seq
    frame[3] = FLAG_VALID if valid else 0
    _put_i16_le(frame, 4, x_mm_x10)
    _put_i16_le(frame, 6, vx_mm_s)
    frame[8] = crc8_atm(frame[:8])
    return bytes(frame)


def decode_frame(frame):
    """Validate and decode one complete ball-measurement frame."""
    raw = bytes(frame)
    if len(raw) != FRAME_SIZE:
        raise ValueError("ball frame must be exactly 9 bytes")
    if raw[0] != HEADER_0 or raw[1] != HEADER_1:
        raise ValueError("invalid ball frame header")
    if raw[3] & ~FLAG_VALID:
        raise ValueError("unsupported ball frame flags")
    if crc8_atm(raw[:8]) != raw[8]:
        raise ValueError("invalid ball frame CRC8")

    x_mm_x10 = _get_i16_le(raw, 4)
    vx_mm_s = _get_i16_le(raw, 6)
    valid = bool(raw[3] & FLAG_VALID)
    if not valid and (x_mm_x10 != 0 or vx_mm_s != 0):
        raise ValueError("invalid ball frame must have a zero payload")

    return {
        "seq": raw[2],
        "flags": raw[3],
        "valid": valid,
        "x_mm_x10": x_mm_x10,
        "vx_mm_s": vx_mm_s,
    }
