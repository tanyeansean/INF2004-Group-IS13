#!/usr/bin/env python3
"""
Packed MQTT payloads, laptop side (Change Requirement 1). Layout version 1.

This mirrors packets.h on the car. Every payload is a bit string read
little-endian: bit 0 is the least significant bit of byte 0. Each field is
(first bit, width, signed, step); the real value is the stored integer x step.
test/run_tests.py checks this table against packets.h.

    python packets.py --test        # self-test
"""
import random
import sys

import barcode_map as bm

LAYOUT_VERSION = 1

# message: (bytes, {field: (first bit, width, signed, step, C constant)})
LAYOUTS = {
    "status": (1, {
        "online":    (0, 1, False, 1, "PKT_STATUS_ONLINE"),
        "version":   (4, 4, False, 1, "PKT_STATUS_VERSION"),
    }),
    "live": (12, {
        "seq":       (0, 8, False, 1, "PKT_LIVE_SEQ"),
        "state":     (8, 4, False, 1, "PKT_LIVE_STATE"),
        "speed_l":   (12, 8, True, 0.5, "PKT_LIVE_SPEED_L"),
        "speed_r":   (20, 8, True, 0.5, "PKT_LIVE_SPEED_R"),
        "dist":      (28, 14, False, 1, "PKT_LIVE_DIST"),       # wraps
        "line_bits": (42, 3, False, 1, "PKT_LIVE_LINE_BITS"),
        "line_pos":  (45, 7, True, 1, "PKT_LIVE_LINE_POS"),
        "on_line":   (52, 1, False, 1, "PKT_LIVE_ON_LINE"),
        "pitch":     (53, 8, True, 0.5, "PKT_LIVE_PITCH"),
        "yaw_rate":  (61, 9, True, 1, "PKT_LIVE_YAW_RATE"),
        "motion":    (70, 3, False, 1, "PKT_LIVE_MOTION"),
        "peak":      (73, 7, False, 0.1, "PKT_LIVE_PEAK"),
        "faults":    (80, 16, False, 1, "PKT_LIVE_FAULTS"),
    }),
    "odom": (4, {
        "enc_l":     (0, 16, False, 1, "PKT_ODOM_ENC_L"),
        "enc_r":     (16, 16, False, 1, "PKT_ODOM_ENC_R"),
    }),
    "heartbeat": (9, {
        "seq":        (0, 8, False, 1, "PKT_HB_SEQ"),
        "uptime":     (8, 16, False, 1, "PKT_HB_UPTIME"),
        "rssi":       (24, 7, False, 1, "PKT_HB_RSSI"),
        "reconnects": (31, 4, False, 1, "PKT_HB_RECONNECTS"),
        "pub_fail":   (35, 8, False, 1, "PKT_HB_PUB_FAIL"),
        "cmd_drop":   (43, 4, False, 1, "PKT_HB_CMD_DROP"),
        "ev_drop":    (47, 4, False, 1, "PKT_HB_EV_DROP"),
        "map_held":   (51, 1, False, 1, "PKT_HB_MAP_HELD"),
        "map":        (52, 20, False, 1, "PKT_HB_MAP"),
    }),
    "cmd": (1, {
        "arg":       (0, 5, False, 1, "PKT_CMD_ARG"),
        "op":        (5, 3, False, 1, "PKT_CMD_OP"),
    }),
    "ack": (1, {
        "result":    (0, 5, False, 1, "PKT_ACK_RESULT"),
        "op":        (5, 3, False, 1, "PKT_ACK_OP"),
    }),
    "ev_state": (1, {
        "from":      (0, 4, False, 1, "PKT_EV_STATE_FROM"),
        "to":        (4, 4, False, 1, "PKT_EV_STATE_TO"),
    }),
    "ev_lm": (1, {
        "letter":    (0, 5, False, 1, "PKT_EV_LETTER"),
        "move":      (5, 3, False, 1, "PKT_EV_MOVE"),
    }),
    "ev_hump": (3, {
        "peak":      (0, 7, False, 0.1, "PKT_EV_HUMP_PEAK"),
        "dist":      (7, 14, False, 1, "PKT_EV_HUMP_DIST"),
    }),
    "ev_obstacle": (6, {
        "dist":      (0, 8, False, 1, "PKT_EV_OBST_DIST"),
        "angle":     (8, 8, False, 1, "PKT_EV_OBST_ANGLE"),
        "width":     (16, 7, False, 1, "PKT_EV_OBST_WIDTH"),
        "clear_l":   (23, 8, False, 1, "PKT_EV_OBST_CLEAR_L"),
        "clear_r":   (31, 8, False, 1, "PKT_EV_OBST_CLEAR_R"),
        "action":    (39, 3, False, 1, "PKT_EV_OBST_ACTION"),
    }),
    "map": (3, {
        "bits":      (0, 20, False, 1, "PKT_MAP_BITS"),
    }),
}

# Which message each topic (under the root) carries.
TOPICS = {
    "status": "status",
    "heartbeat": "heartbeat",
    "telemetry/live": "live",
    "telemetry/odometry": "odom",
    "event/state": "ev_state",
    "event/barcode": "ev_lm",
    "event/junction": "ev_lm",
    "event/hump": "ev_hump",
    "event/obstacle": "ev_obstacle",
    "cmd": "cmd",
    "cmd/ack": "ack",
    "cmd/barcode_map": "map",
    "cmd/barcode_map/ack": "map_ack",
}

STATES = ["IDLE", "LINE_FOLLOW", "TURNING", "ON_HUMP", "OBSTACLE_SCAN",
          "AVOIDING", "LINE_RECOVERY", "STOPPED", "ERROR"]
MOTIONS = ["STATIONARY", "ACCELERATING", "TURNING", "CLIMBING", "DESCENDING", "IMPACT"]
ACTIONS = ["STOP", "CONTINUE", "LEFT", "RIGHT", "REVERSE"]
# START and SPEED have the top bit set, so no plain ASCII byte can mean them.
OPS = {"ping": 0, "stop": 1, "estop": 2, "reset_peak": 3, "speed": 4, "start": 5}
OP_NAMES = {v: k.upper() for k, v in OPS.items()}
OP_NAMES[7] = "(unreadable)"
SPEED_MAX_CMS = 20                 # the car caps speed commands here
ACK_BAD_ARG, ACK_REJECTED, ACK_BAD_LENGTH, ACK_NO_HANDLER, ACK_UNKNOWN_OP = 27, 28, 29, 30, 31
FAULT_WIFI_LOSS = 1 << 15
OUT_OF_RANGE_CM = 255


def name(table, i):
    return table[i] if 0 <= i < len(table) else f"?{i}"


def unpack(msg, payload):
    """Payload bytes -> {field: stored integer} (signed fields sign-extended)."""
    length, fields = LAYOUTS[msg]
    if len(payload) != length:
        raise ValueError(f"{msg}: expected {length} bytes, got {len(payload)}")
    v = int.from_bytes(payload, "little")
    out = {}
    for field, (pos, width, signed, _step, _c) in fields.items():
        x = (v >> pos) & ((1 << width) - 1)
        if signed and x >> (width - 1):
            x -= 1 << width
        out[field] = x
    return out


def pack(msg, **values):
    """{field: stored integer} -> payload bytes (fields not given are 0)."""
    length, fields = LAYOUTS[msg]
    v = 0
    for field, x in values.items():
        pos, width, signed, _step, _c = fields[field]
        lo, hi = (-(1 << (width - 1)), (1 << (width - 1)) - 1) if signed else (0, (1 << width) - 1)
        if not lo <= x <= hi:
            raise ValueError(f"{msg}.{field}={x} outside {lo}..{hi}")
        v |= (x & ((1 << width) - 1)) << pos
    return v.to_bytes(length, "little")


def scaled(msg, payload):
    """Payload -> {field: value in real units} (stored integer x step)."""
    fields = LAYOUTS[msg][1]
    return {f: x * fields[f][3] for f, x in unpack(msg, payload).items()}


def encode_command(text):
    """'start', 'stop', 'estop', 'speed 15', 'reset_peak', 'ping' -> 1 byte."""
    words = text.lower().split()
    if not words or words[0] not in OPS:
        raise ValueError("unknown command (start, stop, estop, speed N, reset_peak, ping)")
    op = OPS[words[0]]
    arg = 0
    if words[0] == "speed":
        if len(words) != 2 or not words[1].isdigit():
            raise ValueError("speed needs a whole number of cm/s, e.g. speed 15")
        arg = int(words[1])
        if arg > 31:
            raise ValueError("speed must be 0-31 cm/s (the car caps it at 20)")
    elif len(words) != 1:
        raise ValueError(f"{words[0]} takes no value")
    return pack("cmd", op=op, arg=arg)


def describe_status(payload):
    s = unpack("status", payload)
    note = "" if s["version"] == LAYOUT_VERSION else \
        f" - WARNING: car uses layout v{s['version']}, this laptop tool knows v{LAYOUT_VERSION}"
    return f"{'online' if s['online'] else 'offline'} (layout v{s['version']}){note}"


def describe_ack(payload):
    a = unpack("ack", payload)
    op, r = a["op"], a["result"]
    what = OP_NAMES.get(op, f"opcode {op}")
    if r == ACK_BAD_ARG:
        return f"{what}: rejected - this command takes no value (stray byte?)"
    if r == ACK_REJECTED:
        return f"{what}: rejected - the car could not take it now (queue full or not ready), send it again"
    if r == ACK_BAD_LENGTH:
        return "rejected - a command must be exactly 1 byte"
    if r == ACK_NO_HANDLER:
        return f"{what}: not handled - no command handler on the car"
    if r == ACK_UNKNOWN_OP:
        return f"{what}: rejected - unknown opcode"
    if op == OPS["ping"]:
        return "PING: pong"
    if op == OPS["speed"]:
        return f"SPEED: accepted, set to {r} cm/s"
    return f"{what}: accepted"


def map_letters(bits20):
    v = bits20
    return tuple(chr(ord('A') + ((v >> (5 * s)) & 0x1F)) if ((v >> (5 * s)) & 0x1F) <= 25 else '?'
                 for s in range(4))


def describe_heartbeat(payload):
    h = unpack("heartbeat", payload)
    held = bm.describe_map(map_letters(h["map"])) if h["map_held"] else "none"
    return (f"seq {h['seq']}, up {h['uptime']} s, rssi -{h['rssi']} dBm, "
            f"reconnects {h['reconnects']}, publish fails {h['pub_fail']}, "
            f"cmds dropped {h['cmd_drop']}, events dropped {h['ev_drop']}, map {held}")


def describe_live(payload):
    d = scaled("live", payload)
    faults = int(d["faults"])
    flag = " WIFI-LOSS" if faults & FAULT_WIFI_LOSS else ""
    return (f"{name(STATES, int(d['state'])):<13} v={d['speed_l']:+5.1f}/{d['speed_r']:+5.1f} "
            f"dist={int(d['dist']):5d}cm line={int(d['line_bits']):03b} pos={int(d['line_pos']):+d} "
            f"on={int(d['on_line'])} pitch={d['pitch']:+5.1f} yaw={int(d['yaw_rate']):+4d} "
            f"{name(MOTIONS, int(d['motion']))} peak={d['peak']:.1f}cm faults=0x{faults:04x}{flag}")


def cm(x):
    return "out of range" if x >= OUT_OF_RANGE_CM else f"{x} cm"


def describe(sub, payload):
    """One readable line for any topic (used for the screen and the CSV)."""
    msg = TOPICS.get(sub)
    try:
        if msg is None:
            return f"{payload.hex(' ')} (unknown topic)"
        if msg == "map":
            if not payload:
                return "empty (stored map removed)"
            return bm.describe_map(bm.decode_map(payload))
        if msg == "map_ack":
            return bm.describe_ack(payload)
        if msg == "ev_lm":
            return bm.describe_event(sub, payload)
        if msg == "status":
            return describe_status(payload)
        if msg == "ack":
            return describe_ack(payload)
        if msg == "heartbeat":
            return describe_heartbeat(payload)
        if msg == "live":
            return describe_live(payload)
        if msg == "cmd":
            c = unpack("cmd", payload)
            arg = f" {c['arg']}" if c["op"] == OPS["speed"] else ""
            return f"{OP_NAMES.get(c['op'], 'opcode %d' % c['op'])}{arg}"
        if msg == "odom":
            o = unpack("odom", payload)
            return f"encoders L {o['enc_l']} R {o['enc_r']} (low 16 bits)"
        if msg == "ev_state":
            e = unpack("ev_state", payload)
            return f"{name(STATES, e['from'])} -> {name(STATES, e['to'])}"
        if msg == "ev_hump":
            e = scaled("ev_hump", payload)
            return f"hump {e['peak']:.1f} cm high at {int(e['dist'])} cm"
        if msg == "ev_obstacle":
            e = unpack("ev_obstacle", payload)
            return (f"obstacle {cm(e['dist'])} at {e['angle']} deg, width {e['width']} cm, "
                    f"clear L {cm(e['clear_l'])} / R {cm(e['clear_r'])}, plan {name(ACTIONS, e['action'])}")
    except ValueError as e:
        return f"{payload.hex(' ')} ({e})"
    return payload.hex(" ")


class Unwrap:
    """Turns a wrapping field (encoder counts, distance) back into a running
    total. Call reset() when the car restarts. signed_start: read the first
    value as signed (encoder counts can go below 0; distance cannot)."""
    def __init__(self, bits=16, signed_start=True):
        self.bits = bits
        self.signed_start = signed_start
        self.reset()

    def reset(self):
        self.total = None
        self.last = None

    def update(self, low):
        span, half = 1 << self.bits, 1 << (self.bits - 1)
        if self.last is None:
            self.total = low - span if (self.signed_start and low >= half) else low
        else:
            step = (low - self.last) & (span - 1)
            self.total += step - span if step >= half else step
        self.last = low
        return self.total


Unwrap16 = Unwrap


def self_test():
    checks = []
    rng = random.Random(13)
    for msg, (length, fields) in LAYOUTS.items():
        # fields fit the message, never overlap
        used = set()
        ok = True
        for f, (pos, width, *_rest) in fields.items():
            bits = set(range(pos, pos + width))
            ok &= not (bits & used) and max(bits) < 8 * length
            used |= bits
        checks.append((ok, f"{msg}: fields fit in {length} bytes without overlap"))
        # random round trips, including both ends of every field
        ok = True
        for i in range(300):
            vals = {}
            for f, (pos, width, signed, *_r) in fields.items():
                lo, hi = (-(1 << (width - 1)), (1 << (width - 1)) - 1) if signed else (0, (1 << width) - 1)
                vals[f] = lo if i == 0 else hi if i == 1 else rng.randint(lo, hi)
            ok &= unpack(msg, pack(msg, **vals)) == vals
        checks.append((ok, f"{msg}: pack/unpack round trip"))
    checks += [
        (encode_command("start") == bytes([0xA0]), "start -> 0xA0"),
        (encode_command("stop") == bytes([0x20]), "stop -> 0x20"),
        (all(unpack("cmd", bytes([b]))["op"] not in (OPS["start"], OPS["speed"]) for b in range(128)),
         "no ASCII byte decodes as START or SPEED"),
        (encode_command("speed 15") == bytes([0x8F]), "speed 15 -> 0x8F"),
        (encode_command("PING") == bytes([0x00]), "ping -> 0x00"),
        (describe_ack(bytes([0x94])) == "SPEED: accepted, set to 20 cm/s", "ack 0x94 = speed 20 used"),
        (describe_ack(bytes([(1 << 5) | ACK_BAD_ARG])).startswith("STOP: rejected"), "bad-arg ack"),
        (describe_ack(bytes([(7 << 5) | ACK_BAD_LENGTH])).startswith("rejected - a command"), "bad-length ack"),
        (describe_status(bytes([0x11])) == "online (layout v1)", "status 0x11 = online v1"),
        (describe_status(bytes([0x10])) == "offline (layout v1)", "status 0x10 = offline v1"),
        (scaled("live", pack("live", speed_l=-128, pitch=127))["speed_l"] == -64.0, "speed -128 steps = -64.0 cm/s"),
    ]
    for bad in ("fly", "speed", "speed fast", "speed 40", "stop 3", ""):
        try:
            encode_command(bad)
            checks.append((False, f"'{bad}' should be refused"))
        except ValueError:
            checks.append((True, f"'{bad}' refused"))
    u = Unwrap(16)
    seq = [65530, 65535, 2, 10, 65534, 4]
    got = [u.update(x) for x in seq]
    checks.append((got == [-6, -1, 2, 10, -2, 4], f"encoder unwrap {got}"))
    d = Unwrap(14, signed_start=False)
    got = [d.update(x) for x in (16000, 16383, 5, 300)]
    checks.append((got == [16000, 16383, 16389, 16684], f"distance unwrap {got}"))
    u.reset()
    checks.append((u.update(3) == 3, "unwrap restarts after reset()"))

    failed = [n for good, n in checks if not good]
    for good, n in checks:
        print(("PASS " if good else "FAIL ") + n)
    print(f"{len(checks) - len(failed)}/{len(checks)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    if sys.argv[1:] == ["--test"]:
        sys.exit(self_test())
    print(__doc__)
