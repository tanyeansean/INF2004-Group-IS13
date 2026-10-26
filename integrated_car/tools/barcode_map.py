#!/usr/bin/env python3
"""
Barcode map for the junction moves (Yong Quan's handoff, 24 Oct 2026).

Just before the demo the instructor says which 4 letters mean LEFT, RIGHT,
FORWARD and U-TURN. The laptop sends that to the car as 3 bytes on
<root>/cmd/barcode_map, and the car answers with 1 byte on
<root>/cmd/barcode_map/ack. telemetry_monitor.py uses this file for its
"map" command; you can also run it on its own:

    python barcode_map.py F Z W X                  # LEFT RIGHT FORWARD U-TURN
    python barcode_map.py A=forward B=left C=right D=uturn
    python barcode_map.py --test                   # check against the handoff

Compare the printed payload with Yong Quan's self-test firmware, which prints
"MAP OK ... (payload 20 88 01)" for ABCD typed on its serial monitor.
"""
import sys

# Slot order in the 3-byte payload: bits 0-4, 5-9, 10-14, 15-19.
MOVES = ("LEFT", "RIGHT", "FORWARD", "UTURN")
EVENT_MOVES = MOVES + ("UNMAPPED",)       # move field of event/barcode, event/junction
LETTER_NONE = 31                           # event letter field: no letter

# nav_map_apply() return codes, sent back as the 1-byte ack.
ACK_MEANING = {
    0: "OK - map applied",
    1: "rejected - payload not 3 bytes (car keeps its previous map)",
    2: "rejected - a letter past Z, or bits 20-23 not 0 (car keeps its previous map)",
    3: "rejected - same letter used for two moves (car keeps its previous map)",
    0xFF: "not applied - firmware has no map handler",
}

MOVE_WORDS = {
    "left": 0, "right": 1,
    "forward": 2, "straight": 2,
    "uturn": 3, "u-turn": 3, "u_turn": 3,
}


def encode_map(left, right, forward, uturn):
    """Copied unchanged from the handoff (section 2)."""
    idx = [ord(c.upper()) - ord('A') for c in (left, right, forward, uturn)]
    if any(not 0 <= i <= 25 for i in idx) or len(set(idx)) != 4:
        raise ValueError("need 4 different letters A-Z")
    v = idx[0] | (idx[1] << 5) | (idx[2] << 10) | (idx[3] << 15)
    return bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])


def check_map(payload):
    """The code the car's nav_map_apply() gives for these bytes (0 = OK)."""
    if len(payload) != 3:
        return 1
    v = payload[0] | (payload[1] << 8) | (payload[2] << 16)
    idx = [(v >> (5 * slot)) & 0x1F for slot in range(4)]
    if (v >> 20) != 0 or any(i > 25 for i in idx):
        return 2
    if len(set(idx)) != 4:
        return 3
    return 0


def decode_map(payload):
    """3 bytes -> (left, right, forward, uturn) letters. ValueError if invalid."""
    code = check_map(payload)
    if code != 0:
        raise ValueError(ACK_MEANING[code].split(" (")[0])
    v = payload[0] | (payload[1] << 8) | (payload[2] << 16)
    return tuple(chr(ord('A') + ((v >> (5 * slot)) & 0x1F)) for slot in range(4))


def describe_map(letters):
    return "  ".join(f"{move}={letter}" for move, letter in zip(MOVES, letters))


def is_letter(s):
    return len(s) == 1 and s.isascii() and s.isalpha()


def parse_map_args(words):
    """
    Accepts  F Z W X   (4 separate letters, in the order LEFT RIGHT FORWARD U-TURN)
    or       A=forward B=left C=right D=uturn   (any order; forward=A also works)
    Returns (left, right, forward, uturn) as capital letters. A single word
    such as "show" is refused, so a mistyped command never becomes a map.
    """
    usage = ("give 4 separate letters in the order LEFT RIGHT FORWARD U-TURN (map F Z W X), "
             "or letter=move pairs (map A=forward B=left C=right D=uturn)")
    words = [w.strip() for w in words if w.strip()]
    if words and all("=" in w for w in words):
        slots = [None] * 4
        for w in words:
            a, b = (s.strip().lower() for s in w.split("=", 1))
            if is_letter(a) and b in MOVE_WORDS:
                letter, move = a, b
            elif is_letter(b) and a in MOVE_WORDS:
                letter, move = b, a
            else:
                raise ValueError(f"'{w}' is not letter=move ({usage})")
            slot = MOVE_WORDS[move]
            if slots[slot] is not None:
                raise ValueError(f"{MOVES[slot]} is given twice")
            slots[slot] = letter.upper()
        missing = [MOVES[i] for i, s in enumerate(slots) if s is None]
        if missing:
            raise ValueError("no letter for " + ", ".join(missing))
        letters = slots
    elif len(words) == 4 and all(is_letter(w) for w in words):
        letters = [w.upper() for w in words]
    else:
        raise ValueError(usage)
    encode_map(*letters)                  # raises unless 4 different letters A-Z
    return tuple(letters)


def describe_ack(payload):
    if len(payload) != 1:
        return f"unexpected ack of {len(payload)} bytes: {payload.hex(' ')}"
    return f"{payload[0]} = {ACK_MEANING.get(payload[0], 'unknown code')}"


def decode_letter_move(payload):
    """1-byte event -> (letter or None, move name)."""
    if len(payload) != 1:
        raise ValueError(f"expected 1 byte, got {len(payload)}")
    idx, move = payload[0] & 0x1F, payload[0] >> 5
    letter = chr(ord('A') + idx) if idx <= 25 else None
    return letter, (EVENT_MOVES[move] if move < len(EVENT_MOVES) else f"move {move}?")


def describe_event(subtopic, payload):
    try:
        letter, move = decode_letter_move(payload)
    except ValueError as e:
        return f"{payload.hex(' ')} ({e})"
    if subtopic == "event/barcode":
        if letter is None:
            return "unreadable barcode"
        return f"barcode {letter} -> {move}"
    who = f"for barcode {letter}" if letter else "(no barcode asked for it)"
    return f"junction: {move} {who}"


def self_test():
    checks = [
        (encode_map('A', 'B', 'C', 'D') == bytes.fromhex("208801"), "ABCD -> 20 88 01"),
        (encode_map('F', 'Z', 'W', 'X') == bytes.fromhex("25db0b"), "FZWX -> 25 DB 0B"),
        (decode_map(bytes.fromhex("25db0b")) == ('F', 'Z', 'W', 'X'), "decode 25 DB 0B"),
        (check_map(b"ABCD") == 1, "text 'ABCD' is rejected (wrong length)"),
        (check_map(b"") == 1, "empty payload is rejected"),
        (check_map(bytes([0x20, 0x88, 0x11])) == 2, "bits 20-23 set -> range error"),
        (check_map(bytes([26, 0, 0])) == 2, "letter 26 -> range error"),
        (check_map(bytes([0, 0, 0])) == 3, "AAAA -> duplicate error"),
        (parse_map_args(["F", "Z", "W", "X"]) == ('F', 'Z', 'W', 'X'), "positional letters"),
        (parse_map_args(["A=forward", "b=Left", "uturn=D", "C=right"]) == ('B', 'C', 'A', 'D'),
         "letter=move pairs in any order"),
        (decode_letter_move(bytes([0x05])) == ('F', "LEFT"), "event byte 05 = F LEFT"),
        (decode_letter_move(bytes([0x82])) == ('C', "UNMAPPED"), "event byte 82 = C UNMAPPED"),
        (decode_letter_move(bytes([0x5F])) == (None, "FORWARD"), "event byte 5F = no letter, FORWARD"),
    ]
    for bad in (["A", "A", "C", "D"], ["A=forward", "B=forward", "C=left", "D=uturn"],
                ["A=forward", "B=left", "C=right"], ["1", "2", "3", "4"], ["ABC"],
                ["show"], ["fzwx"], ["\u00df", "B", "C", "D"], ["\u00e9=left", "B=right", "C=forward", "D=uturn"]):
        try:
            parse_map_args(bad)
            checks.append((False, f"{bad} should be refused"))
        except ValueError:
            checks.append((True, f"{bad} refused"))
    # Every valid map survives encode -> decode, and the car would accept it.
    import itertools
    import random
    rng = random.Random(2004)
    letters = [chr(ord('A') + i) for i in range(26)]
    ok = True
    for combo in itertools.islice(itertools.permutations(letters, 4), 0, None, 997):
        payload = encode_map(*combo)
        ok &= decode_map(payload) == combo and check_map(payload) == 0
    for _ in range(2000):
        combo = tuple(rng.sample(letters, 4))
        payload = encode_map(*combo)
        ok &= decode_map(payload) == combo and check_map(payload) == 0
    checks.append((ok, "encode/decode round trip for sampled maps"))

    failed = [name for good, name in checks if not good]
    for good, name in checks:
        print(("PASS " if good else "FAIL ") + name)
    print(f"{len(checks) - len(failed)}/{len(checks)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    if sys.argv[1:] == ["--test"]:
        sys.exit(self_test())
    try:
        mapping = parse_map_args(sys.argv[1:])
    except ValueError as e:
        print(f"error: {e}")
        sys.exit(2)
    print(describe_map(mapping))
    print("payload", encode_map(*mapping).hex(" ").upper())
