"""The v3 MQTT wire formats shared with the car firmware."""

import string

ACTIONS = ("Unassigned", "Left", "Right", "Straight", "U-turn")
PHASES = ("IR setup", "Ready", "Following line", "Junction", "Scanning obstacle",
          "Bypassing obstacle", "Reacquiring line", "Stopped")
TERRAIN = ("Stationary", "Cruising", "Accelerating", "Turning", "Climbing hump",
           "Descending hump", "Tilt", "Impact")
PLANS = ("Stop", "Continue", "Bypass left", "Bypass right", "Reverse and rescan")
IR_SETUP_STEPS = (
    None,
    "Place all three IR sensors over white paper and tap GP20.",
    "Place all three IR sensors over matte black and tap GP20.",
    "Taking IR readings. Keep the car still.",
    "IR contrast is too low. Put all three sensors over white and tap GP20 to retry.",
)
FAULTS = ("Emergency stop", "Impact", "Motor or encoder", "IMU", "Line", "Blocked path",
          "Control timing", "Ultrasonic signal")


def decode_status(payload):
    if len(payload) != 13:
        raise ValueError("status length")
    word = int.from_bytes(payload, "little")

    def take(size):
        nonlocal word
        value = word & ((1 << size) - 1)
        word >>= size
        return value

    def signed10():
        value = take(10)
        return value - 1024 if value >= 512 else value

    if take(4) != 3:
        raise ValueError("status version")
    phase, terrain, barcode, plan = take(3), take(3), take(5), take(3)
    line, magnetic, faults = bool(take(1)), bool(take(1)), take(8)
    if barcode > 26 or plan >= len(PLANS):
        raise ValueError("status fields")
    distance, front, hump, width = take(24), take(12), take(10), take(10)
    speed, yaw = signed10(), signed10()
    return {
        "phase": PHASES[phase],
        "terrain": TERRAIN[terrain],
        "barcode": chr(ord("A") + barcode - 1) if barcode else None,
        "plan": PLANS[plan] if phase != 0 and width else None,
        "setup": IR_SETUP_STEPS[plan] if phase == 0 else None,
        "line": line,
        "magnetic_rate": magnetic,
        "faults": [name for index, name in enumerate(FAULTS) if faults & (1 << index)],
        "distance_mm": distance,
        "front_mm": front or None,
        "hump_mm": hump,
        "width_mm": width or None,
        "speed_mm_s": speed,
        "yaw_deg_s": yaw,
    }


def decode_routes(payload):
    if len(payload) != 10:
        raise ValueError("route table length")
    word = int.from_bytes(payload, "little")
    if (word & 3) != 1:
        raise ValueError("route table version")
    routes = {letter: (word >> (2 + 3 * index)) & 7
              for index, letter in enumerate(string.ascii_uppercase)}
    if any(action >= len(ACTIONS) for action in routes.values()):
        raise ValueError("route action")
    return routes


def encode_routes(routes):
    word = 1
    if not isinstance(routes, dict) or set(routes) - set(string.ascii_uppercase):
        raise ValueError("route keys must be uppercase A-Z")
    for index, letter in enumerate(string.ascii_uppercase):
        action = routes.get(letter, 0)
        if type(action) is not int or not 0 <= action < len(ACTIONS):
            raise ValueError("route action must be an integer from 0 to 4")
        word |= action << (2 + 3 * index)
    return word.to_bytes(10, "little")
