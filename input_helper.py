import sys

# ============================================================
# Low-level line reader (Windows msvcrt / Unix termios 공통)
# ============================================================

def _read_line(prompt: str) -> str:
    sys.stdout.write(prompt)
    sys.stdout.flush()

    if sys.platform.startswith("win"):
        import msvcrt
        buf = []
        while True:
            ch = msvcrt.getwch()
            if ch in ('\r', '\n'):
                sys.stdout.write('\n')
                sys.stdout.flush()
                break
            elif ch == '\x08':          # Backspace
                if buf:
                    buf.pop()
                    sys.stdout.write('\b \b')
                    sys.stdout.flush()
            elif ch == '\x03':          # Ctrl+C
                raise KeyboardInterrupt
            else:
                buf.append(ch)
                sys.stdout.write(ch)
                sys.stdout.flush()
        return ''.join(buf).strip()
    else:
        import termios, tty
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            termios.tcflow(fd, termios.TCIFLUSH)
            line = sys.stdin.readline()
        finally:
            tty.setcbreak(fd)
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return line.strip()


# ============================================================
# Typed input helpers
# ============================================================

def _ask_str(prompt: str, default: str) -> str:
    raw = _read_line(f"  {prompt} [{default}]: ")
    return raw if raw else default

def _ask_int(prompt: str, default: int) -> int:
    raw = _read_line(f"  {prompt} [{default}]: ")
    return int(raw) if raw else default

def _ask_float(prompt: str, default: float) -> float:
    raw = _read_line(f"  {prompt} [{default}]: ")
    return float(raw) if raw else default

def _ask_select(prompt: str, options: dict[int, str], default: int) -> int:
    """숫자 키 → 문자열 레이블 매핑에서 선택."""
    print(f"\n  {prompt}")
    for k, v in options.items():
        print(f"    {k}: {v}")
    while True:
        raw = _read_line(f"  select [{default}]: ")
        if not raw:
            return default
        try:
            val = int(raw)
            if val in options:
                return val
        except ValueError:
            pass
        print("  invalid input. try again.")


# ============================================================
# Prompt functions
# ============================================================

def prompt_create_object() -> dict:
    print("\n── Create Object ──────────────────────────────")
    return {
        "entity_type":          _ask_int  ("entity_type",          1        ),
        "pos_x":                _ask_float("pos x",                267.5667 ),
        "pos_y":                _ask_float("pos y",               -299.4991 ),
        "pos_z":                _ask_float("pos z",                0.0522   ),
        "rot_x":                _ask_float("rot x",               -0.18     ),
        "rot_y":                _ask_float("rot y",               -179.982  ),
        "rot_z":                _ask_float("rot z",               -0.51     ),
        "driving_mode":         _ask_int  ("driving_mode",         2        ),
        "ground_vehicle_model": _ask_int  ("ground_vehicle_model", 12       ),
    }

def prompt_manual_control_by_id() -> dict:
    print("\n── Manual Control By Id ─────────────────────")
    return {
        "entity_id":  _ask_str  ("entity id",   "Car_2"),
        "throttle":   _ask_float("throttle",    0.0   ),
        "brake":      _ask_float("brake",       0.0   ),
        "steer_angle":_ask_float("steer angle", 0.0   ),
    }

def prompt_transform_control_by_id() -> dict:
    print("\n── Transform Control By Id ──────────────────")
    return {
        "entity_id":  _ask_str  ("entity id",   "Car_2"),
        "pos_x":      _ask_float("pos x",       0.0),
        "pos_y":      _ask_float("pos y",       0.0),
        "pos_z":      _ask_float("pos z",       0.0),
        "rot_x":      _ask_float("rot x",       0.0),
        "rot_y":      _ask_float("rot y",       0.0),
        "rot_z":      _ask_float("rot z",       0.0),
        "steer_angle":_ask_float("steer angle", 0.0),
    }

def prompt_transform_control() -> dict:
    print("\n── Transform Control ────────────────────────")
    return {
        "pos_x":      _ask_float("pos x",       0.0),
        "pos_y":      _ask_float("pos y",       0.0),
        "pos_z":      _ask_float("pos z",       0.0),
        "rot_x":      _ask_float("rot x",       0.0),
        "rot_y":      _ask_float("rot y",       0.0),
        "rot_z":      _ask_float("rot z",       0.0),
        "steer_angle":_ask_float("steer angle", 0.0),
    }

_SCENARIO_COMMANDS = {
    1: "PLAY",
    2: "PAUSE",
    3: "STOP",
    4: "PREV",
    5: "NEXT",    
}

def prompt_scenario_control() -> dict:
    print("\n── Scenario Control ─────────────────────────")
    command = _ask_select("Scenario Command:", _SCENARIO_COMMANDS, default=1)
    print(f"  selected: {_SCENARIO_COMMANDS[command]} ({command})")
    return {"command": command}