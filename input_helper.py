

def prompt_create_object() -> dict:
    """
    콘솔에서 create_object 파라미터를 입력받아 dict로 반환.
    빈 입력이면 괄호 안 기본값 사용.
    Windows msvcrt raw 모드 → 일반 line 모드로 전환 후 복원.
    """
    import sys

    def _read_line(prompt):
        sys.stdout.write(prompt)
        sys.stdout.flush()
        # Windows: msvcrt raw 모드 중에도 sys.stdin.readline()은 동작하지 않으므로
        # msvcrt.getwch() 루프로 직접 한 줄 읽기
        if sys.platform.startswith("win"):
            import msvcrt
            buf = []
            while True:
                ch = msvcrt.getwch()
                if ch in ('\r', '\n'):
                    sys.stdout.write('\n')
                    sys.stdout.flush()
                    break
                elif ch == '\x08':          # backspace
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
            # Unix: termios cbreak 해제 후 readline
            import termios, tty
            fd  = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)   # normal 모드 복원
                termios.tcflow(fd, termios.TCIFLUSH)
                line = sys.stdin.readline()
            finally:
                tty.setcbreak(fd)           # cbreak 재설정 (key_input 복원)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return line.strip()

    def _int(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return int(raw) if raw else default

    def _float(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return float(raw) if raw else default

    print("\n── Create Object ──────────────────────────────")
    return {
        "entity_type":          _int  ("entity_type",          1   ),
        "pos_x":                _float("pos x",                0.0 ),
        "pos_y":                _float("pos y",                0.0 ),
        "pos_z":                _float("pos z",                0.0 ),
        "rot_x":                _float("rot x",                0.0 ),
        "rot_y":                _float("rot y",                0.0 ),
        "rot_z":                _float("rot z",                0.0 ),
        "driving_mode":         _int  ("driving_mode",         1   ),
        "ground_vehicle_model": _int  ("ground_vehicle_model", 12   ),
    }

def prompt_manual_control_by_id() -> dict:
    import sys

    def _read_line(prompt):
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
                elif ch == '\x08':
                    if buf:
                        buf.pop()
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif ch == '\x03':
                    raise KeyboardInterrupt
                else:
                    buf.append(ch)
                    sys.stdout.write(ch)
                    sys.stdout.flush()
            return ''.join(buf).strip()
        else:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                termios.tcflow(fd, termios.TCIFLUSH)
                line = sys.stdin.readline()
            finally:
                tty.setcbreak(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return line.strip()

    def _str(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return raw if raw else default

    def _float(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return float(raw) if raw else default

    print("\n── Manual Control By Id ─────────────────────")
    return {
        "entity_id": _str("entity id", "Car_2"),
        "throttle": _float("throttle", 0.0),
        "brake": _float("brake", 0.0),
        "steer_angle": _float("steer angle", 0.0),
    }

def prompt_transform_control_by_id() -> dict:
    import sys

    def _read_line(prompt):
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
                elif ch == '\x08':
                    if buf:
                        buf.pop()
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif ch == '\x03':
                    raise KeyboardInterrupt
                else:
                    buf.append(ch)
                    sys.stdout.write(ch)
                    sys.stdout.flush()
            return ''.join(buf).strip()
        else:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                termios.tcflow(fd, termios.TCIFLUSH)
                line = sys.stdin.readline()
            finally:
                tty.setcbreak(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return line.strip()

    def _str(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return raw if raw else default

    def _float(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return float(raw) if raw else default

    print("\n── Transform Control By Id ─────────────────────")
    return {
        "entity_id": _str("entity id", "Car_2"),
        "pos_x": _float("pos x", 0.0),
        "pos_y": _float("pos y", 0.0),
        "pos_z": _float("pos z", 0.0),
        "rot_x": _float("rot x", 0.0),
        "rot_y": _float("rot y", 0.0),
        "rot_z": _float("rot z", 0.0),
        "steer_angle": _float("steer angle", 0.0),
    }

def prompt_transform_control() -> dict:
    import sys

    def _read_line(prompt):
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
                elif ch == '\x08':
                    if buf:
                        buf.pop()
                        sys.stdout.write('\b \b')
                        sys.stdout.flush()
                elif ch == '\x03':
                    raise KeyboardInterrupt
                else:
                    buf.append(ch)
                    sys.stdout.write(ch)
                    sys.stdout.flush()
            return ''.join(buf).strip()
        else:
            import termios, tty
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
                termios.tcflow(fd, termios.TCIFLUSH)
                line = sys.stdin.readline()
            finally:
                tty.setcbreak(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            return line.strip()

    def _float(prompt, default):
        raw = _read_line(f"  {prompt} [{default}]: ")
        return float(raw) if raw else default

    print("\n── Transform Control ─────────────────────")
    return {
        "pos_x": _float("pos x", 0.0),
        "pos_y": _float("pos y", 0.0),
        "pos_z": _float("pos z", 0.0),
        "rot_x": _float("rot x", 0.0),
        "rot_y": _float("rot y", 0.0),
        "rot_z": _float("rot z", 0.0),
        "steer_angle": _float("steer angle", 0.0),
    }