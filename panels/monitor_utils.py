from __future__ import annotations
# panels/monitor_utils.py
# UDP Monitor 공통 유틸리티 함수 (monitor.py 에서 분리)

import os
from typing import Any, Dict, List

_TMPL_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "templates")


def get_templates() -> List[str]:
    if not os.path.isdir(_TMPL_DIR):
        return []
    return sorted(f for f in os.listdir(_TMPL_DIR) if f.lower().endswith(".tmpl"))


def short_label(variable_name: str, n: int = 2) -> str:
    parts = variable_name.split(".")
    return ".".join(parts[-n:]) if len(parts) >= n else variable_name


def tab_label(filename: str) -> str:
    name = filename.replace(".tmpl", "")
    return name if len(name) <= 22 else name[:21] + "..."


def make_groups(field_list: List[Dict]) -> List[Dict]:
    groups: List[Dict] = []
    i, total = 0, len(field_list)
    while i < total:
        nm = field_list[i]["name"].lower()
        if (i + 3 < total and nm == "x" and
                field_list[i+1]["name"].lower() == "y" and
                field_list[i+2]["name"].lower() == "z" and
                field_list[i+3]["name"].lower() == "w"):
            vn  = field_list[i]["variable_name"]
            pfx = vn.rsplit(".", 1)[0] if "." in vn else vn
            groups.append({"type": "xyzw", "indices": [i, i+1, i+2, i+3],
                            "label": short_label(pfx), "tag": 0})
            i += 4; continue
        if (i + 2 < total and nm == "x" and
                field_list[i+1]["name"].lower() == "y" and
                field_list[i+2]["name"].lower() == "z"):
            vn  = field_list[i]["variable_name"]
            pfx = vn.rsplit(".", 1)[0] if "." in vn else vn
            groups.append({"type": "xyz", "indices": [i, i+1, i+2],
                            "label": short_label(pfx), "tag": 0})
            i += 3; continue
        groups.append({"type": "single", "indices": [i],
                        "label": short_label(field_list[i]["variable_name"]),
                        "tag": 0})
        i += 1
    return groups


def fmt(val: Any, var_type: str) -> str:
    if var_type in ("FLOAT", "DOUBLE"):
        try:
            f = float(val)
            if abs(f) >= 1e6 or (f != 0.0 and abs(f) < 1e-4):
                return f"{f:.6e}" if var_type == "DOUBLE" else f"{f:.4e}"
            return f"{f:.6f}" if var_type == "DOUBLE" else f"{f:.4f}"
        except Exception:
            return str(val)
    return str(val)


def format_repeat_rows(rows: List[Dict]) -> str:
    if not rows:
        return "(0 items)"
    lines = [f"({len(rows)} items)"]
    for idx, row in enumerate(rows):
        fl = row.get("field_list", [])
        lines.append(f"[{idx}]")
        for g in make_groups(fl):
            t, ix = g["type"], g["indices"]
            if t == "xyz":
                i0, i1, i2 = ix
                lines.append(
                    f"  {g['label']}: "
                    f"X={fmt(fl[i0]['value'], fl[i0]['type'])}  "
                    f"Y={fmt(fl[i1]['value'], fl[i1]['type'])}  "
                    f"Z={fmt(fl[i2]['value'], fl[i2]['type'])}"
                )
            elif t == "xyzw":
                i0, i1, i2, i3 = ix
                lines.append(
                    f"  {g['label']}: "
                    f"X={fmt(fl[i0]['value'], fl[i0]['type'])}  "
                    f"Y={fmt(fl[i1]['value'], fl[i1]['type'])}  "
                    f"Z={fmt(fl[i2]['value'], fl[i2]['type'])}  "
                    f"W={fmt(fl[i3]['value'], fl[i3]['type'])}"
                )
            else:
                i0 = ix[0]
                lines.append(f"  {g['label']}: {fmt(fl[i0]['value'], fl[i0]['type'])}")
    return "\n".join(lines)
