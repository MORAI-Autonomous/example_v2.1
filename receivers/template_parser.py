# receivers/template_parser.py
"""
Generic binary parser driven by MORAI .tmpl JSON files.
Supports FIELDS and REPEAT (case-insensitive) segments.
Types: FLOAT, DOUBLE, INT32, INT64, UINT32, ENUM(→UINT32), STRING(fixed length)
"""
import json
import struct
from typing import Any, Dict, List, Optional

# struct format character + byte size per type
_TYPE_MAP: Dict[str, tuple] = {
    "FLOAT":  ("f", 4),
    "DOUBLE": ("d", 8),
    "INT32":  ("i", 4),
    "INT64":  ("q", 8),
    "UINT32": ("I", 4),
    "ENUM":   ("I", 4),  # treated as UINT32
}

MAX_REPEAT_COUNT = 256  # safety cap against corrupt count fields


# ── Field / Segment definitions ───────────────────────────────────────

class FieldDef:
    __slots__ = ("name", "variable_name", "var_type", "length")

    def __init__(self, name: str, variable_name: str,
                 var_type: str, length: int = 0):
        self.name          = name
        self.variable_name = variable_name
        self.var_type      = var_type.upper()
        self.length        = length   # only used for STRING

    @property
    def is_string(self) -> bool:
        return self.var_type == "STRING"

    @property
    def byte_size(self) -> int:
        if self.is_string:
            return self.length
        return _TYPE_MAP.get(self.var_type, ("", 0))[1]

    @property
    def struct_char(self) -> str:
        if self.is_string:
            return f"{self.length}s"
        return _TYPE_MAP.get(self.var_type, ("x",))[0]


class SegmentDef:
    __slots__ = ("seg_type", "fields", "repeat_field_name")

    def __init__(self, seg_type: str, fields: List[FieldDef],
                 repeat_field_name: str = ""):
        self.seg_type          = seg_type.upper()   # "FIELDS" or "REPEAT"
        self.fields            = fields
        self.repeat_field_name = repeat_field_name

    @property
    def is_repeat(self) -> bool:
        return self.seg_type == "REPEAT"

    def build_fmt(self) -> str:
        return "<" + "".join(f.struct_char for f in self.fields)

    def byte_size(self) -> int:
        return sum(f.byte_size for f in self.fields)


# ── Parser ────────────────────────────────────────────────────────────

class TemplateParser:
    def __init__(self, tmpl_path: str):
        with open(tmpl_path, "r", encoding="utf-8") as fp:
            raw = json.load(fp)

        mt = raw["messageTemplate"]
        self._name: str = mt.get("name", "Unknown")

        self._fields_seg: Optional[SegmentDef] = None
        self._repeat_seg: Optional[SegmentDef] = None

        for seg_raw in mt.get("segmentList", []):
            seg_type = seg_raw.get("type", "FIELDS").upper()
            fields: List[FieldDef] = []
            for f in seg_raw.get("fieldList", []):
                fields.append(FieldDef(
                    name          = f["name"],
                    variable_name = f.get("variableName", f["name"]),
                    var_type      = f.get("variableType", "FLOAT"),
                    length        = int(f.get("length", 0)),
                ))
            rfn = seg_raw.get("repeatFieldName", "")

            if seg_type == "FIELDS" and self._fields_seg is None:
                self._fields_seg = SegmentDef("FIELDS", fields, rfn)
            elif seg_type == "REPEAT" and self._repeat_seg is None:
                self._repeat_seg = SegmentDef("REPEAT", fields, rfn)

    # ── Properties ───────────────────────────────────────────────────

    @property
    def template_name(self) -> str:
        return self._name

    @property
    def has_repeat(self) -> bool:
        return self._repeat_seg is not None

    @property
    def fields_segment(self) -> Optional[SegmentDef]:
        return self._fields_seg

    @property
    def repeat_segment(self) -> Optional[SegmentDef]:
        return self._repeat_seg

    # ── Helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _decode(field: FieldDef, raw: Any) -> Any:
        if field.is_string:
            return raw.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
        return raw

    def _find_count(self, fields_dict: Dict[str, Any]) -> int:
        """
        Locate the repeat-count value in the already-parsed FIELDS dict.
        Strategy:
          1. Collect all keys that contain 'count' (case-insensitive).
          2. Try to match the key that also contains the base word from
             repeatFieldName (e.g. 'wheel' from 'wheel_attributes').
          3. Fall back to the last matching key.
        """
        if not self._repeat_seg:
            return 0

        rfn_last = self._repeat_seg.repeat_field_name.split(".")[-1]
        base = (rfn_last
                .replace("_attributes", "")
                .replace("_datas", "")
                .rstrip("s"))

        candidates = {k: v for k, v in fields_dict.items()
                      if "count" in k.lower()}
        if not candidates:
            return 0

        # prefer key that also contains the base word
        for k, v in candidates.items():
            if base and base in k.lower():
                try:
                    return max(0, int(v))
                except (TypeError, ValueError):
                    return 0

        # fallback: last candidate
        v = list(candidates.values())[-1]
        try:
            return max(0, int(v))
        except (TypeError, ValueError):
            return 0

    # ── Public parse ─────────────────────────────────────────────────

    def parse(self, data: bytes) -> Optional[Dict[str, Any]]:
        """
        Parse raw UDP bytes according to the template.

        Returns a dict:
          {
            "template_name": str,
            "field_list":    [ {name, variable_name, value, type}, ... ],
            "fields":        { variable_name: value, name: value, ... },
            "repeat_rows":   [ {"field_list": [...], "fields": {...}}, ... ],
            "raw_size":      int,
          }
        or None if data is too short for the FIELDS segment.
        """
        offset = 0
        result: Dict[str, Any] = {
            "template_name": self._name,
            "field_list":    [],
            "fields":        {},
            "repeat_rows":   [],
            "raw_size":      len(data),
        }

        # ── FIELDS segment ───────────────────────────────────────────
        if self._fields_seg:
            seg  = self._fields_seg
            size = seg.byte_size()
            if len(data) < offset + size:
                return None

            values = struct.unpack_from(seg.build_fmt(), data, offset)
            offset += size

            for i, fld in enumerate(seg.fields):
                val = self._decode(fld, values[i])
                result["field_list"].append({
                    "name":          fld.name,
                    "variable_name": fld.variable_name,
                    "value":         val,
                    "type":          fld.var_type,
                })
                result["fields"][fld.variable_name] = val
                result["fields"][fld.name]          = val   # short name alias

        # ── REPEAT segment ───────────────────────────────────────────
        if self._repeat_seg:
            count    = min(self._find_count(result["fields"]), MAX_REPEAT_COUNT)
            seg      = self._repeat_seg
            row_size = seg.byte_size()
            row_fmt  = seg.build_fmt()

            for _ in range(count):
                if len(data) < offset + row_size:
                    break
                values = struct.unpack_from(row_fmt, data, offset)
                offset += row_size

                row_fl: List[Dict] = []
                row_d:  Dict[str, Any] = {}
                for i, fld in enumerate(seg.fields):
                    val = self._decode(fld, values[i])
                    row_fl.append({
                        "name":          fld.name,
                        "variable_name": fld.variable_name,
                        "value":         val,
                        "type":          fld.var_type,
                    })
                    row_d[fld.variable_name] = val
                    row_d[fld.name]          = val

                result["repeat_rows"].append({
                    "field_list": row_fl,
                    "fields":     row_d,
                })

        return result
