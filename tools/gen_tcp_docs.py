from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import transport.protocol_defs as proto
from transport.message_schema import (
    MessageSpec,
    describe_payload_size,
    get_message,
    get_response_message,
    get_min_payload_size,
    iter_messages,
    iter_response_messages,
    render_struct_format,
    render_wire_type,
)
OUTPUT_PATH = ROOT / "docs" / "tcp-api.md"


def _expect(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_schema_against_protocol_defs() -> None:
    msg_1102 = get_message(0x1102)
    _expect(proto.SET_SIM_TIME_MODE_REQ_SIZE == get_min_payload_size(msg_1102), "0x1102 size mismatch")

    msg_1201 = get_message(0x1201)
    _expect(proto.SET_TRAJECTORY_FOLLOW_MODE_SIZE == 4, "internal protocol size invariant changed")
    _expect(get_min_payload_size(msg_1201) == 4, "0x1201 min size mismatch")

    msg_1302 = get_message(0x1302)
    _expect(proto.MANUAL_CONTROL_BY_ID_VALUES_FMT.endswith("ddd"), "0x1302 format mismatch")
    _expect(proto.MANUAL_CONTROL_BY_ID_MIN_SIZE == get_min_payload_size(msg_1302), "0x1302 min size mismatch")

    msg_1303 = get_message(0x1303)
    _expect(proto.TRANSFORM_CONTROL_BY_ID_VALUES_FMT.endswith("fffffffd"), "0x1303 format mismatch")
    _expect(proto.TRANSFORM_CONTROL_BY_ID_MIN_SIZE == get_min_payload_size(msg_1303), "0x1303 min size mismatch")

    msg_1304 = get_message(0x1304)
    _expect(proto.SET_TRAJECTORY_MIN_SIZE == get_min_payload_size(msg_1304), "0x1304 min size mismatch")

    msg_1402 = get_message(0x1402)
    _expect(get_min_payload_size(msg_1402) == 4, "0x1402 min size mismatch")

    msg_1505 = get_message(0x1505)
    _expect(get_min_payload_size(msg_1505) == 8, "0x1505 min size mismatch")

    resp_1101 = get_response_message(0x1101)
    _expect(proto.GET_STATUS_PAYLOAD_SIZE == get_min_payload_size(resp_1101), "0x1101 response size mismatch")

    resp_1102 = get_response_message(0x1102)
    _expect(proto.SET_SIM_TIME_MODE_RESP_SIZE == get_min_payload_size(resp_1102), "0x1102 response size mismatch")

    resp_1201 = get_response_message(0x1201)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1201), "0x1201 response size mismatch")

    resp_1202 = get_response_message(0x1202)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1202), "0x1202 response size mismatch")

    resp_1301 = get_response_message(0x1301)
    _expect(get_min_payload_size(resp_1301) == proto.RESULT_SIZE + 4, "0x1301 response min size mismatch")

    resp_1302 = get_response_message(0x1302)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1302), "0x1302 response size mismatch")

    resp_1303 = get_response_message(0x1303)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1303), "0x1303 response size mismatch")

    resp_1304 = get_response_message(0x1304)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1304), "0x1304 response size mismatch")

    resp_1401 = get_response_message(0x1401)
    _expect(
        proto.RESULT_SIZE + proto.ACTIVE_SUITE_STATUS_RESP_MIN_SIZE == get_min_payload_size(resp_1401),
        "0x1401 response min size mismatch",
    )

    resp_1402 = get_response_message(0x1402)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1402), "0x1402 response size mismatch")

    resp_1504 = get_response_message(0x1504)
    _expect(get_min_payload_size(resp_1504) == 12, "0x1504 response size mismatch")

    resp_1505 = get_response_message(0x1505)
    _expect(proto.RESULT_SIZE == get_min_payload_size(resp_1505), "0x1505 response size mismatch")


def render_message_section(message: MessageSpec) -> str:
    binding_label = "Builder" if message.direction == "request" else "Parser"
    binding_value = message.handler if message.direction == "request" else message.parser
    lines = [
        f"## `0x{message.msg_type:04X}` {message.name}",
        "",
        f"- Direction: `{message.direction}`",
        f"- Payload: `{describe_payload_size(message)}`",
        f"- {binding_label}: `{binding_value}`" if binding_value else f"- {binding_label}: n/a",
        "",
        message.summary,
        "",
        f"Wire layout: `{render_struct_format(message.fields)}`",
        "",
    ]

    if message.fields:
        lines.extend(
            [
                "| Field | Type | Description |",
                "|------|------|-------------|",
            ]
        )
        for field in message.fields:
            desc = field.description or "-"
            lines.append(f"| `{field.name}` | `{render_wire_type(field.field_type)}` | {desc} |")
        lines.append("")
    else:
        lines.append("This message has no payload.\n")

    if message.repeat_fields:
        lines.extend(
            [
                "Repeat layout:",
                "",
                "| Field | Type | Description |",
                "|------|------|-------------|",
            ]
        )
        for field in message.repeat_fields:
            desc = field.description or "-"
            lines.append(f"| `{field.name}` | `{render_wire_type(field.field_type)}` | {desc} |")
        lines.append("")

    if message.notes:
        lines.append("Notes:")
        for note in message.notes:
            lines.append(f"- {note}")
        lines.append("")

    return "\n".join(lines)


def render_document() -> str:
    request_messages = list(iter_messages())
    response_messages = list(iter_response_messages())
    messages = request_messages + response_messages
    lines = [
        "# TCP API Reference",
        "",
        "> Auto-generated from `transport/message_schema.py`. Do not edit manually.",
        "",
        "## Summary",
        "",
        "| Msg Type | Direction | Name | Payload |",
        "|----------|-----------|------|---------|",
    ]
    for message in messages:
        lines.append(
            f"| `0x{message.msg_type:04X}` | `{message.direction}` | `{message.name}` | `{describe_payload_size(message)}` |"
        )
    lines.append("")

    lines.append("## Requests")
    lines.append("")
    for message in request_messages:
        lines.append(render_message_section(message))

    lines.append("## Responses")
    lines.append("")
    for message in response_messages:
        lines.append(render_message_section(message))

    return "\n".join(lines).rstrip() + "\n"


def write_document(output_path: Path) -> None:
    output_path.write_text(render_document(), encoding="utf-8")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Generate TCP API markdown from transport.message_schema.")
    parser.add_argument("--check", action="store_true", help="Fail if the generated file is out of date.")
    args = parser.parse_args(argv)

    validate_schema_against_protocol_defs()
    rendered = render_document()

    if args.check:
        current = OUTPUT_PATH.read_text(encoding="utf-8") if OUTPUT_PATH.exists() else ""
        if current != rendered:
            raise SystemExit("docs/tcp-api.md is out of date. Run: python tools/gen_tcp_docs.py")
        return 0

    OUTPUT_PATH.write_text(rendered, encoding="utf-8")
    print(f"Wrote {OUTPUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
