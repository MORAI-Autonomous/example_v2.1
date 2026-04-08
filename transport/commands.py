# udp_manual.py
import socket
import struct

import transport.protocol_defs as proto

def send_manual_udp(
    udp_sock: socket.socket,
    throttle: float,
    brake: float,
    steer: float,
):
    """
    ManualCommand UDP 송신
    payload: <ddd (float64 x3) = 24 bytes
    """
    payload = struct.pack(proto.MANUAL_FMT, throttle, brake, steer)
    if len(payload) != proto.MANUAL_SIZE:
        raise RuntimeError(
            f"Manual payload size mismatch: {len(payload)} (expected {proto.MANUAL_SIZE})"
        )

    udp_sock.sendto(payload, (proto.UDP_IP, proto.UDP_PORT))
    print(
        f"[SEND][UDP] ManualCommand -> {proto.UDP_IP}:{proto.UDP_PORT} "
        f"(thr={throttle:.3f}, brk={brake:.3f}, steer={steer:.3f}) size={len(payload)}B"
    )


def send_transform_control_udp(
    udp_sock: socket.socket,
    pos_x: float,
    pos_y: float,
    pos_z: float,
    rot_x: float,
    rot_y: float,
    rot_z: float,
    steer_angle: float,
):
    """
    TransformControl UDP 송신
    payload:
        pos(x,y,z)
        rot(x,y,z)
        steer_angle

    double x7 = 56 bytes
    """

    payload = struct.pack(
        proto.TRANSFORM_CONTROL_FMT,
        pos_x,
        pos_y,
        pos_z,
        rot_x,
        rot_y,
        rot_z,
        steer_angle,
    )

    if len(payload) != proto.TRANSFORM_CONTROL_SIZE:
        raise RuntimeError(
            f"Transform payload size mismatch: {len(payload)} "
            f"(expected {proto.TRANSFORM_CONTROL_SIZE})"
        )

    udp_sock.sendto(payload, (proto.UDP_IP_TR, proto.UDP_PORT_TR))

    print(
        f"[SEND][UDP] TransformControl -> {proto.UDP_IP_TR}:{proto.UDP_PORT_TR} "
        f"pos=({pos_x:.3f},{pos_y:.3f},{pos_z:.3f}) "
        f"rot=({rot_x:.3f},{rot_y:.3f},{rot_z:.3f}) "
        f"steer={steer_angle:.3f} size={len(payload)}B"
    )