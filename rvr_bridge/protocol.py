"""WebSocket protocol messages for phone-as-BLE-shell relay (#21).

All messages are JSON objects with a ``type`` field. The phone sends
upstream messages (frame, imu, battery, ble_state); the computer sends
downstream commands (capture_frame, drive, raw_motors, stop, wake, sleep,
reset_yaw, get_battery, get_ble_state).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from typing import Any


# ── Upstream (phone → computer) ─────────────────────────────────────────


@dataclass
class FrameMessage:
    type: str = field(init=False, default="frame")
    jpeg_b64: str = ""
    width: int = 0
    height: int = 0
    rotation: int = 0


@dataclass
class ImuMessage:
    type: str = field(init=False, default="imu")
    accel: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    gyro: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    ts: float = 0.0


@dataclass
class BatteryMessage:
    type: str = field(init=False, default="battery")
    pct: int = 0


@dataclass
class PhoneBatteryMessage:
    type: str = field(init=False, default="phone_battery")
    pct: int = 0


@dataclass
class BleStateMessage:
    type: str = field(init=False, default="ble_state")
    state: str = ""  # idle|scanning|connecting|discovering|ready|disconnected|error


# ── Downstream (computer → phone) ───────────────────────────────────────


@dataclass
class CaptureFrameMessage:
    type: str = field(init=False, default="capture_frame")


@dataclass
class DriveMessage:
    type: str = field(init=False, default="drive")
    speed: int = 0        # 0-255
    heading: int = 0      # 0-359 degrees
    flags: int = 0        # DriveFlags


@dataclass
class RawMotorsMessage:
    type: str = field(init=False, default="raw_motors")
    l_mode: int = 0       # RawMotorMode
    l_speed: int = 0      # 0-255
    r_mode: int = 0
    r_speed: int = 0


@dataclass
class StopMessage:
    type: str = field(init=False, default="stop")
    heading: int = 0


@dataclass
class WakeMessage:
    type: str = field(init=False, default="wake")


@dataclass
class SleepMessage:
    type: str = field(init=False, default="sleep")


@dataclass
class ResetYawMessage:
    type: str = field(init=False, default="reset_yaw")


@dataclass
class GetBatteryMessage:
    type: str = field(init=False, default="get_battery")


@dataclass
class GetPhoneBatteryMessage:
    type: str = field(init=False, default="get_phone_battery")


@dataclass
class GetBleStateMessage:
    """Request the phone's *current* BLE state.

    The phone only pushes `ble_state` on a transition (`onStateChange`), so a
    fresh WS connection (e.g. after restarting this agent process) never
    learns the state if the BLE link was already up and didn't change. Sent
    on every iteration of the startup wait loop until a `ready` arrives.
    """
    type: str = field(init=False, default="get_ble_state")


@dataclass
class TorchMessage:
    type: str = field(init=False, default="torch")
    on: bool = False


@dataclass
class BeepMessage:
    type: str = field(init=False, default="beep")
    beep_type: str = "found"  # found|bump|error|click
    volume: int = 80


# ── Serialisation ────────────────────────────────────────────────────────

def encode(msg: Any) -> str:
    return json.dumps(asdict(msg))


def decode(raw: str) -> Any:
    d = json.loads(raw)
    t = d.get("type", "")
    if t == "frame":
        return FrameMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "imu":
        return ImuMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "battery":
        return BatteryMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "phone_battery":
        return PhoneBatteryMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "ble_state":
        return BleStateMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "capture_frame":
        return CaptureFrameMessage()
    if t == "drive":
        return DriveMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "raw_motors":
        return RawMotorsMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "stop":
        return StopMessage(**{k: v for k, v in d.items() if k != "type"})
    if t == "wake":
        return WakeMessage()
    if t == "sleep":
        return SleepMessage()
    if t == "reset_yaw":
        return ResetYawMessage()
    if t == "get_battery":
        return GetBatteryMessage()
    if t == "get_ble_state":
        return GetBleStateMessage()
    raise ValueError(f"Unknown message type: {t}")