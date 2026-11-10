#!/usr/bin/env python3
# Copyright (C) 2026  36573259+Jacob10383@users.noreply.github.com
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Closed-loop X/Y motor control for the Creality Hi.

Separate copy of motor_control.py (K2 Plus, fork commit 49450e8), trimmed
for the Hi. Differences from the K2 module:
- only the X and Y drivers are on the RS-485 bus: no closed-loop extruder,
  no transparent tunnel through nozzle_mcu, no Z/Z1 motors
- stall pins come from the config (motor_x_stall, motor_y_stall)
- stock Hi addresses (X=0x81, Y=0x82), see motor_map_hi.json
- every frame goes through a write gate set by the startup_writes option
  (none, stall, params); flash and calibration frames are always refused
- no cut calibration, MOTOR_CALIBRATE, MOTOR_FLASH_PARAM or
  REQUIRE_EXTRUDER_CLEAR commands
The object is also registered as "motor_control" so homing.py finds it.
When motor_control.py changes upstream, review the diff and port by hand
(this file is ruff-formatted: format a copy of the K2 module before diffing).
"""

from __future__ import annotations

import functools
import json
import logging
import math
import re
import struct
from dataclasses import asdict, dataclass
from functools import partial
from pathlib import Path
from typing import Iterable

from . import serial_485


def _klog(msg, *args, level=logging.info):
    level("motor_control_hi: " + msg, *args)


# ──────────────────────────────────────────────────────────────────────────
# motor_fault_decoder
# ──────────────────────────────────────────────────────────────────────────

"""Decode and format motor protection error/warning bitmasks."""


ERROR_CODE_LABELS = {
    0: "encoder mutation",
    1: "encoder read error",
    2: "software peak overcurrent",
    3: "software continuous overcurrent",
    4: "hardware overcurrent",
    5: "speed feedback continuously over limit",
    6: "position feedback over limit",
    7: "position command mutation",
    8: "excessive position tracking error",
    9: "motor check value abnormal",
    10: "motor check unstable",
    11: "motor phase not connected",
    12: "motor phase resistance mismatch",
    13: "motor check other error",
}

WARNING_CODE_LABELS = {
    0: "communication error",
    1: "motor power supply too low",
    2: "MCU overheating",
}

# Response STATUS latch bits: stall, error, warning.
FRAME_STATUS_STALL = 0x01
FRAME_STATUS_ERROR = 0x02
FRAME_STATUS_WARNING = 0x04
FRAME_STATUS_KNOWN = (
    FRAME_STATUS_STALL | FRAME_STATUS_ERROR | FRAME_STATUS_WARNING
)


def decode_protection_payload(payload: bytes) -> dict:
    error_code = int.from_bytes(payload[:4].ljust(4, b"\x00"), "little")
    warning_code = int.from_bytes(payload[4:8].ljust(4, b"\x00"), "little")
    return {
        "error_code": error_code,
        "warning_code": warning_code,
        "has_error": bool(error_code),
        "active": bool(error_code or warning_code),
    }


def decode_frame_status(status) -> dict:
    value = int(status or 0) & 0xFF
    unknown = value & ~FRAME_STATUS_KNOWN
    return {
        "status": value,
        "stalled": bool(value & FRAME_STATUS_STALL),
        "faulted": bool(value & FRAME_STATUS_ERROR),
        "warned": bool(value & FRAME_STATUS_WARNING),
        "unknown": unknown,
    }


def _decode_mask(mask: int, labels_map: dict[int, str]) -> dict:
    value = int(mask or 0)
    bits = []
    labels = []
    unknown_bits = []
    bit_index = 0
    while value:
        if value & 1:
            bits.append(bit_index)
            label = labels_map.get(bit_index)
            if label is not None:
                labels.append(label)
            if label is None:
                unknown_bits.append(bit_index)
        value >>= 1
        bit_index += 1
    return {
        "bits": bits,
        "labels": labels,
        "unknown_bits": unknown_bits,
    }


def describe_fault_detail(detail: dict) -> dict:
    error_code = int(detail["error_code"] or 0)
    warning_code = int(detail["warning_code"] or 0)
    error_detail = _decode_mask(error_code, ERROR_CODE_LABELS)
    warning_detail = _decode_mask(warning_code, WARNING_CODE_LABELS)
    return {
        "error_code": error_code,
        "warning_code": warning_code,
        "error_bits": error_detail["bits"],
        "warning_bits": warning_detail["bits"],
        "error_labels": error_detail["labels"],
        "warning_labels": warning_detail["labels"],
        "unknown_error_bits": error_detail["unknown_bits"],
        "unknown_warning_bits": warning_detail["unknown_bits"],
    }


def _format_unknown_bits(prefix: str, bits: list[int]) -> str:
    if not bits:
        return ""
    if len(bits) == 1:
        return "%s bit %d" % (prefix, bits[0])
    return "%s bits %s" % (prefix, ",".join(str(bit) for bit in bits))


def format_fault_detail(axis_label: str, detail: dict) -> str:
    decoded = describe_fault_detail(detail)
    parts = []
    if decoded["error_labels"]:
        parts.append(", ".join(decoded["error_labels"]))
    if decoded["warning_labels"]:
        warning_text = ", ".join(decoded["warning_labels"])
        if decoded["error_labels"]:
            parts.append("warning: %s" % (warning_text,))
        else:
            parts.append(warning_text)
    unknown_error_text = _format_unknown_bits(
        "unknown error", decoded["unknown_error_bits"]
    )
    if unknown_error_text:
        parts.append(unknown_error_text)
    unknown_warning_text = _format_unknown_bits(
        "unknown warning", decoded["unknown_warning_bits"]
    )
    if unknown_warning_text:
        parts.append(unknown_warning_text)
    if detail.get("unverified"):
        parts.append("unverified protection query")
        query_error = detail.get("query_error")
        if query_error:
            parts.append(str(query_error))
    if not parts:
        parts.append(
            "err=%d warn=%d" % (decoded["error_code"], decoded["warning_code"])
        )
    return "%s: %s" % (axis_label, "; ".join(parts))


def format_fault_summary(
    fault_details: dict[str, dict] | None,
    axis_labeler,
    empty_text: str = "no active fault details",
) -> str:
    if not fault_details:
        return empty_text
    parts = []
    for axis in sorted(fault_details):
        parts.append(
            format_fault_detail(axis_labeler(axis), fault_details[axis])
        )
    return ", ".join(parts)


# ──────────────────────────────────────────────────────────────────────────
# motor_param_protocol
# ──────────────────────────────────────────────────────────────────────────

"""Motor parameter protocol helpers.

Encodes typed parameter values, decodes firmware response frames, and loads the
registry metadata used for type-aware parameter I/O.
"""


PACK_HEAD = 0xF7


def encode_u16(value: int) -> bytes:
    return int(value).to_bytes(2, "little", signed=False)


def encode_i16(value: int) -> bytes:
    return int(value).to_bytes(2, "little", signed=True)


def encode_u32(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=False)


def encode_i32(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=True)


def encode_float32(value: float) -> bytes:
    return struct.pack("<f", float(value))


def decode_u16(payload: bytes) -> int:
    if len(payload) != 2:
        raise RuntimeError(
            "decode_u16 expected 2 bytes got len=%d payload_hex=%s"
            % (len(payload), payload.hex())
        )
    return int.from_bytes(payload, "little", signed=False)


def decode_i16(payload: bytes) -> int:
    if len(payload) != 2:
        raise RuntimeError(
            "decode_i16 expected 2 bytes got len=%d payload_hex=%s"
            % (len(payload), payload.hex())
        )
    return int.from_bytes(payload, "little", signed=True)


def decode_float32(payload: bytes) -> float:
    if len(payload) != 4:
        raise RuntimeError(
            "decode_float32 expected 4 bytes got len=%d payload_hex=%s"
            % (len(payload), payload.hex())
        )
    return struct.unpack("<f", payload)[0]


def decode_u32(payload: bytes) -> int:
    if len(payload) != 4:
        raise RuntimeError(
            "decode_u32 expected 4 bytes got len=%d payload_hex=%s"
            % (len(payload), payload.hex())
        )
    return int.from_bytes(payload, "little", signed=False)


def decode_i32(payload: bytes) -> int:
    if len(payload) != 4:
        raise RuntimeError(
            "decode_i32 expected 4 bytes got len=%d payload_hex=%s"
            % (len(payload), payload.hex())
        )
    return int.from_bytes(payload, "little", signed=True)


def serialize_value(value, value_type: str) -> bytes:
    if value_type == "uint16_t":
        return encode_u16(int(value))
    if value_type == "int16_t":
        return encode_i16(int(value))
    if value_type == "uint32_t":
        return encode_u32(int(value))
    if value_type == "int32_t":
        return encode_i32(int(value))
    if value_type == "float":
        return encode_float32(float(value))
    raise ValueError(f"Unsupported value_type: {value_type}")


def deserialize_value(payload: bytes, value_type: str):
    if value_type == "uint16_t":
        return decode_u16(payload)
    if value_type == "int16_t":
        return decode_i16(payload)
    if value_type == "uint32_t":
        return decode_u32(payload)
    if value_type == "int32_t":
        return decode_i32(payload)
    if value_type == "float":
        return decode_float32(payload)
    raise ValueError(f"Unsupported value_type: {value_type}")


def parse_response_frame(rx: bytes) -> dict:
    if len(rx) < 6:
        raise ValueError(f"short response: {rx.hex()}")
    if rx[0] != PACK_HEAD:
        raise ValueError(f"unexpected response head 0x{rx[0]:02X}")
    addr = rx[1]
    frame_len = rx[2]
    status = rx[3]
    func = rx[4]
    payload = rx[5:-1]
    crc = rx[-1]
    return {
        "head": rx[0],
        "addr": addr,
        "frame_len": frame_len,
        "status": status,
        "func": func,
        "payload": payload,
        "crc": crc,
    }


CALIBRATION_PARAM_IDS = (9, 25)
CALIBRATION_NEAR_ZERO_THRESHOLD = 0.15


def param_values_match(current, target, param) -> bool:
    if target is None:
        return True
    if current is None:
        return False
    if param.value_type == "float":
        target = decode_float32(encode_float32(float(target)))
        tol = float(param.tolerance or 1e-6)
        return math.isclose(
            float(current), float(target), rel_tol=0.0, abs_tol=tol
        )
    return int(current) == int(target)


def coerce_override_value(param):
    raw = param.override_value
    if raw is None:
        return None
    if param.value_type == "float":
        return float(raw)
    if param.value_type in ("uint16_t", "int16_t", "uint32_t", "int32_t"):
        return int(float(raw))
    return raw


def is_near_zero_calibration_value(value) -> bool:
    if value is None:
        return False
    magnitude = abs(float(value))
    return magnitude <= CALIBRATION_NEAR_ZERO_THRESHOLD or math.isclose(
        magnitude,
        CALIBRATION_NEAR_ZERO_THRESHOLD,
        rel_tol=0.0,
        abs_tol=1e-6,
    )


def calibration_param_result(
    axis: str, param, param_id: int, value=None, error: str | None = None
) -> dict:
    if error is not None:
        return {
            "axis": axis,
            "key": param.key,
            "param_id": param_id,
            "value": None,
            "error": error,
        }
    return {
        "axis": axis,
        "key": param.key,
        "param_id": param_id,
        "value": value,
        "near_zero": is_near_zero_calibration_value(value),
        "near_zero_threshold": CALIBRATION_NEAR_ZERO_THRESHOLD,
    }


@dataclass
class RegistryParam:
    axis: str
    addr: int
    param_id: int
    key: str
    value_type: str
    configurable: bool
    # Initial reference value, or None for unknown calibration/live state.
    # Only explicit cfg overrides create requested write targets.
    value: object | None
    tolerance: float | None
    # Runtime override state populated from `[motor_control]` config.
    override_active: bool = False
    override_value: object | None = None

    @property
    def cfg_key(self) -> str:
        return f"{self.axis}_{self.key}"


class MotorParamRegistry:
    """In-memory motor param map loaded from motor_map.json.

    The JSON is never mutated on disk.  Runtime overrides from the
    ``[motor_control]`` cfg section are applied in-memory only.
    """

    def __init__(self, data: dict):
        self.data = data
        self.by_axis_key: dict[tuple[str, str], RegistryParam] = {}
        self.by_axis_id: dict[tuple[str, int], RegistryParam] = {}
        self.axis_addr: dict[str, int] = {}

        for axis, axis_data in data.items():
            addr = int(axis_data["_addr"], 16)
            self.axis_addr[axis] = addr
            for param_name, entry in axis_data.items():
                if param_name.startswith("_"):
                    continue
                param = RegistryParam(
                    axis=axis,
                    addr=addr,
                    param_id=int(entry["value_id"]),
                    key=param_name,
                    value_type=str(entry.get("value_type", "float")),
                    configurable=bool(entry.get("configurable", True)),
                    value=entry.get("value"),
                    tolerance=entry.get("tolerance"),
                )
                self.by_axis_key[(axis, param_name)] = param
                self.by_axis_id[(axis, param.param_id)] = param

    @classmethod
    def from_file(cls, path: str | Path):
        return cls(json.loads(Path(path).read_text()))

    def get(self, axis: str, key: str) -> RegistryParam:
        return self.by_axis_key[(axis, key)]

    def get_by_id(self, axis: str, param_id: int) -> RegistryParam:
        return self.by_axis_id[(axis, int(param_id))]

    def get_addr(self, axis: str) -> int:
        return self.axis_addr[axis]

    def apply_config_overrides(
        self, cfg_values: dict[str, object | None]
    ) -> list[dict]:
        """Apply cfg overrides onto the loaded map in-memory.

        For each param, check if ``{axis}_{key}`` exists in
        ``[motor_control]`` config.  If so, mark ``override_active=True``
        and set ``override_value`` to the cfg value.
        """
        normalized_cfg = {
            str(key).strip().lower(): value for key, value in cfg_values.items()
        }
        applied = []
        for (axis, key), param in self.by_axis_key.items():
            cfg_key = f"{axis}_{key}"
            raw_value = normalized_cfg.get(cfg_key.lower())
            if raw_value is None:
                continue
            cfg_value = str(raw_value).strip()
            if cfg_value == "":
                continue
            param.override_active = True
            param.override_value = cfg_value
            applied.append(
                {
                    "axis": axis,
                    "key": key,
                    "cfg_key": cfg_key,
                    "cfg_value": cfg_value,
                }
            )
        return applied


# ──────────────────────────────────────────────────────────────────────────
# motor_firmware_client
# ──────────────────────────────────────────────────────────────────────────

"""Shared motor firmware command client.

The motor firmware command vocabulary is shared across kinematic axes and E.  The
host-side difference is how packets reach the bus: kinematic axes send body packets
through ``serial_485``, while E sends full CRC-framed packets through the
nozzle-MCU transparent tunnel.
"""


FUNC_ENCODER_CALIBRATE = 0x03
FUNC_ELEC_OFFSET_CALIBRATE = 0x04
FUNC_SYS_PARAM = 0x06
FUNC_FLASH_PARAM = 0x07
FUNC_GET = 0x08
FUNC_PROTECTION = 0x0C
FUNC_READ_ADDR = 0x0E
FUNC_STALL_MODE = 0x11

DATA_CLEAR_ERR_WARN = 0x05
PROTECTION_QUERY_DATA = 11

# Retry policy:
# Normal motor firmware commands are idempotent enough to retry once at the
# client layer. Commands that are no-ACK, flash-commit, or process-start pass
# attempts=1 at their command helper.
DEFAULT_ATTEMPTS = 2
MOTOR_NO_ACK_TIMEOUT = 0.05

# Timeout policy:
# Most motor commands respond immediately or have failed; only stall-pin event
# verification, flash persistence, and calibration need distinct waits.
MOTOR_COMMAND_TIMEOUT = 1.0
MOTOR_STALL_EVENT_VERIFY_TIMEOUT = 0.25
MOTOR_FLASH_COMMIT_TIMEOUT = 8.0
MOTOR_CALIBRATION_TIMEOUT = 60.0


def decode_untyped_payload(payload: bytes):
    if len(payload) == 4:
        return struct.unpack("<f", payload)[0]
    if len(payload) == 2:
        return int.from_bytes(payload, "little", signed=False)
    return payload.hex()


WRITE_LEVEL_NONE = 0
WRITE_LEVEL_STALL = 1
WRITE_LEVEL_PARAMS = 2
WRITE_LEVEL_NEVER = 99
WRITE_LEVEL_CHOICES = {
    "none": WRITE_LEVEL_NONE,
    "stall": WRITE_LEVEL_STALL,
    "params": WRITE_LEVEL_PARAMS,
}
WRITE_LEVEL_NAMES = {level: name for name, level in WRITE_LEVEL_CHOICES.items()}


class MotorWriteBlocked(RuntimeError):
    pass


def frame_write_level(func: int, payload: bytes) -> int:
    """Lowest startup_writes level that may send this frame.

    Anything not listed here is refused: an unknown frame is never sent.
    """
    func = int(func) & 0xFF
    payload = bytes(payload)
    first = payload[0] if payload else None
    if func == FUNC_READ_ADDR and first in (0x01, 0x02):
        # 0x01 (broadcast) latches the address from the STEP/DIR levels,
        # as the stock firmware does on every start; 0x02 reads it back.
        return WRITE_LEVEL_NONE
    if func == FUNC_GET:
        return WRITE_LEVEL_NONE
    if func == FUNC_PROTECTION and first == PROTECTION_QUERY_DATA:
        return WRITE_LEVEL_NONE
    if func == FUNC_PROTECTION and first == DATA_CLEAR_ERR_WARN:
        return WRITE_LEVEL_STALL
    if func == FUNC_STALL_MODE and first == 0x04:
        return WRITE_LEVEL_NONE
    if func == FUNC_STALL_MODE and first in (0x01, 0x02):
        return WRITE_LEVEL_STALL
    if func == FUNC_SYS_PARAM and first == 0x02:
        return WRITE_LEVEL_NONE
    if func == FUNC_SYS_PARAM and first in (0x01, 0x03):
        return WRITE_LEVEL_PARAMS
    if func == FUNC_FLASH_PARAM and first == 0x06:
        return WRITE_LEVEL_NONE
    return WRITE_LEVEL_NEVER


class MotorFirmwareClient:
    def __init__(self, transport, *, framed: bool = False):
        self.transport = transport
        self.framed = bool(framed)
        # Read-only until MotorControl applies the startup_writes option.
        self.write_level = WRITE_LEVEL_NONE

    def _check_write_gate(self, addr: int, func: int, payload: bytes):
        needed = frame_write_level(func, payload)
        if needed <= self.write_level:
            return
        if needed == WRITE_LEVEL_NEVER:
            reason = "this frame is never sent on the Creality Hi"
        else:
            reason = "needs [motor_control_hi] startup_writes: %s" % (
                WRITE_LEVEL_NAMES[needed],
            )
        raise MotorWriteBlocked(
            "motor write blocked addr=0x%02X func=0x%02X payload=%s (%s)"
            % (int(addr) & 0xFF, int(func) & 0xFF, bytes(payload).hex(), reason)
        )

    @staticmethod
    def _validate_response_crc(response: bytes):
        crc_ok, expected, actual = serial_485.check_485_frame_crc(response)
        if not crc_ok:
            raise RuntimeError(
                "response crc mismatch expected=0x%02X actual=0x%02X rx=%s"
                % (expected, actual, bytes(response).hex())
            )

    @staticmethod
    def _coerce_attempts(attempts: int) -> int:
        attempts = int(attempts)
        if attempts < 1:
            raise ValueError("motor firmware attempts must be >= 1")
        return attempts

    def _packet(self, addr: int, func: int, payload: bytes = b"") -> bytes:
        if self.framed:
            return serial_485.build_485_frame(addr, func, payload)
        return serial_485.build_485_body(addr, func, payload)

    @staticmethod
    def _public_frame_result(result: dict, frame: dict) -> dict:
        return {
            **result,
            "payload_hex": frame["payload"].hex(),
            "frame_head": frame["head"],
            "frame_len": frame["frame_len"],
            "frame_status": frame["status"],
            "frame_func": frame["func"],
            "frame_crc": frame["crc"],
        }

    def _parse_validated_response(
        self, response: bytes, addr: int, func: int, label: str
    ) -> dict:
        if len(response) < 6:
            raise RuntimeError(
                "short response for addr=0x%02X func=0x%02X rx=%s"
                % (int(addr) & 0xFF, int(func) & 0xFF, response.hex())
            )
        if response[0] != serial_485.PACK_HEAD:
            raise RuntimeError(
                "unexpected response head for %s: 0x%02X rx=%s"
                % (label, response[0], response.hex())
            )
        declared_len = response[2]
        if (
            declared_len < serial_485.PACK_LEN_MIN
            or declared_len > serial_485.PACK_LEN_MAX
        ):
            raise RuntimeError(
                "unexpected response length byte for %s: %d rx=%s"
                % (label, declared_len, response.hex())
            )
        expected_len = declared_len + 3
        if len(response) != expected_len:
            raise RuntimeError(
                "response length mismatch for %s: declared_len=%d "
                "actual=%d rx=%s"
                % (label, declared_len, len(response), response.hex())
            )
        self._validate_response_crc(response)
        frame = parse_response_frame(response)
        self._validate_frame(label, frame, addr, func)
        return frame

    def _transact_command(
        self,
        addr: int,
        func: int,
        payload: bytes = b"",
        *,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
        attempts: int = DEFAULT_ATTEMPTS,
        response_timeout: float | None = None,
        label: str = "command",
        frame_validator=None,
    ) -> tuple[dict, dict | None]:
        self._check_write_gate(addr, func, payload)
        packet = self._packet(addr, func, payload)
        attempts = self._coerce_attempts(attempts)
        for attempt in range(attempts):
            # The firmware client owns retry policy; transports get one wire
            # transaction per client attempt so serial and nozzle stay aligned.
            if response_timeout is None:
                response = self.transport.send(
                    packet, timeout=timeout, attempts=1
                )
            else:
                response = self.transport.send(
                    packet,
                    timeout=timeout,
                    attempts=1,
                    response_timeout=response_timeout,
                )
            result = {
                "addr": int(addr) & 0xFF,
                "packet_hex": packet.hex(),
                "response_hex": None if response is None else response.hex(),
            }
            retry_available = attempt + 1 < attempts
            if response is None:
                if retry_available:
                    continue
                raise RuntimeError(
                    "no response for addr=0x%02X func=0x%02X"
                    % (int(addr) & 0xFF, int(func) & 0xFF)
                )
            response = bytes(response)
            if response == b"":
                if retry_available:
                    continue
                raise RuntimeError(
                    "empty response for addr=0x%02X func=0x%02X"
                    % (int(addr) & 0xFF, int(func) & 0xFF)
                )
            if len(response) < 6:
                if retry_available:
                    continue
                raise RuntimeError(
                    "short response for addr=0x%02X func=0x%02X rx=%s"
                    % (int(addr) & 0xFF, int(func) & 0xFF, response.hex())
                )
            try:
                frame = self._parse_validated_response(
                    response, addr, func, label
                )
                if frame_validator is not None:
                    frame_validator(frame)
            except RuntimeError:
                if retry_available:
                    continue
                raise
            return self._public_frame_result(result, frame), frame
        raise RuntimeError(
            "no response for addr=0x%02X func=0x%02X"
            % (int(addr) & 0xFF, int(func) & 0xFF)
        )

    def _no_ack_command(
        self,
        addr: int,
        func: int,
        payload: bytes = b"",
        *,
        timeout: float = MOTOR_NO_ACK_TIMEOUT,
    ) -> dict:
        self._check_write_gate(addr, func, payload)
        packet = self._packet(addr, func, payload)
        response = self.transport.send(packet, timeout=timeout, attempts=1)
        response_bytes = None if response is None else bytes(response)
        response_hex = None if response_bytes is None else response_bytes.hex()
        unexpected = bool(response_bytes)
        return {
            "addr": int(addr) & 0xFF,
            "packet_hex": packet.hex(),
            "response_hex": response_hex,
            "unexpected_response": unexpected,
            "unexpected_response_hex": response_hex if unexpected else None,
        }

    def _acked_command(
        self,
        addr: int,
        func: int,
        payload: bytes = b"",
        *,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
        attempts: int = DEFAULT_ATTEMPTS,
        response_timeout: float | None = None,
        label: str = "command",
        frame_validator=None,
    ) -> tuple[dict, dict | None]:
        return self._transact_command(
            addr,
            func,
            payload,
            timeout=timeout,
            attempts=attempts,
            response_timeout=response_timeout,
            label=label,
            frame_validator=frame_validator,
        )

    @staticmethod
    def _validate_frame(label: str, frame: dict, addr: int, func: int):
        if int(addr) not in (0xFE, 0xFF) and frame["addr"] != (
            int(addr) & 0xFF
        ):
            raise RuntimeError(
                "unexpected %s addr: 0x%02X expected=0x%02X"
                % (label, frame["addr"], int(addr) & 0xFF)
            )
        if frame["func"] != func:
            raise RuntimeError(
                "unexpected %s func: 0x%02X" % (label, frame["func"])
            )

    def _validated_frame_result(
        self,
        result: dict,
        frame: dict | None,
        label: str,
        strict_status: bool = False,
    ) -> tuple[dict, dict]:
        if frame is None:
            raise RuntimeError("missing response frame for %s" % (label,))
        if strict_status and frame["status"] != 0x00:
            raise RuntimeError(
                "unexpected %s status: 0x%02X" % (label, frame["status"])
            )
        return frame, result

    @staticmethod
    def _decode_param_payload(payload: bytes, value_type: str | None):
        if len(payload) == 0:
            return None
        if value_type is not None:
            return deserialize_value(payload, value_type)
        return decode_untyped_payload(payload)

    def _decode_write_ack_payload(
        self, payload: bytes, value_type: str | None
    ) -> tuple[int | None, object | None]:
        ack_status = None
        response_value = None
        if payload:
            if len(payload) == 1:
                ack_status = payload[0]
            else:
                response_value = self._decode_param_payload(payload, value_type)
        return ack_status, response_value

    def sys_param_read(
        self,
        addr: int,
        param_id: int,
        *,
        value_type: str | None = None,
        param_key: str | None = None,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_SYS_PARAM,
            bytes([0x02, int(param_id) & 0xFF]),
            timeout=timeout,
            label="sys_param_read",
        )
        frame, public = self._validated_frame_result(
            result, frame, "sys_param_read"
        )
        return {
            **public,
            "param_id": int(param_id),
            "param_key": param_key,
            "value": self._decode_param_payload(frame["payload"], value_type),
        }

    def sys_param_write(
        self,
        addr: int,
        param_id: int,
        value,
        *,
        value_type: str,
        param_key: str | None = None,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ) -> dict:
        value_bytes = serialize_value(value, value_type)
        result, frame = self._acked_command(
            addr,
            FUNC_SYS_PARAM,
            bytes([0x01, int(param_id) & 0xFF]) + value_bytes,
            timeout=timeout,
            label="sys_param_write",
        )
        frame, public = self._validated_frame_result(
            result, frame, "sys_param_write"
        )
        payload = frame["payload"]
        ack_status, response_value = self._decode_write_ack_payload(
            payload, value_type
        )
        return {
            **public,
            "param_id": int(param_id),
            "param_key": param_key,
            "value": value,
            "ack_status": ack_status,
            "response_value": response_value,
        }

    def sys_param_apply(
        self, addr: int, *, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_SYS_PARAM,
            b"\x03",
            timeout=timeout,
            label="sys_param_apply",
        )
        _frame, result = self._validated_frame_result(
            result, frame, "sys_param_apply"
        )
        return {**result, "applied": True}

    def flash_param_read(
        self,
        addr: int,
        param_id: int,
        *,
        value_type: str | None = None,
        param_key: str | None = None,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_FLASH_PARAM,
            bytes([0x06, int(param_id) & 0xFF]),
            timeout=timeout,
            label="flash_param_read",
        )
        frame, public = self._validated_frame_result(
            result, frame, "flash_param_read"
        )
        return {
            **public,
            "param_id": int(param_id),
            "param_key": param_key,
            "value": self._decode_param_payload(frame["payload"], value_type),
        }

    def protection(
        self, addr: int, data: int = 11, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_PROTECTION,
            bytes([int(data) & 0xFF]),
            timeout=timeout,
            label="protection",
        )
        frame, public = self._validated_frame_result(
            result, frame, "protection"
        )
        return {
            **public,
            "data": int(data),
            **decode_protection_payload(frame["payload"]),
        }

    def protection_clear(
        self, addr: int, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ) -> dict:
        return self._no_ack_command(
            addr, FUNC_PROTECTION, bytes([DATA_CLEAR_ERR_WARN]), timeout=timeout
        )

    def read_addr(
        self, addr: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        result, frame = self._acked_command(
            addr, FUNC_READ_ADDR, b"\x02", timeout=timeout, label="read_addr"
        )
        frame, public = self._validated_frame_result(result, frame, "read_addr")
        payload = frame["payload"]
        if len(payload) != 1:
            raise RuntimeError(
                "unexpected read_addr payload len=%d payload_hex=%s"
                % (len(payload), payload.hex())
            )
        decoded = decode_frame_status(frame["status"])
        if decoded["unknown"]:
            raise RuntimeError(
                "unexpected read_addr status for addr=0x%02X: 0x%02X"
                " (unknown status bits 0x%02X - not safe to proceed)"
                % (int(addr) & 0xFF, decoded["status"], decoded["unknown"])
            )
        return {
            **public,
            "reported_addr": payload[0],
            "status": decoded["status"],
            "faulted": decoded["faulted"],
            "warned": decoded["warned"],
            "stalled": decoded["stalled"],
        }

    def startup_prepare_probe_bus(
        self, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ) -> dict:
        return self._no_ack_command(
            0xFF, FUNC_READ_ADDR, b"\x01", timeout=timeout
        )

    def startup_probe_addr(
        self, addr: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_READ_ADDR,
            b"\x02",
            timeout=timeout,
            label="startup_probe",
        )
        frame, public = self._validated_frame_result(
            result, frame, "startup_probe"
        )
        decoded = decode_frame_status(frame["status"])
        if decoded["unknown"]:
            raise RuntimeError(
                "unexpected startup probe status for addr=0x%02X: 0x%02X"
                " (unknown status bits 0x%02X - not safe to proceed)"
                % (int(addr) & 0xFF, decoded["status"], decoded["unknown"])
            )
        return {
            **public,
            "status": decoded["status"],
            "faulted": decoded["faulted"],
            "warned": decoded["warned"],
            "stalled": decoded["stalled"],
        }

    def set_stall_mode(
        self, addr: int, mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        requested = int(mode) & 0xFF

        def validate_ack(frame):
            payload = frame["payload"]
            ack_mode = payload[0] if payload else None
            if ack_mode != requested:
                raise RuntimeError(
                    "stall_mode ack mismatch addr=0x%02X "
                    "requested=0x%02X ack=%s"
                    % (
                        int(addr) & 0xFF,
                        requested,
                        (
                            "missing"
                            if ack_mode is None
                            else "0x%02X" % (ack_mode,)
                        ),
                    )
                )

        result, frame = self._acked_command(
            addr,
            FUNC_STALL_MODE,
            bytes([int(mode) & 0xFF]),
            timeout=timeout,
            label="stall_mode",
            frame_validator=validate_ack,
        )
        frame, public = self._validated_frame_result(
            result, frame, "stall_mode"
        )
        payload = frame["payload"]
        return {
            **public,
            "mode": payload[0],
        }

    def get_stall_mode_raw(
        self, addr: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        result, frame = self._acked_command(
            addr,
            FUNC_STALL_MODE,
            b"\x04",
            timeout=timeout,
            label="stall_mode_read",
        )
        frame, public = self._validated_frame_result(
            result, frame, "stall_mode_read"
        )
        mode = frame["payload"][0] if frame["payload"] else None
        if mode is None:
            raise RuntimeError(
                "empty payload for raw stall-mode get addr=0x%02X"
                % (int(addr) & 0xFF,)
            )
        return {
            **public,
            "mode": mode,
            "raw_mode": mode,
        }

    def get_value(
        self,
        addr: int,
        index: int,
        *,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
        attempts: int = DEFAULT_ATTEMPTS,
    ) -> float:
        result, frame = self._acked_command(
            addr,
            FUNC_GET,
            bytes([int(index) & 0xFF]),
            timeout=timeout,
            attempts=attempts,
            label="get",
        )
        frame, _public = self._validated_frame_result(result, frame, "get")
        if len(frame["payload"]) != 4:
            raise RuntimeError(
                "unexpected get payload length=%d hex=%s"
                % (len(frame["payload"]), frame["payload"].hex())
            )
        return struct.unpack("<f", frame["payload"])[0]


# ──────────────────────────────────────────────────────────────────────────
# motor_control_debug_surface
# ──────────────────────────────────────────────────────────────────────────

"""Debug/status command mixin for motor control."""


class MotorControlDebugSurfaceMixin:
    def _iter_runtime_cfg_override_params(
        self, axes: tuple[str, ...] | None = None
    ):
        allowed = set(self.managed_axes if axes is None else axes)
        for (axis, key), param in self.registry.by_axis_key.items():
            if not param.override_active:
                continue
            if axis not in allowed:
                continue
            yield param

    def _collect_cfg_override_status(
        self,
        axes: tuple[str, ...] | None = None,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ) -> dict:
        results = []
        ok = mismatch = errors = 0
        for param in self._iter_runtime_cfg_override_params(axes=axes):
            target = self._coerce_value(param, param.override_value)
            try:
                current = self.axes.target(param.axis).sys_param_read(
                    param, timeout=timeout
                )["value"]
                matches = param_values_match(current, target, param)
                entry = {
                    "axis": param.axis,
                    "axis_label": self._format_axis_label(param.axis),
                    "key": param.key,
                    "cfg_key": param.cfg_key,
                    "target_value": target,
                    "current_value": current,
                    "matches": bool(matches),
                }
                if matches:
                    ok += 1
                else:
                    mismatch += 1
            except Exception as exc:
                entry = {
                    "axis": param.axis,
                    "axis_label": self._format_axis_label(param.axis),
                    "key": param.key,
                    "cfg_key": param.cfg_key,
                    "target_value": target,
                    "current_value": None,
                    "matches": False,
                    "error": repr(exc),
                }
                errors += 1
            results.append(entry)
        return {
            "count": len(results),
            "ok": ok,
            "mismatch": mismatch,
            "errors": errors,
            "results": results,
        }

    def _format_cfg_override_status_line(self, entry: dict) -> str:
        axis_label = entry["axis_label"]
        key = entry["key"]
        target = entry.get("target_value")
        current = entry.get("current_value")
        if entry.get("error"):
            return f"{axis_label} {key}: ERROR | cfg={target} | err={entry['error']}"
        state = "OK" if entry.get("matches") else "MISMATCH"
        return f"{axis_label} {key}: {state} | cfg={target} | current={current}"

    def _read_axis_calibration_status(self, axis: str) -> dict:
        axis_label = self._format_axis_label(axis)
        try:
            entries = self.axes.read_calibration_params(
                axis, timeout=MOTOR_COMMAND_TIMEOUT
            )
        except Exception as exc:
            return {
                "axis": axis,
                "axis_label": axis_label,
                "values": {},
                "near_zero": False,
                "unsafe_entries": [],
                "error": repr(exc),
            }

        values = {}
        errors = {}
        unsafe_entries = []
        for entry in entries:
            key = entry["key"]
            value_entry = {
                "param_id": entry.get("param_id"),
                "value": entry.get("value"),
                "near_zero": bool(entry.get("near_zero")),
                "near_zero_threshold": entry.get("near_zero_threshold"),
            }
            if entry.get("error"):
                value_entry["error"] = entry["error"]
                errors[key] = entry["error"]
            values[key] = value_entry
            if entry.get("near_zero"):
                unsafe_entries.append(entry)

        result = {
            "axis": axis,
            "axis_label": axis_label,
            "values": values,
            "near_zero": bool(unsafe_entries),
            "unsafe_entries": unsafe_entries,
        }
        if errors:
            result["errors"] = errors
        return result

    def _collect_calibration_status(self) -> list[dict]:
        return [
            self._read_axis_calibration_status(axis)
            for axis in self.managed_axes
        ]

    def _format_calibration_status_line(self, entry: dict) -> str:
        axis_label = entry["axis_label"]
        if entry.get("error"):
            return "Calibration %s: ERROR %s" % (axis_label, entry["error"])

        values = entry.get("values", {})
        offset = values.get("param_elec_offset", {}).get("value")
        offset_err = values.get("param_elec_offset_err_deg", {}).get("value")
        line = "Calibration %s: offset=%s pulse | offset_err=%s deg" % (
            axis_label,
            self._format_decimal(offset),
            self._format_decimal(offset_err),
        )
        if entry.get("near_zero"):
            thresholds = {
                self._format_unsafe_calibration_threshold(unsafe_entry)
                for unsafe_entry in entry.get("unsafe_entries", [])
            }
            threshold_text = ", ".join(sorted(thresholds)) or "near zero"
            line += " | UNSAFE %s" % (threshold_text,)
        if entry.get("errors"):
            error_text = ", ".join(
                "%s=%s" % (key, value)
                for key, value in sorted(entry["errors"].items())
            )
            line += " | errors: %s" % (error_text,)
        return line

    def _transport_ready_status(self) -> dict:
        return {
            "serial485_transport_ready": bool(
                self.transport is not None and self.transport.is_configured()
            ),
            "nozzle_transport_ready": bool(
                self.extruder_transport is not None
                and self.extruder_transport.is_configured()
            ),
        }

    def _build_status_snapshot(self) -> dict:
        homing_session = self._get_homing_session_state()
        decoded_fault_detail = {
            axis: describe_fault_detail(detail)
            for axis, detail in sorted(self.motor_fault_detail.items())
        }
        return {
            "implementation": "active-python",
            "controller_class": self.__class__.__name__,
            "axis_controller_class": self.axes.__class__.__name__,
            "transport_class": self.transport.__class__.__name__,
            **self._transport_ready_status(),
            "registry_param_count": len(self.registry.by_axis_key),
            "is_ready": self.is_ready,
            "motor_params_init": self.motor_params_init,
            "motor_ready": self.is_ready and self.motor_params_init,
            "is_homing": self.is_homing,
            "homing_move_depth": self._homing_move_depth,
            "homing_session_active": homing_session["active"],
            "homing_session_aborted": homing_session["aborted"],
            "homing_abort_in_progress": homing_session["abort_in_progress"],
            "startup_started": self._startup_started,
            "startup_complete": self._startup_complete,
            "startup_step_index": self._startup_step_index,
            "startup_step_count": len(self._startup_steps()),
            "startup_error": self._startup_error,
            "runtime_cfg_override_count": len(self._runtime_cfg_overrides),
            "stall_state": self.stall_monitor.read_all(),
            "cut_check_active": bool(self.is_check_cut_pos_start),
            "cut_latched": bool(self.cut_state),
            "cut_pos_offset": self.cut_pos_offset,
            "fault_cleanup_pending": self._fault_cleanup_pending,
            "fault_cleanup_axes": sorted(self._fault_cleanup_axes),
            "fault_cleanup_homing_fault": self._fault_cleanup_homing_fault,
            "fault_cleanup_startup_fault": self._fault_cleanup_startup_fault,
            "startup_fault_clear_ok": self._startup_fault_clear_ok,
            "startup_fault_clear_result": self._startup_fault_clear_result,
            "motor_error_code": self.motor_error_code,
            "motor_warning_code": self.motor_warning_code,
            "motor_fault_detail": self.motor_fault_detail,
            "motor_fault_detail_decoded": decoded_fault_detail,
            "calibration": self._collect_calibration_status(),
        }

    def cmd_MOTOR_STATUS(self, gcmd):
        status = self._build_status_snapshot()
        if self._gcmd_raw_detail(gcmd):
            gcmd.respond_info(
                "MOTOR_STATUS %s" % (json.dumps(status, default=str),)
            )
            return status
        if status["motor_ready"]:
            gcmd.respond_info("Motor control: ready")
        elif status["startup_complete"] and status["startup_error"]:
            gcmd.respond_info(
                "Motor control: startup failed (%s)"
                % (status["startup_error"],)
            )
        elif status["startup_started"]:
            gcmd.respond_info(
                "Motor control: starting (step %d/%d)"
                % (
                    status["startup_step_index"] + 1,
                    status["startup_step_count"],
                )
            )
        else:
            gcmd.respond_info("Motor control: idle")
        gcmd.respond_info(
            "Serial485 transport: %s"
            % ("ready" if status["serial485_transport_ready"] else "not ready",)
        )
        gcmd.respond_info(
            "Nozzle transport: %s"
            % ("ready" if status["nozzle_transport_ready"] else "not ready",)
        )
        active_faults = {
            axis: detail
            for axis, detail in sorted(self.motor_fault_detail.items())
            if detail.get("active")
        }
        if active_faults:
            gcmd.respond_info(
                "Active faults: %s"
                % (self._format_fault_summary(active_faults),)
            )
        else:
            gcmd.respond_info("Active faults: none")
        for entry in status["calibration"]:
            gcmd.respond_info(self._format_calibration_status_line(entry))
        if status["fault_cleanup_pending"]:
            gcmd.respond_info(
                "Fault cleanup: pending axes=%s"
                % (",".join(status["fault_cleanup_axes"]) or "none",)
            )
        if status["startup_fault_clear_result"]:
            gcmd.respond_info(status["startup_fault_clear_result"])
        return status

    def cmd_MOTOR_CFG_OVERRIDE_STATUS(self, gcmd):
        try:
            if (
                gcmd.get("NUM", None) is not None
                or gcmd.get("EXTRUDER", None) is not None
            ):
                raise RuntimeError(
                    "Use AXIS=%s; NUM and EXTRUDER are not supported"
                    % (self._format_axis_labels(self.managed_axes),)
                )
            axis_raw = gcmd.get("AXIS", None)
            axes = (
                self._parse_managed_axis_spec(axis_raw)
                if axis_raw is not None
                else None
            )
            raw_detail = self._gcmd_raw_detail(gcmd)
            status = self._collect_cfg_override_status(
                axes=axes, timeout=MOTOR_COMMAND_TIMEOUT
            )
            if raw_detail:
                gcmd.respond_info("MOTOR_CFG_OVERRIDE_STATUS %s" % (status,))
                return status
            if status["count"] == 0:
                gcmd.respond_info(
                    "MOTOR_CFG_OVERRIDE_STATUS no live cfg-backed param overrides"
                )
                return status
            for entry in status["results"]:
                gcmd.respond_info(self._format_cfg_override_status_line(entry))
            gcmd.respond_info(
                "MOTOR_CFG_OVERRIDE_STATUS summary count=%d ok=%d mismatch=%d errors=%d"
                % (
                    status["count"],
                    status["ok"],
                    status["mismatch"],
                    status["errors"],
                )
            )
            return status
        except Exception as exc:
            raise gcmd.error(str(exc))

    def cmd_MOTOR_RETRY_STARTUP(self, gcmd):
        self._begin_startup(force=True, allow_auto_retry=False)
        gcmd.respond_info("motor control startup retry scheduled")

    def cmd_MOTOR_QUERY_FAULTS(self, gcmd):
        try:
            start_time = self.reactor.monotonic()
            result = self.query_protection_status(
                axes=self.managed_axes, timeout=MOTOR_COMMAND_TIMEOUT
            )
            elapsed_ms = (self.reactor.monotonic() - start_time) * 1000.0
            lines = ["MOTOR_QUERY_FAULTS elapsed_ms=%.3f" % (elapsed_ms,)]
            for axis in self.managed_axes:
                detail = result[axis]
                err = int(detail["error_code"] or 0)
                warn = int(detail["warning_code"] or 0)
                frame_status = int(detail.get("frame_status") or 0)
                active = detail["active"]
                lines.append(
                    "%s: error=0x%08X  warning=0x%08X  frame_status=0x%02X  active=%s"
                    % (axis.upper(), err, warn, frame_status, active)
                )
            for line in lines:
                gcmd.respond_info(line)
            return {"axes": result, "elapsed_ms": elapsed_ms}
        except Exception as exc:
            raise gcmd.error(str(exc))


# ──────────────────────────────────────────────────────────────────────────
# motor_param_runtime
# ──────────────────────────────────────────────────────────────────────────

"""Runtime executor for motor param reads, writes, applies, and overrides.

This module defines the transport and compare semantics used by the live
motor-control stack for cfg-backed param application at startup.
"""


def _klog(msg, *args, level=logging.info):
    level("motor_param_runtime: " + msg, *args)


@dataclass
class OverrideAction:
    axis: str
    key: str
    param_id: int
    op: str
    override_value: object | None
    current_value: object | None
    wrote_value: object | None


class MotorParamExecutor:
    def __init__(self, registry: MotorParamRegistry):
        self.registry = registry

    def read_param(
        self, target, key: str, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        axis = target.axis
        param = self.registry.get(axis, key)
        try:
            result = target.sys_param_read(param, timeout=timeout)
            return result["value"]
        except RuntimeError as exc:
            if "no response" not in str(exc):
                raise
            raise RuntimeError(f"no response for {axis}.{key}") from exc

    def write_param(
        self, target, key: str, value, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        axis = target.axis
        param = self.registry.get(axis, key)
        result = target.sys_param_write(param, value, timeout=timeout)
        return result["response_value"]

    def apply_overrides(
        self, target, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> list[OverrideAction]:
        """Apply all override-active runtime params for an axis.

        Iterates every param where ``override_active=True``, reads the
        current board value, and writes + applies if there is a mismatch.
        If ``configurable=False``, a warning is emitted but the write is
        still attempted.
        """
        axis = target.axis
        actions: list[OverrideAction] = []
        changed_params = []
        changed = False
        for (ax, _), param in sorted(
            self.registry.by_axis_key.items(), key=lambda item: item[1].param_id
        ):
            if ax != axis or not param.override_active:
                continue
            target_value = coerce_override_value(param)
            if target_value is None:
                actions.append(
                    OverrideAction(
                        axis=axis,
                        key=param.key,
                        param_id=param.param_id,
                        op="skip_no_value",
                        override_value=None,
                        current_value=None,
                        wrote_value=None,
                    )
                )
                continue
            if not param.configurable:
                _klog(
                    "%s.%s is not configurable, attempting override anyway",
                    axis,
                    param.key,
                    level=logging.warning,
                )
            try:
                current = self.read_param(target, param.key, timeout=timeout)
            except Exception as exc:
                actions.append(
                    OverrideAction(
                        axis=axis,
                        key=param.key,
                        param_id=param.param_id,
                        op="read_error",
                        override_value=target_value,
                        current_value=None,
                        wrote_value=repr(exc),
                    )
                )
                continue
            if param_values_match(current, target_value, param):
                actions.append(
                    OverrideAction(
                        axis=axis,
                        key=param.key,
                        param_id=param.param_id,
                        op="read_ok",
                        override_value=target_value,
                        current_value=current,
                        wrote_value=None,
                    )
                )
                continue
            try:
                wrote = self.write_param(
                    target, param.key, target_value, timeout=timeout
                )
            except Exception as exc:
                actions.append(
                    OverrideAction(
                        axis=axis,
                        key=param.key,
                        param_id=param.param_id,
                        op="write_error",
                        override_value=target_value,
                        current_value=current,
                        wrote_value=repr(exc),
                    )
                )
                continue
            changed = True
            changed_params.append((param, target_value))
            actions.append(
                OverrideAction(
                    axis=axis,
                    key=param.key,
                    param_id=param.param_id,
                    op="write",
                    override_value=target_value,
                    current_value=current,
                    wrote_value=wrote,
                )
            )
        if changed:
            try:
                target.sys_param_apply(timeout=timeout)
            except Exception as exc:
                _klog(
                    "%s override apply ack issue: %s",
                    axis,
                    exc,
                    level=logging.warning,
                )
                actions.append(
                    OverrideAction(
                        axis=axis,
                        key="__apply__",
                        param_id=-1,
                        op="apply_error",
                        override_value=None,
                        current_value=None,
                        wrote_value=repr(exc),
                    )
                )
            for param, target_value in changed_params:
                try:
                    verified = self.read_param(
                        target, param.key, timeout=timeout
                    )
                except Exception as exc:
                    actions.append(
                        OverrideAction(
                            axis=axis,
                            key=param.key,
                            param_id=param.param_id,
                            op="verify_error",
                            override_value=target_value,
                            current_value=None,
                            wrote_value=repr(exc),
                        )
                    )
                    continue
                if not param_values_match(verified, target_value, param):
                    actions.append(
                        OverrideAction(
                            axis=axis,
                            key=param.key,
                            param_id=param.param_id,
                            op="verify_mismatch",
                            override_value=target_value,
                            current_value=verified,
                            wrote_value=None,
                        )
                    )
        return actions

    def read_calibration_params(
        self, target, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> list[dict]:
        """Read calibration params (0x09, 0x19) and warn if near-zero.

        These calibration IDs are selected by ``CALIBRATION_PARAM_IDS``.
        Their map reference values are unknown until readback. We populate
        the runtime ``value`` field and warn if any are near zero; constructor
        zeroes are not evidence that a motor has been calibrated.
        """
        axis = target.axis
        results = []
        for param_id in CALIBRATION_PARAM_IDS:
            param = self.registry.get_by_id(axis, param_id)
            try:
                value = self.read_param(target, param.key, timeout=timeout)
            except Exception as exc:
                _klog(
                    "%s.%s calibration read failed: %s",
                    axis,
                    param.key,
                    exc,
                    level=logging.warning,
                )
                results.append(
                    calibration_param_result(
                        axis, param, param_id, error=repr(exc)
                    )
                )
                continue
            param.value = value
            result = calibration_param_result(axis, param, param_id, value)
            near_zero = result["near_zero"]
            if near_zero:
                _klog(
                    "%s.%s calibration value is near-zero (%.6f). "
                    "DO NOT use this motor until calibration is completed; "
                    "motion may behave erratically. Run MOTOR_CALIBRATE AXIS=%s",
                    axis,
                    param.key,
                    float(value),
                    axis.upper(),
                    level=logging.warning,
                )
            results.append(result)
        return results


# ──────────────────────────────────────────────────────────────────────────
# nozzle_mcu_transport
# ──────────────────────────────────────────────────────────────────────────

"""Transport adapter for extruder motor packets over nozzle MCU transparent I/O.

Registers the transparent command pair on `nozzle_mcu` and exposes a `send()`
method for full motor firmware packets.
"""


# ──────────────────────────────────────────────────────────────────────────
# serial485_transport
# ──────────────────────────────────────────────────────────────────────────


class Serial485TransportAdapter:
    def __init__(self, serial485_obj):
        self.serial485 = serial485_obj

    @classmethod
    def from_printer(cls, printer):
        return cls(printer.lookup_object("serial_485 serial485"))

    def is_configured(self) -> bool:
        return bool(self.serial485._status_fields()["connected"])

    def send(
        self,
        packet: bytes,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
        attempts: int = 1,
        response_timeout: float | None = None,
    ) -> bytes | None:
        wait_timeout = timeout if response_timeout is None else response_timeout
        return self.serial485.cmd_send_data_with_response(
            packet, wait_timeout, attempts=attempts
        )


# ──────────────────────────────────────────────────────────────────────────
# motor_axis_target
# ──────────────────────────────────────────────────────────────────────────

"""Axis-target routing for the shared motor firmware command set."""


# The Hi has closed-loop drivers on X and Y only. EXTRUDER_AXIS is kept as a
# name so the shared helpers below still read like the K2 module, but no
# extruder axis is ever managed.
KINEMATIC_AXES = ("x", "y")
EXTRUDER_AXIS = "e"
EXTRUDER_AXES = ()
ALL_AXES = KINEMATIC_AXES
AXIS_NUM_MAP = {
    1: "x",
    2: "y",
}
AXIS_TO_NUM_MAP = {axis: num for num, axis in AXIS_NUM_MAP.items()}
EXTRUDER_BOOTSTRAP_ADDR = 0x81
EXTRUDER_MIN_RUNTIME_ADDR = 0x81
EXTRUDER_MAX_RUNTIME_ADDR = 0x8F
STAGED_CALIBRATION_AXES = EXTRUDER_AXES
RUNTIME_FAULT_ACTION_SHUTDOWN = "shutdown"
RUNTIME_FAULT_ACTION_RECOVER = "recover"
RUNTIME_FAULT_ACTION_BY_AXIS = {
    **{axis: RUNTIME_FAULT_ACTION_SHUTDOWN for axis in KINEMATIC_AXES},
}


@dataclass(frozen=True)
class MotorAxisTarget:
    axis: str
    addr: int
    client: MotorFirmwareClient

    def _with_axis(self, result: dict, **extra) -> dict:
        return {**result, "axis": self.axis, **extra}

    def sys_param_read(
        self, param, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.sys_param_read(
                self.addr,
                param.param_id,
                value_type=param.value_type,
                param_key=param.key,
                timeout=timeout,
            )
        )

    def sys_param_write(
        self, param, value, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.sys_param_write(
                self.addr,
                param.param_id,
                value,
                value_type=param.value_type,
                param_key=param.key,
                timeout=timeout,
            )
        )

    def sys_param_apply(self, timeout: float = MOTOR_COMMAND_TIMEOUT) -> dict:
        return self._with_axis(
            self.client.sys_param_apply(self.addr, timeout=timeout)
        )

    def flash_param_read(
        self, param_id: int, param=None, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.flash_param_read(
                self.addr,
                int(param_id),
                value_type=None if param is None else param.value_type,
                param_key=None if param is None else param.key,
                timeout=timeout,
            )
        )

    def protection(
        self, data: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.protection(self.addr, data=data, timeout=timeout)
        )

    def clear_err_warn_code(
        self, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ) -> dict:
        result = self.client.protection_clear(self.addr, timeout=timeout)
        return self._with_axis(result, data=DATA_CLEAR_ERR_WARN)

    def set_stall_mode(
        self, mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.set_stall_mode(self.addr, mode, timeout=timeout)
        )

    def get_stall_mode_raw(
        self, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return self._with_axis(
            self.client.get_stall_mode_raw(self.addr, timeout=timeout)
        )

    def set_stall_mode_raw(
        self, raw_mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        value = int(raw_mode) & 0xFF
        result = self.client.set_stall_mode(self.addr, value, timeout=timeout)
        return self._with_axis(
            result, raw_mode=value, ack_mode=result.get("mode")
        )


@dataclass
class AxisApplyResult:
    axis: str
    checked: int
    read_ok: int
    skipped: int
    writes: int
    errors: int
    actions: list[dict]


class MotorAxisController:
    def __init__(
        self,
        registry: MotorParamRegistry,
        *,
        serial_transport,
        extruder_transport,
        kinematic_axes=KINEMATIC_AXES,
    ):
        self.registry = registry
        self.kinematic_axes = tuple(kinematic_axes)
        self.all_axes = self.kinematic_axes + EXTRUDER_AXES
        self.executor = MotorParamExecutor(registry)
        self.serial_transport = serial_transport
        self.extruder_transport = extruder_transport
        self.serial_client = MotorFirmwareClient(serial_transport, framed=False)
        if extruder_transport is not None:
            raise RuntimeError("the Creality Hi has no closed-loop extruder")
        self.extruder_client = None
        self.extruder_addr = None
        self._serial_targets = {
            axis: self._make_serial_target(axis) for axis in self.kinematic_axes
        }
        self._extruder_target = None

    def _make_serial_target(self, axis: str) -> MotorAxisTarget:
        return MotorAxisTarget(
            axis=axis,
            addr=self.registry.get_addr(axis),
            client=self.serial_client,
        )

    def target(self, axis: str) -> MotorAxisTarget:
        axis = str(axis).strip().lower()
        if axis in self._serial_targets:
            return self._serial_targets[axis]
        if axis in KINEMATIC_AXES:
            raise RuntimeError(
                f"motor axis {axis!r} is not listed in [motor_control] motor_axes"
            )
        raise RuntimeError(f"unsupported motor axis {axis!r}")

    def apply_overrides(
        self, axis: str, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> AxisApplyResult:
        target = self.target(axis)
        actions = self.executor.apply_overrides(target, timeout=timeout)
        action_dicts = [asdict(action) for action in actions]
        return AxisApplyResult(
            axis=target.axis,
            checked=sum(
                1 for action in action_dicts if action["param_id"] >= 0
            ),
            read_ok=sum(
                1 for action in action_dicts if action["op"] == "read_ok"
            ),
            skipped=sum(
                1 for action in action_dicts if action["op"].startswith("skip")
            ),
            writes=sum(1 for action in action_dicts if action["op"] == "write"),
            errors=sum(
                1
                for action in action_dicts
                if action["op"]
                in (
                    "read_error",
                    "write_error",
                    "verify_error",
                    "verify_mismatch",
                )
            ),
            actions=action_dicts,
        )

    def read_calibration_params(
        self, axis: str, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> list[dict]:
        target = self.target(axis)
        return self.executor.read_calibration_params(target, timeout=timeout)

    def prepare_serial_bus(self, timeout: float = MOTOR_NO_ACK_TIMEOUT) -> dict:
        return {
            **self.serial_client.startup_prepare_probe_bus(timeout=timeout),
            "transport": "serial485",
            "probe_addr": 0xFF,
        }

    def probe_serial_axes(self, timeout: float = MOTOR_COMMAND_TIMEOUT) -> dict:
        results = {}
        for addr in sorted(
            target.addr for target in self._serial_targets.values()
        ):
            results[f"0x{addr:02X}"] = self.serial_client.startup_probe_addr(
                addr, timeout=timeout
            )
        return results

    def set_homing_stall_mode(
        self, mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        if mode not in (1, 2):
            raise RuntimeError("XY stall mode must be 1 or 2")
        return {
            "mode": f"stall{mode}",
            "packets": [
                self.target(axis).set_stall_mode(mode, timeout=timeout)
                for axis in ("x", "y")
            ],
        }

    def set_normal_stall_mode(
        self, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return {
            "mode": "normal_stall",
            "axes": self.all_axes,
            "packets": [
                self.target(axis).set_stall_mode(0x02, timeout=timeout)
                for axis in self.all_axes
            ],
        }

    def get_axis_stall_mode_raw(
        self, axis: str, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        axis = str(axis).strip().lower()
        if axis != "x":
            raise RuntimeError(
                f"raw cut-calibration stall mode is only verified for axis {axis!r}"
            )
        return self.target(axis).get_stall_mode_raw(timeout=timeout)

    def set_axis_stall_mode_raw(
        self, axis: str, raw_mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        axis = str(axis).strip().lower()
        if axis != "x":
            raise RuntimeError(
                f"raw cut-calibration stall mode is only verified for axis {axis!r}"
            )
        return self.target(axis).set_stall_mode_raw(raw_mode, timeout=timeout)

    def check_protection(
        self, axes, data: int = 11, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        return {
            axis: self.target(axis).protection(data=data, timeout=timeout)
            for axis in axes
        }

    def clear_fault_latches(
        self,
        axes,
        data: int = DATA_CLEAR_ERR_WARN,
        timeout: float = MOTOR_NO_ACK_TIMEOUT,
    ) -> dict:
        if int(data) != DATA_CLEAR_ERR_WARN:
            raise RuntimeError(
                f"unsupported protection clear data=0x{int(data) & 0xFF:02X}"
            )
        return {
            axis: self.target(axis).clear_err_warn_code(timeout=timeout)
            for axis in axes
        }


# ──────────────────────────────────────────────────────────────────────────
# motor_control_config_model
# ──────────────────────────────────────────────────────────────────────────

"""Configuration model for the `[motor_control]` section.

Combines the K2 pin layout with cutter calibration and firmware parameter
overrides used by startup, fault handling, and command execution.
"""


# Motor ids match Kalico's [stepper_*] sections, which own the STEP/DIR pins.
# XY are always closed-loop motors; Z/Z1 are optional (a K2 Pro drives Z
# with a plain stepper driver and has no Z motors on the bus).
REQUIRED_MOTOR_AXES = ("x", "y")

# Stall pins are read from the [motor_control_hi] section (stock Hi: PA8 for
# X and PA6 for Y). There is no built-in layout.
STALL_PIN_OPTIONS = ("motor_x_stall", "motor_y_stall")


def output_pin_options(axes) -> tuple[str, ...]:
    return tuple(
        f"motor_{axis}_{kind}" for axis in axes for kind in ("dir", "step")
    )


OUTPUT_PIN_OPTIONS = output_pin_options(KINEMATIC_AXES)

PIN_OPTIONS = OUTPUT_PIN_OPTIONS + STALL_PIN_OPTIONS


@dataclass(frozen=True)
class MotorPinConfig:
    raw: str
    pin_desc: str


@dataclass(frozen=True)
class MotorControlConfigModel:
    raw_options: dict[str, str | None]
    cut_pos_offset: float
    pins: dict[str, MotorPinConfig]
    motor_axes: tuple[str, ...] = KINEMATIC_AXES
    mcu_temperature_sensors: tuple[str, ...] = KINEMATIC_AXES + EXTRUDER_AXES

    @classmethod
    def from_config(cls, config, param_options=()):
        accepted = {
            "cut_pos_offset",
            *(option.lower() for option in PIN_OPTIONS),
            *(str(option).lower() for option in param_options),
        }
        raw = {
            option: config.get(option)
            for option in config.fileconfig.options(config.section)
            if option.lower() in accepted
        }
        motor_axes = cls._parse_motor_axes(config)
        managed_axes = motor_axes + EXTRUDER_AXES
        pins = {}
        for axis in managed_axes:
            option = f"motor_{axis}_stall"
            pin_desc = config.get(option)
            pins[option] = MotorPinConfig(raw=pin_desc, pin_desc=pin_desc)
        for axis in motor_axes:
            stepper = config.getsection("stepper_" + axis)
            for kind in ("dir", "step"):
                pin_desc = stepper.get(kind + "_pin")
                pins[f"motor_{axis}_{kind}"] = MotorPinConfig(
                    raw=pin_desc, pin_desc=pin_desc
                )
        sensor_option = (
            config.get("mcu_temperature_sensors", ", ".join(managed_axes))
            .strip()
            .lower()
        )
        sensor_axes = (
            ()
            if sensor_option in ("", "none")
            else tuple(axis.strip() for axis in sensor_option.split(","))
        )
        if any(axis not in managed_axes for axis in sensor_axes):
            raise config.error(
                "mcu_temperature_sensors must be a comma-separated list of "
                "%s, or none" % (", ".join(managed_axes),)
            )
        return cls(
            raw_options=raw,
            cut_pos_offset=config.getfloat("cut_pos_offset", 0.4),
            pins=pins,
            motor_axes=motor_axes,
            mcu_temperature_sensors=tuple(dict.fromkeys(sensor_axes)),
        )

    @staticmethod
    def _parse_motor_axes(config) -> tuple[str, ...]:
        raw = config.get("motor_axes", ", ".join(KINEMATIC_AXES))
        axes = {axis.strip() for axis in raw.lower().split(",") if axis.strip()}
        if any(axis not in KINEMATIC_AXES for axis in axes) or any(
            axis not in axes for axis in REQUIRED_MOTOR_AXES
        ):
            raise config.error("motor_axes must be x, y on the Creality Hi")
        return tuple(axis for axis in KINEMATIC_AXES if axis in axes)


# ──────────────────────────────────────────────────────────────────────────
# motor_stall_monitor
# ──────────────────────────────────────────────────────────────────────────

"""Stall input monitor for motor control.

Registers configured stall pins through Klipper's button framework and forwards
state changes to the motor-control runtime.
"""


STALL_AXIS_PINS = (
    ("x", "motor_x_stall"),
    ("y", "motor_y_stall"),
)


class MotorStallMonitor:
    def __init__(self, config, replacement, config_model):
        self.printer = config.get_printer()
        self.replacement = replacement
        self.buttons = self.printer.load_object(config, "buttons")
        self.states = {axis: None for axis, _ in STALL_AXIS_PINS}
        self.pin_map = {
            axis: config_model.pins.get(option)
            for axis, option in STALL_AXIS_PINS
        }
        self._main_axes = []
        self._nozzle_axes = []

    def initialize(self):
        for axis in KINEMATIC_AXES:
            pin_cfg = self.pin_map.get(axis)
            if pin_cfg is not None:
                self._main_axes.append((axis, pin_cfg.raw))
        pin_cfg = self.pin_map.get(EXTRUDER_AXIS)
        if pin_cfg is not None:
            self._nozzle_axes.append((EXTRUDER_AXIS, pin_cfg.raw))
        if self._main_axes:
            self.buttons.register_buttons(
                [pin for _axis, pin in self._main_axes],
                functools.partial(self._dispatch_group, axes=self._main_axes),
            )
        if self._nozzle_axes:
            self.buttons.register_buttons(
                [pin for _axis, pin in self._nozzle_axes],
                functools.partial(self._dispatch_group, axes=self._nozzle_axes),
            )

    def _dispatch_group(self, eventtime, state, axes):
        for i, (axis, _pin) in enumerate(axes):
            active = 1 if (state & (1 << i)) else 0
            prev = self.states[axis]
            if prev == active:
                continue
            self.states[axis] = active
            self.replacement.note_stall_pin_event(
                axis=axis, active=active, eventtime=eventtime
            )

    def read_all(self):
        return dict(self.states)


# ──────────────────────────────────────────────────────────────────────────
# motor_temp_sensors
# ──────────────────────────────────────────────────────────────────────────

"""Expose MOT2 MCU temperatures as Fluidd-visible sensors."""


GET_MCU_TEMP_INDEX = 17
POLL_INTERVAL = 6.0
POLL_TIMEOUT = 0.25


class Mot2AxisTempSensor:
    def __init__(self):
        self.temperature = 0.0
        self.measured_min = 99999999.0
        self.measured_max = 0.0

    def note(self, temp: float):
        self.temperature = float(temp)
        if temp:
            self.measured_min = min(self.measured_min, self.temperature)
            self.measured_max = max(self.measured_max, self.temperature)

    def get_status(self, _eventtime):
        return {
            "temperature": round(self.temperature, 2),
            "measured_min_temp": round(self.measured_min, 2),
            "measured_max_temp": round(self.measured_max, 2),
        }


class Mot2TempSensorHub:
    def __init__(self, replacement, axes=ALL_AXES):
        self.replacement = replacement
        self.reactor = replacement.reactor
        self.axes = tuple(axes)
        self.sensors = {}
        for axis in self.axes:
            sensor = Mot2AxisTempSensor()
            name = "temperature_sensor motor_%s_MCU" % (axis.upper(),)
            replacement.printer.add_object(name, sensor)
            self.sensors[axis] = sensor
        self._timer = (
            self.reactor.register_timer(self._poll) if self.axes else None
        )
        self._started = False
        self._axis_index = 0

    def start(self):
        if not self.axes:
            return
        self._started = True
        self.reactor.update_timer(self._timer, self.reactor.monotonic())

    def stop(self):
        self._started = False
        if self._timer is not None:
            self.reactor.update_timer(self._timer, self.reactor.NEVER)

    def _poll(self, _eventtime):
        if not self._started:
            return self.reactor.NEVER
        if self.replacement.is_ready and self.replacement.motor_params_init:
            axis = self.axes[self._axis_index]
            self._axis_index = (self._axis_index + 1) % len(self.axes)
            try:
                target = self.replacement.axes.target(axis)
                self.sensors[axis].note(
                    target.client.get_value(
                        target.addr,
                        GET_MCU_TEMP_INDEX,
                        timeout=POLL_TIMEOUT,
                        attempts=1,
                    )
                )
            except Exception:
                pass
        return self.reactor.monotonic() + POLL_INTERVAL


# ──────────────────────────────────────────────────────────────────────────
# motor_pin_manager
# ──────────────────────────────────────────────────────────────────────────

"""Mainboard motor pin-state manager.

Drives STEP/DIR address straps for the stock Creality MCU during discovery.
"""


# Physical levels. Each driver latches its RS-485 address from them:
# addr = 0x81 + DIR + 2 * STEP. Pins also start at these levels.
# Physical STEP/DIR levels while the drivers latch their address:
# addr = 0x81 + DIR + 2 * STEP. Stock Hi addresses: X = 0x81, Y = 0x82
# (the K2 module assigns them the other way round).
PIN_DIR_SEQUENCE = (
    ("motor_x_dir", 0),
    ("motor_x_step", 0),
    ("motor_y_dir", 1),
    ("motor_y_step", 0),
)


def _shareable_pin_desc(pin_desc: str) -> str:
    desc = pin_desc.strip()
    while desc and desc[0] in "!^~":
        desc = desc[1:].strip()
    return desc


class MotorPinManager:
    def __init__(self, printer, config_model: MotorControlConfigModel):
        self.printer = printer
        self.config_model = config_model
        self.reactor = printer.get_reactor()
        self.mcu = printer.lookup_object("mcu")
        self.ppins = printer.lookup_object("pins")
        # Only managed motors are strapped; their levels don't depend on
        # whether the other motors exist.
        options = set(output_pin_options(config_model.motor_axes))
        self.dir_sequence = tuple(
            (option, level)
            for option, level in PIN_DIR_SEQUENCE
            if option in options
        )
        self.normal_sequence = tuple(
            (option, 0) for option, _level in self.dir_sequence
        )
        self.outputs = {}
        self._inverted = {}
        self._init_outputs()

    def _init_outputs(self):
        for option, start_level in self.dir_sequence:
            pin_cfg = self.config_model.pins.get(option)
            if pin_cfg is None:
                raise RuntimeError(
                    f"missing required output pin config for {option}"
                )
            self.ppins.allow_multi_use_pin(
                _shareable_pin_desc(pin_cfg.pin_desc)
            )
            pin = self.ppins.setup_pin("digital_out", pin_cfg.pin_desc)
            # Klipper writes logical values; undo the [stepper_*] pin's "!".
            inverted = self.ppins.parse_pin(pin_cfg.pin_desc, can_invert=True)[
                "invert"
            ]
            initial = start_level ^ inverted
            pin.setup_max_duration(0.0)
            pin.setup_start_value(initial, initial)
            self.outputs[option] = pin
            self._inverted[option] = inverted

    def _schedule_sequence(self, sequence: Iterable[tuple[str, int]]):
        start = self.mcu.estimated_print_time(self.reactor.monotonic()) + 0.05
        for idx, (attr, level) in enumerate(sequence):
            pin = self.outputs[attr]
            when = start + idx * 0.003
            pin.set_digital(when, level ^ self._inverted[attr])
        # set_digital queues future writes. Do not broadcast the address-latch
        # request (or leave cleanup) before the final write's scheduled time.
        deadline = when + 0.010
        while True:
            now = self.reactor.monotonic()
            remaining = deadline - self.mcu.estimated_print_time(now)
            if remaining <= 0.0:
                break
            self.reactor.pause(now + remaining)

    def set_motor_pin_dir(self):
        return self._schedule_sequence(self.dir_sequence)

    def set_motor_pin_normal(self):
        return self._schedule_sequence(self.normal_sequence)


# ──────────────────────────────────────────────────────────────────────────
# motor_control
# ──────────────────────────────────────────────────────────────────────────

"""Host-side motor control for startup, parameters, calibration, and faults.

Architecture:
- config model: `MotorControlConfigModel`
- host-side address pin orchestration: `MotorPinManager`
- RS485 transport: `Serial485TransportAdapter`
- firmware axis routing: `MotorAxisController`
"""


def _klog(msg, *args, level=logging.info):
    level("motor_control: " + msg, *args)


DEFAULT_REGISTRY_PATH = Path(__file__).parent / "motor_map_hi.json"
STARTUP_AUTO_RETRY_DELAY = 2.0
STARTUP_AUTO_RETRY_LIMIT = 3
STALL_EVENT_MIN_INTERVAL = 0.100
FAULT_CLEANUP_RETRY_DELAY = 0.100
PROTECTION_POLL_INTERVAL = 60.0
CALIBRATION_STAGE_ENCODER = "encoder"
CALIBRATION_STAGE_OFFSET = "offset"
# Stage 1 completes only after firmware observes 0xC0 and returns 0x0C.
EXTRUDER_ENCODER_STAGE1_COMPLETE_CODE = 0x0C
CALIBRATION_STAGE_ALIAS_MAP = {
    "1": CALIBRATION_STAGE_ENCODER,
    "2": CALIBRATION_STAGE_OFFSET,
    CALIBRATION_STAGE_ENCODER: CALIBRATION_STAGE_ENCODER,
    CALIBRATION_STAGE_OFFSET: CALIBRATION_STAGE_OFFSET,
}
STARTUP_STEP_ACTIONS = {
    "stall_mode": "configuring motor stall modes",
    "protect_check": "checking motor protection status",
    "protect_clear": "clearing motor protection faults",
    "protect_recheck": "rechecking motor protection status",
    "apply_overrides": "applying motor configuration",
}
STARTUP_STEP_FUNCTIONS = {
    "serial_target_discovery": 0x0E,
    "extruder_target_discovery": 0x0E,
    "stall_mode": 0x11,
    "protect_check": 0x0C,
    "protect_clear": 0x0C,
    "protect_recheck": 0x0C,
}
STARTUP_SERIAL_AXIS_BY_ADDR = {
    0x81: "Y",
    0x82: "X",
    0x83: "Z",
    0x84: "Z1",
}
STARTUP_PROTOCOL_ERROR_RE = re.compile(
    r"^(no response|empty response) for addr=0x([0-9a-fA-F]{2}) "
    r"func=0x([0-9a-fA-F]{2})$"
)


class MotorControl(MotorControlDebugSurfaceMixin):
    def __init__(
        self, config, registry_path: str | Path = DEFAULT_REGISTRY_PATH
    ):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.pause_resume = self.printer.load_object(config, "pause_resume")
        self.write_level = config.getchoice(
            "startup_writes", WRITE_LEVEL_CHOICES, "none"
        )
        self.registry = MotorParamRegistry.from_file(registry_path)
        self.config_model = MotorControlConfigModel.from_config(
            config,
            (param.cfg_key for param in self.registry.by_axis_key.values()),
        )
        self._runtime_cfg_overrides = self.registry.apply_config_overrides(
            self.config_model.raw_options
        )
        self.kinematic_axes = self.config_model.motor_axes
        self.managed_axes = self.kinematic_axes + EXTRUDER_AXES
        self.transport = Serial485TransportAdapter.from_printer(self.printer)
        self.extruder_transport = None
        self.axes = MotorAxisController(
            self.registry,
            serial_transport=self.transport,
            extruder_transport=None,
            kinematic_axes=self.kinematic_axes,
        )
        self.axes.serial_client.write_level = self.write_level
        self.pin_manager = MotorPinManager(self.printer, self.config_model)
        self.stall_monitor = MotorStallMonitor(
            config, replacement=self, config_model=self.config_model
        )
        if self._runtime_cfg_overrides:
            _klog(
                "loaded %d live cfg param override(s) into runtime registry",
                len(self._runtime_cfg_overrides),
            )

        self.is_homing = False
        self.is_ready = False
        self.motor_params_init = False
        self.auto_retry = True
        self.cut_pos_offset = self.config_model.cut_pos_offset
        self.position_min_cut_x = -10.0
        self.cut_state = False
        self.is_check_cut_pos_start = False
        self.motor_error_code = {}
        self.motor_warning_code = {}
        self.motor_fault_detail = {}
        for num in range(1, 6):
            self.motor_error_code[num] = 0
            self.motor_error_code[str(num)] = 0
            self.motor_warning_code[num] = 0
            self.motor_warning_code[str(num)] = 0
        self._stall_last_event = {axis: 0.0 for axis in AXIS_TO_NUM_MAP}
        self._homing_move_depth = 0
        self._fault_cleanup_pending = False
        self._fault_cleanup_axes = set()
        self._fault_cleanup_homing_fault = False
        self._fault_cleanup_startup_fault = False
        self._kinematic_calibration_confirm_axes = None
        self._fault_cleanup_timer = self.reactor.register_timer(
            self._fault_cleanup_handler
        )
        self._protection_poll_timer = self.reactor.register_timer(
            self._protection_poll_handler, self.reactor.NEVER
        )

        self._startup_error = ""
        self._startup_started = False
        self._startup_complete = False
        self._startup_step_index = 0
        self._startup_fault_clear_ok = None
        self._startup_fault_clear_result = ""
        self._startup_protection_active = {}
        self._startup_auto_retry_count = 0
        self._startup_allow_auto_retry = True
        self._startup_timer = self.reactor.register_timer(self._startup_handler)
        self.temp_sensors = Mot2TempSensorHub(
            self, self.config_model.mcu_temperature_sensors
        )

        for axis, pin_cfg in self.stall_monitor.pin_map.items():
            if pin_cfg is not None:
                setattr(self, f"motor_{axis}_stall", pin_cfg.raw)
        self.printer.register_event_handler(
            "homing:homing_move_begin", self._handle_homing_move_begin
        )
        self.printer.register_event_handler(
            "homing:homing_move_end", self._handle_homing_move_end
        )
        self.printer.register_event_handler(
            "gcode:command_error", self._handle_gcode_command_error
        )
        self.printer.register_event_handler(
            "klippy:shutdown", self._handle_shutdown
        )
        self.printer.register_event_handler(
            "serial_485:ready", self._handle_serial485_ready
        )
        self.stall_monitor.initialize()

        self._register_commands()

    def _register_commands(self):
        self._register_runtime_commands()
        self._register_calibration_commands()
        self._register_debug_commands()

    def _register_command_specs(self, specs):
        for name, handler, desc in specs:
            self.gcode.register_command(name, handler, desc=desc)

    def _register_runtime_commands(self):
        self._register_command_specs(
            (
                (
                    "MOTOR_STATUS",
                    self.cmd_MOTOR_STATUS,
                    "MOTOR_STATUS [VERBOSE=1] Print readable motor-control state",
                ),
                (
                    "MOTOR_CLEAR_ERROR",
                    self.cmd_MOTOR_CLEAR_ERROR,
                    "MOTOR_CLEAR_ERROR Query active faults, clear them, and report the result",
                ),
                (
                    "MOTOR_READ_PARAM",
                    self.cmd_MOTOR_READ_PARAM,
                    "MOTOR_READ_PARAM PARAM=x_param_stall_cur_A",
                ),
            )
        )

    def _register_calibration_commands(self):
        # Calibration and flash commands write to the drivers: they are not
        # registered on the Hi, and the write gate refuses their frames.
        return

    def _register_debug_commands(self):
        self._register_command_specs(
            (
                (
                    "MOTOR_QUERY_FAULTS",
                    self.cmd_MOTOR_QUERY_FAULTS,
                    "MOTOR_QUERY_FAULTS Live query and print error/warning/status codes for all axes with elapsed time",
                ),
                (
                    "MOTOR_CFG_OVERRIDE_STATUS",
                    self.cmd_MOTOR_CFG_OVERRIDE_STATUS,
                    "MOTOR_CFG_OVERRIDE_STATUS [AXIS=%s] [DETAIL=raw] Report live cfg-backed targets and current board values"
                    % (self._format_axis_labels(self.managed_axes),),
                ),
                (
                    "MOTOR_RETRY_STARTUP",
                    self.cmd_MOTOR_RETRY_STARTUP,
                    "MOTOR_RETRY_STARTUP Restart motor-control startup",
                ),
            )
        )

    def _axis_from_num(self, num: int) -> str:
        axis = AXIS_NUM_MAP.get(num)
        if axis is None:
            raise RuntimeError(f"unsupported motor num={num}")
        return axis

    def _coerce_value(self, param, raw_value):
        value_type = param.value_type
        if value_type == "float":
            return float(raw_value)
        if value_type in ("uint16_t", "int16_t", "uint32_t", "int32_t"):
            return int(float(raw_value))
        raise RuntimeError(f"unsupported value type {value_type}")

    @staticmethod
    def _parse_axis_spec(raw_value: str | None) -> tuple[str, ...]:
        if raw_value is None:
            raise RuntimeError("AXIS is required")
        compact = "".join(
            ch for ch in raw_value.strip().lower() if ch.isalnum()
        )
        if not compact:
            raise RuntimeError("AXIS is required")
        axes = []
        i = 0
        while i < len(compact):
            if compact.startswith("z1", i):
                axis = "z1"
                i += 2
            else:
                axis = compact[i]
                i += 1
            if axis not in AXIS_TO_NUM_MAP:
                raise RuntimeError(
                    f"unsupported AXIS={raw_value!r}; use X,Y,Z,Z1,E"
                )
            if axis not in axes:
                axes.append(axis)
        return tuple(axes)

    def _parse_managed_axis_spec(
        self, raw_value: str | None
    ) -> tuple[str, ...]:
        axes = self._parse_axis_spec(raw_value)
        unmanaged = [axis for axis in axes if axis not in self.managed_axes]
        if unmanaged:
            raise RuntimeError(
                "AXIS=%s is not listed in [motor_control] motor_axes"
                % (self._format_axis_labels(unmanaged),)
            )
        return axes

    @staticmethod
    def _format_axis_label(axis: str) -> str:
        return "Z1" if axis == "z1" else axis.upper()

    def _format_axis_labels(self, axes) -> str:
        return "".join(self._format_axis_label(axis) for axis in axes)

    def _lookup_param_by_cfg_key(self, name: str):
        needle = str(name or "").strip().lower()
        if not needle:
            raise RuntimeError("PARAM is required")
        matches = []
        for param in self.registry.by_axis_key.values():
            cfg_key = str(param.cfg_key).strip().lower()
            if cfg_key == needle:
                matches.append(param)
        if not matches:
            for param in self.registry.by_axis_key.values():
                if str(param.key).strip().lower() == needle:
                    matches.append(param)
        if not matches:
            raise RuntimeError(f"unknown motor PARAM={name!r}")
        if len(matches) > 1:
            labels = ", ".join(
                f"{self._format_axis_label(param.axis)}:{param.cfg_key}"
                for param in matches
            )
            raise RuntimeError(
                f"ambiguous motor PARAM={name!r}; matches {labels}"
            )
        return matches[0]

    def _resolve_param_command_target(self, gcmd):
        param = self._lookup_param_by_cfg_key(gcmd.get("PARAM"))
        label = param.cfg_key
        return param, label

    @staticmethod
    def _warn_if_param_not_configurable(gcmd, param, label: str):
        if not param.configurable:
            gcmd.respond_info(
                "Warning: %s is not marked configurable, attempting anyway"
                % label
            )

    @staticmethod
    def _gcmd_raw_detail(gcmd) -> bool:
        detail = (gcmd.get("DETAIL", "") or "").strip().lower()
        if detail in ("raw", "verbose", "debug", "1", "true", "yes", "on"):
            return True
        verbose = gcmd.get("VERBOSE", None)
        if verbose is None:
            return False
        return int(float(verbose)) != 0

    @staticmethod
    def _format_decimal(value, digits: int = 2) -> str:
        if value is None:
            return "n/a"
        return f"{float(value):.{digits}f}"

    @staticmethod
    def _format_readback_value(value) -> str:
        if value is None:
            return "n/a"
        if isinstance(value, bool):
            return str(value)
        if isinstance(value, int):
            return str(value)
        return repr(float(value))

    def _format_runtime_apply_result_line(self, result: dict) -> str:
        live_value = self._format_readback_value(result["live_readback_value"])
        if result["live_matches"]:
            return "Runtime apply verified by readback Live readback=%s" % (
                live_value,
            )
        suffix = " Expected=%s" % (
            self._format_readback_value(result["written_value"]),
        )
        if result.get("live_readback_error"):
            suffix = " Readback issue=%s" % (result["live_readback_error"],)
        return "Warning: Runtime apply failed Live readback=%s%s" % (
            live_value,
            suffix,
        )

    def _format_flash_write_result_line(self, result: dict) -> str:
        flash_value = self._format_readback_value(
            result["flash_readback_value"]
        )
        if result.get("flash_matches"):
            return "Flash write verified by readback Flash readback=%s" % (
                flash_value,
            )
        suffix = " Expected=%s" % (
            self._format_readback_value(result["written_value"]),
        )
        flash_readback = result.get("flash_readback")
        if isinstance(flash_readback, dict) and flash_readback.get("error"):
            suffix = " Readback issue=%s" % (flash_readback["error"],)
        return "Warning: Flash write failed Flash readback=%s%s" % (
            flash_value,
            suffix,
        )

    def _format_unsafe_calibration_warnings(
        self, entries: list[dict]
    ) -> list[str]:
        kinematic_axes = []
        has_extruder = False
        for entry in entries:
            axis = str(entry.get("axis", "?"))
            if axis in KINEMATIC_AXES:
                if axis not in kinematic_axes:
                    kinematic_axes.append(axis)
                continue
            if axis in EXTRUDER_AXES:
                has_extruder = True
        warnings = []
        ordered_kinematic_axes = tuple(
            axis for axis in KINEMATIC_AXES if axis in kinematic_axes
        )
        if ordered_kinematic_axes:
            labels = [
                self._format_axis_label(axis) for axis in ordered_kinematic_axes
            ]
            label_text = "/".join(labels)
            command_axes = "".join(labels)
            subject, verb = (
                ("Motor", "is") if len(labels) == 1 else ("Motors", "are")
            )
            warnings.append(
                "%s %s %s not calibrated. Calibrate with "
                "MOTOR_CALIBRATE AXIS=%s."
                % (subject, label_text, verb, command_axes)
            )
        if has_extruder:
            warnings.append(
                "Motor E is not calibrated. Calibrate with MOTOR_CALIBRATE "
                "AXIS=E STAGE=encoder, power cycle, then MOTOR_CALIBRATE "
                "AXIS=E STAGE=offset."
            )
        return warnings

    @staticmethod
    def _format_unsafe_calibration_threshold(entry: dict) -> str:
        threshold = entry.get("near_zero_threshold")
        if threshold is None:
            return "near zero"
        return "at/below %.3f" % (float(threshold),)

    @staticmethod
    def _calibration_snapshot_value(snapshot: dict, key: str):
        return snapshot.get("values", {}).get(key, {}).get("value")

    def _flash_param_read(
        self, axis: str, param_id: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        target = self.axes.target(axis)
        try:
            param = self.registry.get_by_id(target.axis, int(param_id))
        except KeyError:
            param = None
        return target.flash_param_read(param_id, param=param, timeout=timeout)

    def _read_live_param(
        self, param, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        axis = param.axis
        axis_label = self._format_axis_label(axis)
        cfg_key = param.cfg_key
        result = self.axes.target(axis).sys_param_read(param, timeout=timeout)
        value = result.get("value")
        return {
            **result,
            "axis": axis,
            "axis_label": axis_label,
            "cfg_key": cfg_key,
            "type": param.value_type,
            "param_key": param.key,
            "param_id": param.param_id,
            "value": value,
        }

    def _read_param_views(
        self, param, timeout: float = MOTOR_COMMAND_TIMEOUT
    ) -> dict:
        live = self._read_live_param(param, timeout=timeout)
        flash = self._flash_param_read(
            param.axis, param.param_id, timeout=timeout
        )
        return {
            "axis": live["axis"],
            "axis_label": live["axis_label"],
            "cfg_key": live["cfg_key"],
            "type": live["type"],
            "param_key": live["param_key"],
            "param_id": live["param_id"],
            "live": live,
            "live_value": live["value"],
            "flash": flash,
            "flash_value": flash.get("value"),
        }

    def _respond_param_readbacks(self, gcmd, label: str, result: dict):
        gcmd.respond_info(label)
        gcmd.respond_info(
            "Live readback=%s"
            % (self._format_readback_value(result["live_value"]),)
        )
        gcmd.respond_info(
            "Flash readback=%s"
            % (self._format_readback_value(result["flash_value"]),)
        )

    def _read_calibration_snapshot(
        self, axis: str, strict: bool = True
    ) -> dict:
        values = {}
        errors = {}
        for param_id in CALIBRATION_PARAM_IDS:
            try:
                result = self._flash_param_read(
                    axis, param_id, timeout=MOTOR_COMMAND_TIMEOUT
                )
            except Exception as exc:
                if strict:
                    raise
                errors[str(param_id)] = repr(exc)
                continue
            value = result.get("value")
            param_key = result.get("param_key") or f"id_{param_id}"
            # Keep the in-memory map up to date with the freshly-read board value.
            try:
                p = self.registry.get_by_id(axis, param_id)
                p.value = value
            except (KeyError, AttributeError):
                pass
            values[param_key] = {
                "param_id": param_id,
                "value": value,
            }
        snapshot = {
            "axis": axis,
            "axis_label": self._format_axis_label(axis),
            "values": values,
        }
        if errors:
            snapshot["errors"] = errors
        return snapshot

    def _startup_steps(self):
        # startup_writes: none  -> assign addresses, then read only
        #                 stall -> also set the normal stall mode and clear
        #                          latched faults
        #                 params-> also write the cfg parameter overrides
        discovery = (
            "serial_target_discovery",
            self._startup_discover_serial_targets,
        )
        protect_check = (
            "protect_check",
            partial(
                self._startup_check_axis_protection,
                "motors",
                self.managed_axes,
                timeout=MOTOR_COMMAND_TIMEOUT,
            ),
        )
        if self.write_level < WRITE_LEVEL_STALL:
            return (discovery, protect_check)
        steps = (
            discovery,
            ("stall_mode", self.axes.set_normal_stall_mode),
            protect_check,
            (
                "protect_clear",
                partial(self._startup_clear_axis_faults, "motors"),
            ),
            (
                "protect_recheck",
                partial(
                    self._startup_recheck_axis_protection,
                    "motors",
                    timeout=MOTOR_COMMAND_TIMEOUT,
                ),
            ),
        )
        if self.write_level < WRITE_LEVEL_PARAMS:
            return steps
        return steps + (
            (
                "apply_overrides",
                partial(
                    self._startup_apply_axis_set_overrides,
                    self.managed_axes,
                    override_timeout=MOTOR_COMMAND_TIMEOUT,
                    calibration_timeout=MOTOR_COMMAND_TIMEOUT,
                ),
            ),
        )

    def _startup_discover_serial_targets(self):
        try:
            self.pin_manager.set_motor_pin_dir()
            prepared = self.axes.prepare_serial_bus()
            result = self.axes.probe_serial_axes(timeout=MOTOR_COMMAND_TIMEOUT)
        finally:
            self.pin_manager.set_motor_pin_normal()
        faulted = sorted(
            addr for addr, detail in result.items() if detail.get("faulted")
        )
        warned = sorted(
            addr for addr, detail in result.items() if detail.get("warned")
        )
        stalled = sorted(
            addr for addr, detail in result.items() if detail.get("stalled")
        )
        if faulted:
            self.gcode.respond_info(
                "Motor control: fault active on axes %s at startup, attempting clear"
                % faulted
            )
        if warned:
            self.gcode.respond_info(
                "Motor control: warning active on axes %s at startup, attempting clear"
                % warned
            )
        if stalled:
            self.gcode.respond_info(
                "Motor control: stall latch active on axes %s at startup"
                % stalled
            )
        return {
            "prepare": prepared,
            "targets": result,
        }

    def _startup_check_axis_protection(
        self, label: str, axes, *, timeout: float
    ):
        result = self.query_protection_status(
            axes=axes, data=PROTECTION_QUERY_DATA, timeout=timeout
        )
        active = {
            axis: detail
            for axis, detail in result.items()
            if detail.get("active")
        }
        warnings = {
            axis: detail
            for axis, detail in active.items()
            if not detail.get("has_error")
        }
        if warnings:
            self._emit_runtime_warning(
                {"source": "startup_check"}, warnings, status_text="ACTIVE"
            )
        self._startup_protection_active[label] = tuple(sorted(active))
        return result

    def _startup_clear_axis_faults(
        self, label: str, *, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ):
        active_axes = self._startup_protection_active.get(label, ())
        if not active_axes:
            return {
                "skipped": True,
                "reason": f"no active {label} startup faults",
            }
        return self.clear_fault_latches(
            axes=active_axes, data=5, timeout=timeout
        )

    def _startup_recheck_axis_protection(self, label: str, *, timeout: float):
        active_axes = self._startup_protection_active.get(label, ())
        if not active_axes:
            return {
                "skipped": True,
                "reason": f"no active {label} startup faults",
            }
        result = self.query_protection_status(
            axes=active_axes, data=PROTECTION_QUERY_DATA, timeout=timeout
        )
        active = {
            axis: detail
            for axis, detail in result.items()
            if detail.get("active")
        }
        errors = {
            axis: detail
            for axis, detail in result.items()
            if detail.get("has_error")
        }
        if errors:
            summary = self._format_fault_summary(errors)
            raise RuntimeError(
                f"{label} protection error still active after startup clear: "
                f"{summary}"
            )
        if active:
            axes = tuple(sorted(active.keys()))
            warn_msg = self._runtime_fault_message(
                {"source": "startup_recheck"},
                axes,
                active,
                is_warning=True,
                status_text="ACTIVE",
            )
            self.gcode.respond_raw(f"!! {warn_msg}")
        return result

    def _startup_gcode_warning(self, message: str):
        try:
            self.gcode.respond_raw(f"!! {message}")
        except Exception:
            _klog("failed emitting startup warning", level=logging.exception)

    @staticmethod
    def _unsafe_calibration_entries(entries: list[dict]) -> list[dict]:
        return [entry for entry in entries if entry.get("near_zero")]

    @staticmethod
    def _calibration_error_entries(entries: list[dict]) -> list[dict]:
        return [entry for entry in entries if entry.get("error")]

    def _startup_apply_axis_overrides(
        self, axis: str, *, timeout: float
    ) -> list:
        result = self.axes.apply_overrides(axis, timeout=timeout)
        actions = list(result.actions)
        writes = [a for a in actions if a.get("op") == "write"]
        errors = [
            a
            for a in actions
            if a.get("op")
            in (
                "read_error",
                "write_error",
                "verify_error",
                "verify_mismatch",
            )
        ]
        if writes:
            _klog(
                "startup runtime override axis=%s wrote=%d", axis, len(writes)
            )
        for action in errors:
            self._startup_gcode_warning(
                "motor control: startup runtime override axis=%s key=%s op=%s err=%s"
                % (
                    axis,
                    action.get("key"),
                    action.get("op"),
                    self._startup_action_error_text(action),
                )
            )
        return actions

    def _read_startup_axis_calibration(
        self, axis: str, *, timeout: float
    ) -> list[dict]:
        return self.axes.read_calibration_params(axis, timeout=timeout)

    @staticmethod
    def _startup_action_error_text(action: dict):
        if action.get("op") == "verify_mismatch":
            return "expected %r, read back %r" % (
                action.get("override_value"),
                action.get("current_value"),
            )
        return action.get("error") or action.get("wrote_value")

    def _startup_apply_axis_set_overrides(
        self, axes, *, override_timeout: float, calibration_timeout: float
    ):
        failures = {}
        unsafe_calibration = []
        calibration_errors = []
        axis_results = {}
        for axis in axes:
            actions = []
            calibration = []
            try:
                actions = self._startup_apply_axis_overrides(
                    axis, timeout=override_timeout
                )
            except Exception as exc:
                failures[axis] = repr(exc)
                _klog(
                    "startup runtime override failed axis=%s err=%s",
                    axis,
                    exc,
                    level=logging.warning,
                )
            try:
                calibration = self._read_startup_axis_calibration(
                    axis, timeout=calibration_timeout
                )
            except Exception as exc:
                calibration = []
                calibration_errors.append(
                    {
                        "axis": axis,
                        "key": "__read__",
                        "error": repr(exc),
                    }
                )
                _klog(
                    "startup calibration read failed axis=%s err=%s",
                    axis,
                    exc,
                    level=logging.warning,
                )
            calibration_errors.extend(
                self._calibration_error_entries(calibration)
            )
            unsafe_calibration.extend(
                self._unsafe_calibration_entries(calibration)
            )
            axis_results[axis] = {
                "actions": actions,
                "calibration": calibration,
            }
        if calibration_errors:
            joined = ", ".join(
                "%s.%s: %s"
                % (
                    entry.get("axis"),
                    entry.get("key"),
                    entry.get("error"),
                )
                for entry in calibration_errors
            )
            message = (
                f"motor control: startup calibration read failed: {joined}"
            )
            _klog(
                "startup calibration read failed: %s",
                joined,
                level=logging.warning,
            )
            self._startup_gcode_warning(message)
        if unsafe_calibration:
            for message in self._format_unsafe_calibration_warnings(
                unsafe_calibration
            ):
                log_message = message
                if log_message.startswith("motor control: "):
                    log_message = log_message[len("motor control: ") :]
                _klog("%s", log_message, level=logging.warning)
                self._startup_gcode_warning(message)
        if failures:
            joined = ", ".join(
                f"{axis}: {err}" for axis, err in failures.items()
            )
            raise RuntimeError(f"startup runtime override failed: {joined}")
        return axis_results

    def _startup_handler(self, eventtime):
        if self._startup_complete:
            return self.reactor.NEVER
        steps = self._startup_steps()
        if self._startup_step_index >= len(steps):
            self.motor_params_init = True
            self.is_ready = True
            self._startup_complete = True
            self.reactor.update_timer(
                self._protection_poll_timer,
                self.reactor.monotonic() + PROTECTION_POLL_INTERVAL,
            )
            self.temp_sensors.start()
            _klog("startup complete")
            self.gcode.respond_info(
                "Motor control startup succeeded on attempt %d."
                % (self._startup_auto_retry_count + 1,)
            )
            return self.reactor.NEVER
        step_name, step_fn = steps[self._startup_step_index]
        try:
            _klog(
                "startup step %d/%d %s",
                self._startup_step_index + 1,
                len(steps),
                step_name,
            )
            step_fn()
            self._startup_step_index += 1
            return self.reactor.monotonic()
        except Exception as exc:
            failure = self._format_startup_failure(step_name, exc)
            self._startup_error = failure
            _klog("startup step failed: %s", step_name, level=logging.exception)
            attempt_label = self._startup_attempt_label()
            if (
                self.auto_retry
                and self._startup_allow_auto_retry
                and self._startup_auto_retry_count < STARTUP_AUTO_RETRY_LIMIT
            ):
                next_retry = self._startup_auto_retry_count + 1
                try:
                    self.gcode.respond_info(
                        "motor control %s %s. "
                        "Retrying in %.0fs. Attempt %d/%d next."
                        % (
                            attempt_label,
                            failure,
                            STARTUP_AUTO_RETRY_DELAY,
                            next_retry,
                            STARTUP_AUTO_RETRY_LIMIT,
                        )
                    )
                except Exception:
                    _klog(
                        "failed emitting startup auto-retry warning",
                        level=logging.exception,
                    )
                self._startup_auto_retry_count = next_retry
                self._reset_startup_state(reset_retry_state=False)
                return self.reactor.monotonic() + STARTUP_AUTO_RETRY_DELAY
            self._startup_complete = True
            self.is_ready = False
            self.motor_params_init = False
            try:
                self.gcode.respond_raw(
                    "!! motor control %s %s" % (attempt_label, failure)
                )
            except Exception:
                _klog(
                    "failed emitting final startup failure warning",
                    level=logging.exception,
                )
            return self.reactor.NEVER

    @staticmethod
    def _format_startup_failure(step_name: str, exc: Exception) -> str:
        text = str(exc)
        match = STARTUP_PROTOCOL_ERROR_RE.match(text)
        if step_name == "serial_target_discovery":
            axis = (
                None
                if not match
                else STARTUP_SERIAL_AXIS_BY_ADDR.get(int(match.group(2), 16))
            )
            action = (
                "discovering the %s motor" % (axis,)
                if axis is not None
                else "discovering the serial motors"
            )
        elif step_name == "extruder_target_discovery":
            action = "discovering the extruder motor"
        else:
            action = STARTUP_STEP_ACTIONS.get(
                step_name, "running %s" % (step_name.replace("_", " "),)
            )
        if not match:
            return "failed while %s: %s" % (action, text)
        reason, addr_hex, func_hex = match.groups()
        addr = int(addr_hex, 16)
        func = int(func_hex, 16)
        details = "address 0x%02X" % (addr,)
        if STARTUP_STEP_FUNCTIONS.get(step_name) != func:
            details += ", function 0x%02X" % (func,)
        return "failed while %s (%s): %s" % (action, details, reason)

    def _startup_attempt_label(self) -> str:
        if self._startup_auto_retry_count <= 0:
            return "startup"
        return "retry %d/%d" % (
            self._startup_auto_retry_count,
            STARTUP_AUTO_RETRY_LIMIT,
        )

    def _reset_startup_state(self, reset_retry_state: bool = True):
        self._startup_started = True
        self._startup_complete = False
        self.is_ready = False
        self.motor_params_init = False
        self.temp_sensors.stop()
        self._startup_error = ""
        self._startup_step_index = 0
        self._fault_cleanup_pending = False
        self._fault_cleanup_axes = set()
        self._fault_cleanup_homing_fault = False
        self._fault_cleanup_startup_fault = False
        self._startup_fault_clear_ok = None
        self._startup_fault_clear_result = ""
        self._startup_protection_active = {}
        if reset_retry_state:
            self._startup_auto_retry_count = 0
            self._startup_allow_auto_retry = True
        self.reactor.update_timer(self._fault_cleanup_timer, self.reactor.NEVER)
        self.reactor.update_timer(
            self._protection_poll_timer, self.reactor.NEVER
        )

    def _begin_startup(
        self, force: bool = False, allow_auto_retry: bool = True
    ):
        if self._startup_started and not force:
            return
        self._reset_startup_state(reset_retry_state=True)
        self._startup_allow_auto_retry = allow_auto_retry
        when = self.reactor.monotonic()
        _klog("scheduling startup sequence at %.3f", when)
        self.reactor.update_timer(self._startup_timer, when)

    def _handle_serial485_ready(self):
        self._begin_startup(force=False, allow_auto_retry=True)

    def _handle_shutdown(self):
        self.is_ready = False
        self.motor_params_init = False
        self.temp_sensors.stop()
        for timer in (
            self._startup_timer,
            self._fault_cleanup_timer,
            self._protection_poll_timer,
        ):
            self.reactor.update_timer(timer, self.reactor.NEVER)

    def _update_fault_state(self, axis: str, detail: dict):
        num = AXIS_TO_NUM_MAP.get(axis)
        if num is None:
            return
        error_value = int(detail.get("error_code", 0) or 0)
        warning_value = int(detail.get("warning_code", 0) or 0)
        for key in (num, str(num)):
            self.motor_error_code[key] = error_value
            self.motor_warning_code[key] = warning_value
        self.motor_fault_detail[axis] = detail

    def _clear_fault_state(self, axes):
        for axis in axes:
            num = AXIS_TO_NUM_MAP.get(axis)
            if num is None:
                continue
            for key in (num, str(num)):
                self.motor_error_code[key] = 0
                self.motor_warning_code[key] = 0
            self.motor_fault_detail.pop(axis, None)

    def get_axis_stall_mode_raw(
        self, axis: str, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        return self.axes.get_axis_stall_mode_raw(
            str(axis).strip().lower(), timeout=timeout
        )

    def set_axis_stall_mode_raw(
        self, axis: str, raw_mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        return self.axes.set_axis_stall_mode_raw(
            str(axis).strip().lower(), raw_mode=raw_mode, timeout=timeout
        )

    def set_homing_stall_mode(
        self, mode: int, timeout: float = MOTOR_COMMAND_TIMEOUT
    ):
        return self.axes.set_homing_stall_mode(int(mode), timeout=timeout)

    def query_protection_status(
        self,
        axes=None,
        data: int = PROTECTION_QUERY_DATA,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ):
        axes = self.managed_axes if axes is None else tuple(axes)
        result = self.axes.check_protection(axes, data=data, timeout=timeout)
        for axis, detail in result.items():
            if detail.get("active"):
                self._update_fault_state(axis, detail)
            else:
                self._clear_fault_state((axis,))
        return result

    def clear_fault_latches(
        self, axes=None, data: int = 5, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ):
        axes = self.managed_axes if axes is None else tuple(axes)
        result = self.axes.clear_fault_latches(axes, data=data, timeout=timeout)
        self._clear_fault_state(tuple(result.keys()))
        return result

    def recover_faults_for_homing_start(
        self,
        axes=None,
        query_data: int = PROTECTION_QUERY_DATA,
        clear_data: int = 5,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ):
        axes = self.kinematic_axes if axes is None else tuple(axes)
        before = self.query_protection_status(
            axes=axes, data=query_data, timeout=timeout
        )
        active_before = {
            axis: detail
            for axis, detail in before.items()
            if detail.get("active")
        }
        errors_before = {
            axis: detail
            for axis, detail in before.items()
            if detail.get("has_error")
        }
        warnings_before = {
            axis: detail
            for axis, detail in before.items()
            if detail.get("active") and not detail.get("has_error")
        }
        if warnings_before:
            self._emit_runtime_warning(
                {"source": "homing_start"}, warnings_before
            )
        cleared = {}
        if errors_before:
            cleared = self.clear_fault_latches(
                tuple(sorted(errors_before)), data=clear_data
            )
            after = self.query_protection_status(
                axes=axes, data=query_data, timeout=timeout
            )
        else:
            after = before
        persistent = {
            axis: detail
            for axis, detail in after.items()
            if detail.get("active")
        }
        persistent_errors = {
            axis: detail
            for axis, detail in after.items()
            if detail.get("has_error")
        }
        return {
            "before": before,
            "active_before": active_before,
            "cleared": cleared,
            "after": after,
            "persistent": persistent,
            "persistent_errors": persistent_errors,
        }

    def query_kinematic_protection_status(
        self,
        data: int = PROTECTION_QUERY_DATA,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ):
        return self.query_protection_status(
            axes=self.kinematic_axes, data=data, timeout=timeout
        )

    def clear_kinematic_fault_latches(
        self, data: int = 5, timeout: float = MOTOR_NO_ACK_TIMEOUT
    ):
        return self.clear_fault_latches(
            axes=self.kinematic_axes, data=data, timeout=timeout
        )

    def recover_kinematic_faults_for_homing_start(
        self,
        query_data: int = PROTECTION_QUERY_DATA,
        clear_data: int = 5,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
    ):
        return self.recover_faults_for_homing_start(
            axes=self.kinematic_axes,
            query_data=query_data,
            clear_data=clear_data,
            timeout=timeout,
        )

    def _get_homing_session_state(self):
        homing = self.printer.lookup_object("homing")
        return {
            "active": bool(homing.has_active_homing_session()),
            "aborted": bool(homing.is_homing_session_aborted()),
            "abort_in_progress": bool(homing.is_homing_abort_in_progress()),
        }

    def _is_homing_context_active(self) -> bool:
        session = self._get_homing_session_state()
        return bool(
            self.is_homing
            or self._homing_move_depth > 0
            or (session["active"] and not session["aborted"])
            or session["abort_in_progress"]
        )

    def _invalidate_axes_homing(self, axes):
        if not axes:
            return
        axes = set(axes)
        try:
            toolhead = self.printer.lookup_object("toolhead")
            kin = toolhead.get_kinematics()
            clear_axes = ""
            if "x" in axes or "y" in axes:
                clear_axes += "xy"
            if "z" in axes or "z1" in axes:
                clear_axes += "z"
            if clear_axes:
                kin.clear_homing_state(clear_axes)
        except Exception:
            _klog(
                "failed invalidating homing state axes=%s",
                sorted(axes),
                level=logging.exception,
            )

    def _set_homing_move_active(self, active: bool):
        self.is_homing = bool(active)

    def _handle_homing_move_begin(self, _hmove):
        self._homing_move_depth += 1
        self._set_homing_move_active(True)

    def _handle_homing_move_end(self, _hmove):
        if self._homing_move_depth > 0:
            self._homing_move_depth -= 1
        self._set_homing_move_active(self._homing_move_depth > 0)

    def _handle_gcode_command_error(self):
        if self._homing_move_depth <= 0:
            return
        _klog(
            "clearing homing-active state after gcode command error",
            level=logging.warning,
        )
        self._homing_move_depth = 0
        self._set_homing_move_active(False)
        if self._fault_cleanup_pending and self._fault_cleanup_homing_fault:
            self.reactor.update_timer(
                self._fault_cleanup_timer, self.reactor.NOW
            )

    def _request_homing_fault_abort(self, detail: dict):
        homing = self.printer.lookup_object("homing", None)
        if homing is None or not hasattr(homing, "request_motor_fault_abort"):
            _klog(
                "homing object has no request_motor_fault_abort(); "
                "falling back to deferred cleanup",
                level=logging.warning,
            )
            return False
        homing.request_motor_fault_abort(detail)
        return True

    def _schedule_fault_cleanup(
        self,
        eventtime: float,
        axes,
        during_homing: bool = False,
        during_startup: bool = False,
    ):
        self._fault_cleanup_pending = True
        self._fault_cleanup_axes.update(axes)
        if during_homing:
            self._fault_cleanup_homing_fault = True
        if during_startup:
            self._fault_cleanup_startup_fault = True
        when = max(
            eventtime + FAULT_CLEANUP_RETRY_DELAY,
            self.reactor.monotonic() + FAULT_CLEANUP_RETRY_DELAY,
        )
        self.reactor.update_timer(self._fault_cleanup_timer, when)

    def _fault_cleanup_ready(self, toolhead, eventtime: float) -> bool:
        _print_time, _est_print_time, lookahead_empty = toolhead.check_busy(
            eventtime
        )
        special_state = getattr(toolhead, "special_queuing_state", "")
        homing_state = self._get_homing_session_state()
        if self._fault_cleanup_homing_fault:
            if (
                self._homing_move_depth > 0
                or self.is_homing
                or homing_state["active"]
            ):
                return False
            if homing_state["abort_in_progress"]:
                return False
            if not lookahead_empty:
                return False
            return special_state not in ("Drip", "Priming")
        if self._fault_cleanup_startup_fault:
            if not lookahead_empty:
                return False
            return special_state not in ("Drip", "Priming")
        return not special_state and lookahead_empty

    def _fault_cleanup_handler(self, eventtime):
        if not self._fault_cleanup_pending:
            return self.reactor.NEVER
        toolhead = self.printer.lookup_object("toolhead", None)
        if toolhead is None:
            return eventtime + FAULT_CLEANUP_RETRY_DELAY
        try:
            if not self._fault_cleanup_ready(toolhead, eventtime):
                return eventtime + FAULT_CLEANUP_RETRY_DELAY
        except Exception:
            _klog(
                "failed checking busy state during fault cleanup",
                level=logging.exception,
            )
            return eventtime + FAULT_CLEANUP_RETRY_DELAY
        axes = tuple(sorted(self._fault_cleanup_axes))
        invalidate_axes = axes
        startup_fault = self._fault_cleanup_startup_fault
        if self._fault_cleanup_homing_fault:
            invalidate_axes = tuple(
                sorted(set(self.kinematic_axes).union(axes))
            )
            _klog(
                "fault cleanup invalidating full kinematic homing state "
                "after homing-time fault axes=%s",
                invalidate_axes,
                level=logging.warning,
            )
        # Avoid mutating homing state from the async stall callback while
        # Klipper is still unwinding an active drip/home move.
        self._invalidate_axes_homing(invalidate_axes)
        startup_result_ok = None
        startup_result_msg = ""
        clear_failed = False
        if axes:
            try:
                result = self.clear_fault_latches(axes, data=5)
                _klog("cleared fault/warn codes result=%s", result)
            except Exception:
                clear_failed = True
                _klog(
                    "failed clearing fault/warn codes", level=logging.exception
                )
        try:
            self.printer.lookup_object("stepper_enable").motor_off()
            _klog("completed deferred motor_off axes=%s", axes)
        except Exception:
            _klog(
                "failed deferred motor_off axes=%s",
                axes,
                level=logging.exception,
            )
        if startup_fault:
            axes_label = self._format_fault_axes(axes)
            if axes and not clear_failed:
                try:
                    protection = self.query_protection_status(
                        axes=axes,
                        data=PROTECTION_QUERY_DATA,
                        timeout=MOTOR_COMMAND_TIMEOUT,
                    )
                except Exception as exc:
                    startup_result_ok = False
                    startup_result_msg = (
                        "motor control startup deferred clear could not be verified "
                        f"axes={axes_label}: {exc!r} | proceeding"
                    )
                else:
                    persistent = {
                        axis: detail
                        for axis, detail in protection.items()
                        if axis in axes and detail.get("active")
                    }
                    if persistent:
                        startup_result_ok = False
                        startup_result_msg = (
                            "motor control startup deferred clear completed with "
                            f"persistent faults axes={axes_label}: "
                            f"{self._format_fault_summary(persistent)} | proceeding"
                        )
                    else:
                        startup_result_ok = True
                        startup_result_msg = f"motor control startup deferred clear succeeded axes={axes_label}"
            elif clear_failed:
                startup_result_ok = False
                startup_result_msg = (
                    "motor control startup deferred clear failed "
                    f"axes={axes_label} | proceeding"
                )
            else:
                startup_result_ok = True
                startup_result_msg = f"motor control startup deferred clear succeeded axes={axes_label}"
            self._startup_fault_clear_ok = startup_result_ok
            self._startup_fault_clear_result = startup_result_msg
            if startup_result_ok:
                _klog("%s", startup_result_msg)
            else:
                _klog("%s", startup_result_msg, level=logging.warning)
            self.gcode.respond_info(startup_result_msg)
        self._fault_cleanup_pending = False
        self._fault_cleanup_axes.clear()
        self._fault_cleanup_homing_fault = False
        self._fault_cleanup_startup_fault = False
        return self.reactor.NEVER

    def get_homing_fault_blocker(self):
        if self.write_level < WRITE_LEVEL_STALL:
            return (
                "Homing needs [motor_control_hi] startup_writes: stall "
                "(the homing stall mode is a write to the motor drivers)"
            )
        motor_ready = bool(self.is_ready and self.motor_params_init)
        if self._startup_started and not self._startup_complete:
            return (
                "Motor control is not ready for homing: startup still running"
            )
        if self._fault_cleanup_pending and not self._fault_cleanup_homing_fault:
            axes = tuple(sorted(self._fault_cleanup_axes))
            axes_label = self._format_fault_axes(axes)
            return (
                "Motor control is not in a clean state for homing: "
                f"fault cleanup still pending for axes {axes_label}"
            )
        if self._startup_error:
            return (
                "Motor control is not ready for homing: startup failed "
                f"({self._startup_error})"
            )
        if not motor_ready:
            return (
                "Motor control is not ready for homing: "
                f"is_ready={self.is_ready} motor_params_init={self.motor_params_init}"
            )
        return None

    def _query_protection_axes(
        self, axes, data: int, timeout: float, source: str
    ) -> dict:
        axes = tuple(axes)
        details = self.query_protection_status(
            axes=axes, data=data, timeout=timeout
        )
        return {"source": source, "axes": details}

    @staticmethod
    def _group_has_active_fault(detail: dict) -> bool:
        axes = detail.get("axes", {})
        if isinstance(axes, dict):
            return any(bool(item.get("active")) for item in axes.values())
        return False

    @staticmethod
    def _active_fault_details(detail: dict) -> dict[str, dict]:
        axes = detail.get("axes", {})
        if isinstance(axes, dict):
            return {
                axis: item
                for axis, item in axes.items()
                if isinstance(item, dict) and item.get("active")
            }
        return {}

    @staticmethod
    def _active_error_details(detail: dict) -> dict[str, dict]:
        axes = detail.get("axes", {})
        if isinstance(axes, dict):
            return {
                axis: item
                for axis, item in axes.items()
                if isinstance(item, dict) and item.get("has_error")
            }
        return {}

    @staticmethod
    def _active_warning_only_details(detail: dict) -> dict[str, dict]:
        axes = detail.get("axes", {})
        if isinstance(axes, dict):
            return {
                axis: item
                for axis, item in axes.items()
                if isinstance(item, dict)
                and item.get("active")
                and not item.get("has_error")
            }
        return {}

    def _format_fault_axes(self, axes) -> str:
        labels = [self._format_axis_label(axis) for axis in axes]
        return ",".join(labels) if labels else "unknown"

    def _format_fault_summary(self, fault_details: dict[str, dict]) -> str:
        return format_fault_summary(
            fault_details,
            axis_labeler=self._format_axis_label,
            empty_text="no active fault details",
        )

    def _runtime_fault_message(
        self,
        detail: dict,
        axes,
        fault_details: dict[str, dict],
        during_homing: bool = False,
        is_warning: bool = False,
        status_text: str = "",
    ) -> str:
        source = detail.get("source", "unknown")
        severity = "warning" if is_warning else "fault"
        qualifier = f" ({status_text})" if status_text else ""
        context = "during homing " if during_homing else ""
        return "Motor protection %s%s %son axes %s (source=%s): %s" % (
            severity,
            qualifier,
            context,
            self._format_fault_axes(axes),
            source,
            self._format_fault_summary(fault_details),
        )

    def _emit_runtime_warning(
        self,
        detail: dict,
        warnings: dict[str, dict],
        during_homing: bool = False,
        status_text: str = "",
    ):
        axes = tuple(sorted(warnings))
        msg = self._runtime_fault_message(
            detail,
            axes,
            warnings,
            during_homing=during_homing,
            is_warning=True,
            status_text=status_text,
        )
        _klog("warning-only: %s", msg, level=logging.warning)
        self.gcode.respond_raw("!! %s" % msg)

    def _recoverable_fault_subject(self, axes) -> str:
        axes = tuple(axes)
        return "Motor axes %s" % (self._format_fault_axes(axes),)

    def _pause_for_recoverable_fault_axes(self, axes):
        if EXTRUDER_AXIS not in axes:
            return
        print_stats = None
        try:
            print_stats = self.printer.lookup_object("print_stats", None)
        except Exception:
            _klog(
                "failed looking up print_stats for recoverable fault",
                level=logging.exception,
            )
        if getattr(print_stats, "state", None) != "printing":
            return
        # Async PAUSE delivery: send_pause_command gates SD immediately; the
        # macro runs later through the reactor.
        try:
            if not self.pause_resume.pause_command_sent:
                self.pause_resume.send_pause_command()
                self.reactor.register_async_callback(
                    lambda e: self.gcode.run_script("PAUSE")
                )
        except Exception:
            _klog(
                "failed requesting pause for recoverable fault",
                level=logging.exception,
            )

    def _reset_recoverable_fault_axes(self, axes):
        if EXTRUDER_AXIS not in axes:
            return
        stepper_enable = self.printer.lookup_object("stepper_enable")
        stepper_enable.set_motors_enable(["extruder"], False)

    def _handle_runtime_recoverable_fault(
        self, detail: dict, fault_details: dict[str, dict]
    ):
        axes = tuple(sorted(fault_details))
        source = detail.get("source", "unknown")
        subject = self._recoverable_fault_subject(axes)
        err_msg = "%s protection fault (source=%s): %s" % (
            subject,
            source,
            self._format_fault_summary(fault_details),
        )
        try:
            self.gcode.respond_raw(f"!! {err_msg}")
        except Exception:
            _klog(
                "failed emitting raw recoverable fault error",
                level=logging.exception,
            )
        self._pause_for_recoverable_fault_axes(axes)
        try:
            clear_result = self.clear_fault_latches(axes, data=5)
        except Exception as exc:
            _klog(
                "failed clearing recoverable fault after runtime stall",
                level=logging.exception,
            )
            try:
                self.gcode.respond_raw(
                    "!! %s protection fault clear failed: %r" % (subject, exc)
                )
            except Exception:
                _klog(
                    "failed emitting recoverable clear failure warning",
                    level=logging.exception,
                )
            return
        try:
            protection = self.query_protection_status(
                axes=axes,
                data=PROTECTION_QUERY_DATA,
                timeout=MOTOR_COMMAND_TIMEOUT,
            )
        except Exception as exc:
            _klog(
                "failed rechecking recoverable fault after clear",
                level=logging.exception,
            )
            try:
                self.gcode.respond_raw(
                    "!! %s protection fault clear could not be verified: %r"
                    % (subject, exc)
                )
            except Exception:
                _klog(
                    "failed emitting recoverable recheck warning",
                    level=logging.exception,
                )
            return
        recheck_detail = {
            "source": f"{source}:recover_recheck",
            "axes": protection,
        }
        warnings = self._active_warning_only_details(recheck_detail)
        if warnings:
            self._emit_runtime_warning(recheck_detail, warnings)
        persistent = {
            axis: item
            for axis, item in protection.items()
            if item.get("active") and item.get("has_error")
        }
        if persistent:
            warn_msg = (
                "%s protection fault remains active after clear attempt: %s"
                % (subject, self._format_fault_summary(persistent))
            )
            _klog("%s", warn_msg, level=logging.warning)
            try:
                self.gcode.respond_raw(f"!! {warn_msg}")
            except Exception:
                _klog(
                    "failed emitting persistent recoverable fault warning",
                    level=logging.exception,
                )
            return
        self._clear_fault_state(axes)
        try:
            self._reset_recoverable_fault_axes(axes)
        except Exception as exc:
            _klog(
                "cleared recoverable fault but failed to reset enable state",
                level=logging.exception,
            )
            try:
                self.gcode.respond_raw(
                    "!! %s protection fault cleared, but failed to reset motor state: %r"
                    % (subject, exc)
                )
            except Exception:
                _klog(
                    "failed emitting recoverable enable reset warning",
                    level=logging.exception,
                )
            return
        if axes == EXTRUDER_AXES:
            success_msg = (
                "Extruder motor protection fault cleared; next extrusion will "
                "re-enable the extruder motor"
            )
        else:
            success_msg = "%s protection fault cleared" % (subject,)
        _klog(
            "%s source=%s clear=%s recheck=%s",
            success_msg,
            source,
            clear_result,
            protection,
        )
        try:
            self.gcode.respond_info(success_msg)
        except Exception:
            _klog(
                "failed emitting recoverable clear success message",
                level=logging.exception,
            )

    def _handle_active_fault(
        self, detail: dict, eventtime: float, during_homing: bool | None = None
    ):
        if not self._group_has_active_fault(detail):
            return
        if during_homing is None:
            during_homing = self._is_homing_context_active()
        startup_active = self._startup_started and not self._startup_complete
        errors = self._active_error_details(detail)
        warnings = self._active_warning_only_details(detail)
        if warnings:
            self._emit_runtime_warning(
                detail, warnings, during_homing=during_homing
            )
            if not errors:
                return
        if not errors:
            return
        shutdown_errors = {
            axis: item
            for axis, item in errors.items()
            if RUNTIME_FAULT_ACTION_BY_AXIS.get(
                axis, RUNTIME_FAULT_ACTION_SHUTDOWN
            )
            == RUNTIME_FAULT_ACTION_SHUTDOWN
        }
        recover_errors = {
            axis: item
            for axis, item in errors.items()
            if RUNTIME_FAULT_ACTION_BY_AXIS.get(axis)
            == RUNTIME_FAULT_ACTION_RECOVER
        }
        _klog("active protection fault detail=%s", detail, level=logging.error)
        if startup_active:
            msg = (
                "motor control startup found a persisted protection fault; "
                "attempting deferred clear before ready"
            )
            self.gcode.respond_info(msg)
            self._schedule_fault_cleanup(
                eventtime, tuple(sorted(errors)), during_startup=True
            )
            return
        if recover_errors:
            self._handle_runtime_recoverable_fault(detail, recover_errors)
        if shutdown_errors and not during_homing:
            msg = self._runtime_fault_message(
                detail,
                tuple(sorted(shutdown_errors)),
                shutdown_errors,
                during_homing=False,
            )
            self.printer.invoke_shutdown(msg)
            return
        if during_homing and shutdown_errors:
            homing_detail = {
                **detail,
                "axes": {
                    axis: detail["axes"][axis]
                    for axis in shutdown_errors
                    if axis in detail.get("axes", {})
                },
            }
            homing_axes = tuple(sorted(shutdown_errors))
            abort_requested = False
            try:
                abort_requested = self._request_homing_fault_abort(
                    homing_detail
                )
            except Exception:
                _klog(
                    "immediate homing fault abort failed; "
                    "falling back to deferred cleanup",
                    level=logging.exception,
                )
            if abort_requested:
                return
            self._schedule_fault_cleanup(
                eventtime, homing_axes, during_homing=True
            )
            return
        if not recover_errors and not shutdown_errors:
            _klog(
                "active fault had no runtime policy target detail=%s",
                detail,
                level=logging.warning,
            )

    def get_status(self, _eventtime=None):
        return {
            "cut": {
                "active": bool(self.is_check_cut_pos_start),
                "latched": bool(self.cut_state),
            },
            **self._transport_ready_status(),
            "is_homing": self.is_homing,
            "motor_ready": self.is_ready and self.motor_params_init,
            "startup_writes": WRITE_LEVEL_NAMES[self.write_level],
        }

    def note_stall_pin_event(self, axis: str, active: int, eventtime: float):
        if not (
            self._startup_complete and self.is_ready and self.motor_params_init
        ):
            return
        if axis == "x" and self.is_check_cut_pos_start:
            if not active:
                return
            last = self._stall_last_event.get(axis, 0.0)
            if eventtime - last < STALL_EVENT_MIN_INTERVAL:
                return
            self._stall_last_event[axis] = eventtime
            self.cut_state = True
            _klog("cut-position stall latched axis=%s active=%s", axis, active)
            return
        if not active:
            return
        last = self._stall_last_event.get(axis, 0.0)
        if eventtime - last < STALL_EVENT_MIN_INTERVAL:
            return
        self._stall_last_event[axis] = eventtime
        _klog(
            "stall pin event axis=%s active=%s",
            axis,
            active,
            level=logging.warning,
        )
        was_homing = self._is_homing_context_active()
        if not was_homing:
            try:
                self.printer.send_event(
                    "motor_control:protection_signal", {"axis": axis}
                )
            except Exception:
                _klog("PLR protection listener failed", level=logging.exception)
        self.reactor.register_async_callback(
            lambda e, a=axis, act=active, h=was_homing: (
                self._reactor_stall_query(a, act, was_homing=h)
            )
        )

    def _unverified_fault_detail(
        self, axis: str, source: str, exc: Exception
    ) -> dict:
        return {
            "source": f"{source}:query_failed",
            "axes": {
                axis: {
                    "axis": axis,
                    "active": True,
                    "has_error": True,
                    "error_code": 0,
                    "warning_code": 0,
                    "query_error": repr(exc),
                    "unverified": True,
                },
            },
        }

    def _handle_unverified_axis_fault(
        self,
        axis: str,
        source: str,
        exc: Exception,
        during_homing: bool | None = None,
    ):
        detail = self._unverified_fault_detail(axis, source, exc)
        self._update_fault_state(axis, detail["axes"][axis])
        self._handle_active_fault(
            detail, self.reactor.monotonic(), during_homing=during_homing
        )

    def _reactor_stall_query(
        self, axis: str, active: int, was_homing: bool = False
    ):
        if not active or self.printer.is_shutdown():
            return
        source = f"stall_pin:{active}"
        during_homing = was_homing or self._is_homing_context_active()
        try:
            singleton = self.check_protection_code(
                axis=axis,
                data=PROTECTION_QUERY_DATA,
                timeout=MOTOR_STALL_EVENT_VERIFY_TIMEOUT,
                source=source,
            )
            _klog(
                "stall singleton query axis=%s active=%s "
                "was_homing=%s detail=%s",
                axis,
                active,
                was_homing,
                singleton,
            )
            if self._group_has_active_fault(singleton):
                self._handle_active_fault(
                    singleton,
                    self.reactor.monotonic(),
                    during_homing=during_homing,
                )
        except Exception as exc:
            _klog(
                "stall query failed axis=%s active=%s",
                axis,
                active,
                level=logging.exception,
            )
            self._handle_unverified_axis_fault(
                axis, source, exc, during_homing=during_homing
            )

    def _protection_poll_handler(self, eventtime):
        if not self._startup_complete or not self.is_ready:
            return eventtime + PROTECTION_POLL_INTERVAL
        if self._fault_cleanup_pending:
            return eventtime + PROTECTION_POLL_INTERVAL
        self._process_protection_poll()
        return eventtime + PROTECTION_POLL_INTERVAL

    def _process_protection_poll(self):
        try:
            axes_result = {}
            for axis in self.managed_axes:
                try:
                    axes_result.update(
                        self.query_protection_status(
                            axes=(axis,),
                            data=PROTECTION_QUERY_DATA,
                            timeout=MOTOR_COMMAND_TIMEOUT,
                        )
                    )
                except Exception as exc:
                    _klog(
                        "periodic protection poll failed axis=%s",
                        axis,
                        level=logging.exception,
                    )
            detail = {"source": "periodic_poll", "axes": axes_result}
            if not self._group_has_active_fault(detail):
                return
            errors = self._active_error_details(detail)
            warnings = self._active_warning_only_details(detail)
            if errors:
                self._handle_active_fault(detail, self.reactor.monotonic())
                return
            if warnings:
                self._emit_runtime_warning(detail, warnings)
        except Exception:
            _klog("periodic protection poll failed", level=logging.exception)

    def check_protection_code(
        self,
        axis=None,
        data: int = PROTECTION_QUERY_DATA,
        timeout: float = MOTOR_COMMAND_TIMEOUT,
        source: str = "manual",
    ):
        if axis is None:
            return self._query_protection_axes(
                axes=self.managed_axes,
                data=data,
                timeout=timeout,
                source=source,
            )
        axis_name = (
            axis if isinstance(axis, str) else self._axis_from_num(int(axis))
        )
        return self._query_protection_axes(
            axes=(axis_name,), data=data, timeout=timeout, source=source
        )

    def cmd_MOTOR_CLEAR_ERROR(self, gcmd):
        try:
            before = self.check_protection_code(
                axis=None,
                data=PROTECTION_QUERY_DATA,
                timeout=MOTOR_COMMAND_TIMEOUT,
                source="clear-precheck",
            )
            active_before = self._active_fault_details(before)
            if not active_before:
                gcmd.respond_info(
                    "MOTOR_CLEAR_ERROR: no active motor faults or warnings"
                )
                return {
                    "before": before,
                    "cleared": {},
                    "after": before,
                    "persistent": {},
                }
            gcmd.respond_info(
                "MOTOR_CLEAR_ERROR pending: %s"
                % (self._format_fault_summary(active_before),)
            )
            cleared = self.clear_fault_latches(
                tuple(sorted(active_before)), data=5
            )
            after = self.check_protection_code(
                axis=None,
                data=PROTECTION_QUERY_DATA,
                timeout=MOTOR_COMMAND_TIMEOUT,
                source="clear-postcheck",
            )
            persistent = self._active_fault_details(after)
            if persistent:
                gcmd.respond_info(
                    "MOTOR_CLEAR_ERROR remaining: %s"
                    % (self._format_fault_summary(persistent),)
                )
            else:
                gcmd.respond_info(
                    "MOTOR_CLEAR_ERROR: cleared all active motor faults"
                )
            return {
                "before": before,
                "cleared": cleared,
                "after": after,
                "persistent": persistent,
            }
        except Exception as exc:
            raise gcmd.error(str(exc))

    def cmd_MOTOR_READ_PARAM(self, gcmd):
        try:
            param, label = self._resolve_param_command_target(gcmd)
            self._warn_if_param_not_configurable(gcmd, param, label)
            result = self._read_param_views(
                param, timeout=MOTOR_COMMAND_TIMEOUT
            )
            self._respond_param_readbacks(gcmd, label, result)
            return result
        except Exception as exc:
            raise gcmd.error(str(exc))


def load_config(config):
    motor_control = MotorControl(config)
    # homing.py and z_align.py look the object up as "motor_control".
    config.get_printer().add_object("motor_control", motor_control)
    return motor_control
