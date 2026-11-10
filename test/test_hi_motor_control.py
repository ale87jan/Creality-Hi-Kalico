# Checks of the Creality Hi closed-loop motor module (motor_control_hi):
# the write gate, the startup sequence per startup_writes level and the
# phase 2 config in batch mode against the real MCU dictionaries.
from __future__ import annotations

import json
import pathlib
import struct

import pytest
from test_hi_config import (
    XY_STEP_PINS,
    parse_output,
    run_batch,
    stepped_oids,
    stepper_oids,
)

from klippy.extras import motor_control_hi as mc
from klippy.extras import serial_485

ROOT = pathlib.Path(__file__).parent.parent
PHASE2 = ROOT / "config" / "hi" / "phase2.cfg"
MAP = json.loads((ROOT / "klippy/extras/motor_map_hi.json").read_text())
FORMATS = {"uint16_t": "<H", "int16_t": "<h", "uint32_t": "<I", "float": "<f"}


class FakeMotorBus:
    "Two motor drivers (0x81, 0x82) answering like a stock Hi"

    def __init__(self):
        self.sent = []
        self.params = {}
        for axis in ("x", "y"):
            addr = int(MAP[axis]["_addr"], 16)
            self.params[addr] = {
                entry["value_id"]: (entry["value_type"], entry["value"])
                for name, entry in MAP[axis].items()
                if not name.startswith("_")
            }

    def is_configured(self):
        return True

    def send(self, packet, timeout=1.0, attempts=1, response_timeout=None):
        packet = bytes(packet)
        addr, func, payload = packet[0], packet[3], packet[4:]
        self.sent.append((addr, func, payload))
        if addr not in self.params:
            return None
        if func == mc.FUNC_READ_ADDR and payload == b"\x02":
            return self._reply(addr, func, bytes([addr]))
        if func == mc.FUNC_PROTECTION and payload == b"\x0b":
            return self._reply(addr, func, bytes(8))
        if func == mc.FUNC_STALL_MODE:
            return self._reply(addr, func, payload[:1])
        if func == mc.FUNC_SYS_PARAM and payload[:1] == b"\x02":
            value_type, value = self.params[addr][payload[1]]
            return self._reply(
                addr, func, struct.pack(FORMATS[value_type], value)
            )
        if func == mc.FUNC_SYS_PARAM and payload[:1] == b"\x01":
            value_type, _old = self.params[addr][payload[1]]
            value = struct.unpack(FORMATS[value_type], payload[2:])[0]
            self.params[addr][payload[1]] = (value_type, value)
            return self._reply(addr, func, b"\x01")
        if func == mc.FUNC_SYS_PARAM and payload[:1] == b"\x03":
            return self._reply(addr, func, b"\x01")
        return None

    @staticmethod
    def _reply(addr, func, payload):
        body = bytes([addr, len(payload) + 3, 0x00, func]) + payload
        return bytes([0xF7]) + body + bytes([serial_485.crc8(body[1:])])


def make_axes(level):
    bus = FakeMotorBus()
    registry = mc.MotorParamRegistry.from_file(mc.DEFAULT_REGISTRY_PATH)
    axes = mc.MotorAxisController(
        registry, serial_transport=bus, extruder_transport=None
    )
    axes.serial_client.write_level = mc.WRITE_LEVEL_CHOICES[level]
    return bus, registry, axes


def is_write(frame):
    _addr, func, payload = frame
    return mc.frame_write_level(func, payload) > mc.WRITE_LEVEL_NONE


@pytest.mark.parametrize(
    "func,payload,level",
    [
        (mc.FUNC_READ_ADDR, b"\x01", "none"),
        (mc.FUNC_READ_ADDR, b"\x02", "none"),
        (mc.FUNC_GET, b"", "none"),
        (mc.FUNC_GET, b"\x11", "none"),
        (mc.FUNC_PROTECTION, b"\x0b", "none"),
        (mc.FUNC_SYS_PARAM, b"\x02\x29", "none"),
        (mc.FUNC_FLASH_PARAM, b"\x06\x29", "none"),
        (mc.FUNC_STALL_MODE, b"\x04", "none"),
        (mc.FUNC_STALL_MODE, b"\x01", "stall"),
        (mc.FUNC_STALL_MODE, b"\x02", "stall"),
        (mc.FUNC_PROTECTION, b"\x05", "stall"),
        (mc.FUNC_SYS_PARAM, b"\x01\x29\x00\x00\x00\x00", "params"),
        (mc.FUNC_SYS_PARAM, b"\x03", "params"),
    ],
)
def test_frame_write_level(func, payload, level):
    assert mc.frame_write_level(func, payload) == mc.WRITE_LEVEL_CHOICES[level]


@pytest.mark.parametrize(
    "func,payload",
    [
        (mc.FUNC_ENCODER_CALIBRATE, b""),
        (mc.FUNC_ELEC_OFFSET_CALIBRATE, b""),
        (mc.FUNC_FLASH_PARAM, b"\x05\x29\x00\x00\x00\x00"),
        (mc.FUNC_FLASH_PARAM, b"\x03"),
        (mc.FUNC_STALL_MODE, b"\x03"),
        (mc.FUNC_PROTECTION, b"\x00"),
        (mc.FUNC_SYS_PARAM, b""),
        (0x0F, b"\x00"),
        (0x55, b""),
    ],
)
def test_frames_never_sent(func, payload):
    assert mc.frame_write_level(func, payload) == mc.WRITE_LEVEL_NEVER
    for level in mc.WRITE_LEVEL_CHOICES:
        bus, _registry, axes = make_axes(level)
        with pytest.raises(mc.MotorWriteBlocked):
            axes.serial_client._acked_command(0x81, func, payload)
        with pytest.raises(mc.MotorWriteBlocked):
            axes.serial_client._no_ack_command(0x81, func, payload)
        assert bus.sent == []


def test_registry_uses_stock_hi_addresses():
    _bus, registry, axes = make_axes("none")
    assert sorted(registry.axis_addr) == ["x", "y"]
    assert registry.get_addr("x") == 0x81
    assert registry.get_addr("y") == 0x82
    assert axes.all_axes == ("x", "y")
    assert registry.get("x", "motor_param_R").param_id == 41
    with pytest.raises(RuntimeError):
        axes.target("e")
    with pytest.raises(RuntimeError):
        axes.target("z")


def test_address_straps_match_stock_addresses():
    # addr = 0x81 + DIR + 2 * STEP
    levels = dict(mc.PIN_DIR_SEQUENCE)
    for axis in ("x", "y"):
        addr = 0x81 + levels[f"motor_{axis}_dir"]
        addr += 2 * levels[f"motor_{axis}_step"]
        assert addr == int(MAP[axis]["_addr"], 16)


def test_level_none_reads_and_never_writes():
    bus, registry, axes = make_axes("none")
    axes.prepare_serial_bus()
    assert set(axes.probe_serial_axes()) == {"0x81", "0x82"}
    protection = axes.check_protection(("x", "y"))
    assert not any(detail["active"] for detail in protection.values())
    param = registry.get("x", "motor_param_R")
    assert axes.target("x").sys_param_read(param)["value"] == pytest.approx(0.6)
    sent = len(bus.sent)
    with pytest.raises(mc.MotorWriteBlocked):
        axes.set_normal_stall_mode()
    with pytest.raises(mc.MotorWriteBlocked):
        axes.set_homing_stall_mode(1)
    with pytest.raises(mc.MotorWriteBlocked):
        axes.clear_fault_latches(("x", "y"))
    with pytest.raises(mc.MotorWriteBlocked):
        axes.target("x").sys_param_write(param, 0.7)
    with pytest.raises(mc.MotorWriteBlocked):
        axes.target("x").sys_param_apply()
    assert len(bus.sent) == sent
    assert not any(is_write(frame) for frame in bus.sent)


def test_level_none_ignores_config_overrides():
    bus, registry, axes = make_axes("none")
    registry.apply_config_overrides({"x_param_stall_cur_A": "0.9"})
    result = axes.apply_overrides("x")
    assert result.writes == 0
    assert not any(is_write(frame) for frame in bus.sent)
    param = registry.get("x", "param_stall_cur_A")
    assert bus.params[0x81][param.param_id][1] == pytest.approx(0.7)


def test_level_stall_allows_stall_mode_but_not_params():
    bus, registry, axes = make_axes("stall")
    axes.set_normal_stall_mode()
    axes.set_homing_stall_mode(1)
    axes.clear_fault_latches(("x", "y"))
    param = registry.get("y", "param_stall_cur_A")
    with pytest.raises(mc.MotorWriteBlocked):
        axes.target("y").sys_param_write(param, 0.9)
    assert {frame[0] for frame in bus.sent} == {0x81, 0x82}
    assert not any(
        frame[1] in (mc.FUNC_SYS_PARAM, mc.FUNC_FLASH_PARAM)
        for frame in bus.sent
    )


def test_level_params_writes_only_differing_overrides():
    bus, registry, axes = make_axes("params")
    registry.apply_config_overrides(
        {"x_param_stall_cur_A": "0.9", "x_motor_param_R": "0.6"}
    )
    result = axes.apply_overrides("x")
    assert result.writes == 1 and result.errors == 0
    writes = [frame for frame in bus.sent if is_write(frame)]
    param = registry.get("x", "param_stall_cur_A")
    assert writes == [
        (
            0x81,
            mc.FUNC_SYS_PARAM,
            b"\x01" + bytes([param.param_id]) + struct.pack("<f", 0.9),
        ),
        (0x81, mc.FUNC_SYS_PARAM, b"\x03"),
    ]


@pytest.mark.parametrize(
    "level,steps",
    [
        ("none", ["serial_target_discovery", "protect_check"]),
        (
            "stall",
            [
                "serial_target_discovery",
                "stall_mode",
                "protect_check",
                "protect_clear",
                "protect_recheck",
            ],
        ),
        (
            "params",
            [
                "serial_target_discovery",
                "stall_mode",
                "protect_check",
                "protect_clear",
                "protect_recheck",
                "apply_overrides",
            ],
        ),
    ],
)
def test_startup_steps_per_level(level, steps):
    _bus, _registry, axes = make_axes(level)
    control = mc.MotorControl.__new__(mc.MotorControl)
    control.write_level = mc.WRITE_LEVEL_CHOICES[level]
    control.axes = axes
    control.managed_axes = ("x", "y")
    assert [name for name, _fn in control._startup_steps()] == steps


def test_homing_blocked_without_stall_level():
    control = mc.MotorControl.__new__(mc.MotorControl)
    control.write_level = mc.WRITE_LEVEL_NONE
    assert "startup_writes: stall" in control.get_homing_fault_blocker()


def test_write_commands_are_not_part_of_the_module():
    for name in (
        "cmd_MOTOR_FLASH_PARAM",
        "cmd_MOTOR_CALIBRATE",
        "cmd_CALIBRATE_CUT_POS",
        "cmd_REQUIRE_EXTRUDER_CLEAR",
    ):
        assert not hasattr(mc.MotorControl, name)
    assert not hasattr(mc, "NozzleTransparentTransportAdapter")
    assert not hasattr(mc.MotorFirmwareClient, "flash_param_save")
    assert not hasattr(mc.MotorFirmwareClient, "encoder_calibrate_official")


def phase2_config(tmp_path, extra=""):
    # The RS-485 port must not exist: the bus stays disconnected in batch
    # mode and motor_control_hi never starts talking.
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE2}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n" + extra
    )
    return config


def test_phase2_config_loads(tmp_path):
    gcode = tmp_path / "phase2.gcode"
    gcode.write_text("M105\nQUERY_ENDSTOPS\n")
    proc, out = run_batch(gcode, tmp_path, phase2_config(tmp_path))
    assert proc.returncode == 0, proc.stderr[-2000:]
    main = parse_output(out, "mcu")
    nozzle = parse_output(out, "nozzle_mcu")
    assert not any("transparent" in line for line in main + nozzle)
    assert any("config_ioRemap" in line for line in nozzle)
    buttons = [line for line in main if "buttons_add" in line]
    assert {
        field.split("=")[1]
        for line in buttons
        for field in line.split()
        if field.startswith("pin=")
    } == {"PA8", "PA6"}
    assert stepped_oids(main) == set()


@pytest.mark.parametrize(
    "gcode,error",
    [
        ("G28", "Only G28 X and G28 Y are enabled"),
        ("G28 Z", "Only G28 X and G28 Y are enabled"),
        ("G28 X Z", "Only G28 X and G28 Y are enabled"),
        ("G28 X", "startup_writes: stall"),
        ("G28 Y", "startup_writes: stall"),
        ("G1 X10 F600", "Must home axis first"),
    ],
)
def test_phase2_config_rejects_unsafe_moves(tmp_path, gcode, error):
    gcode_file = tmp_path / "unsafe.gcode"
    gcode_file.write_text(gcode + "\n")
    proc, out = run_batch(gcode_file, tmp_path, phase2_config(tmp_path))
    assert error in proc.stderr
    main = parse_output(out, "mcu")
    assert set(XY_STEP_PINS) <= set(stepper_oids(main))
    assert stepped_oids(main) == set()
