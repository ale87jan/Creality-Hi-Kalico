# Batch-mode checks of the phase 3 config (pressure probe, Z homing) against
# the real MCU dictionaries of the Creality Hi (test/hi/*.dict).
from __future__ import annotations

import pytest
from test_hi_config import (
    ROOT,
    parse_output,
    run_batch,
    stepped_oids,
    stepper_oids,
)

PHASE3 = ROOT / "config" / "hi" / "phase3.cfg"
Z_STEP_PIN = "PB4"
Z1_STEP_PIN = "PB6"


def phase3_config(tmp_path):
    # The RS-485 port must not exist: the bus stays disconnected in batch
    # mode and motor_control_hi never starts talking.
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE3}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n"
    )
    return config


def fields_of(lines, command):
    found = [line for line in lines if command + " " in line]
    assert len(found) == 1, found
    return dict(f.split("=") for f in found[0].split() if "=" in f)


def test_phase3_config_loads_probe(tmp_path):
    gcode = tmp_path / "phase3.gcode"
    gcode.write_text("M105\nQUERY_PROBE\n")
    proc, out = run_batch(gcode, tmp_path, phase3_config(tmp_path))
    assert proc.returncode == 0, proc.stderr[-2000:]
    main = parse_output(out, "mcu")
    nozzle = parse_output(out, "nozzle_mcu")

    # Both Z motors are stopped by the probe trigger line on the main MCU.
    oids = stepper_oids(main)
    step = fields_of(main, "config_prtouch_step")
    assert step["swp_pin"] == "PC7"
    assert {step["oid_zstp"], step["oid_xstp"]} == {
        oids[Z_STEP_PIN],
        oids[Z1_STEP_PIN],
    }

    # The CS1237 load cell is read by the nozzle MCU (stock pins).
    pres = fields_of(nozzle, "config_prtouch_pres")
    assert pres["swp_pin"] == "PA15"
    assert pres["clk_pin"] == "PB13"
    assert pres["sdo_pin"] == "PB14"

    # The X endstop remap of phase 2 is still configured.
    assert any("config_ioRemap" in line for line in nozzle)
    assert stepped_oids(main) == set()


@pytest.mark.parametrize(
    "gcode,error",
    [
        ("G28", "startup_writes: stall"),
        ("G28 Z", "startup_writes: stall"),
        ("PROBE", "Must home before probe"),
        ("G1 Z5 F300", "Must home axis first"),
    ],
)
def test_phase3_config_moves_nothing_before_xy_homing(tmp_path, gcode, error):
    # With startup_writes: none X/Y cannot home, so nothing may step: not
    # the Z homing (it needs X/Y homed to reach the bed centre) and not a
    # bare PROBE.
    gcode_file = tmp_path / "unsafe.gcode"
    gcode_file.write_text(gcode + "\n")
    proc, out = run_batch(gcode_file, tmp_path, phase3_config(tmp_path))
    assert error in proc.stderr
    main = parse_output(out, "mcu")
    assert stepped_oids(main) == set()
