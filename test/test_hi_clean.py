# Batch-mode checks of the Hi nozzle cleaning macros (config/hi/clean.cfg).
# The probe cannot answer in batch mode, so PROBE is replaced by a counter
# and the homing by a known position: the tests cover the macro logic and
# the G-code it produces, not the probing itself.
from __future__ import annotations

from test_hi_config import ROOT, parse_output, run_batch

PHASE4 = ROOT / "config" / "hi" / "phase4.cfg"
# NOZZLE_WIPE: touch_cnt (2) rounds of 3 touches, one more, then the metal
# plate before and after the PTFE wipe. NOZZLE_QUICK_WIPE: before and after.
FULL_TOUCHES = 2 * 3 + 1 + 2
QUICK_TOUCHES = 2

STUBS = """
[homing_override]
gcode:
  SET_KINEMATIC_POSITION X=130 Y=130 Z=10

[gcode_macro PROBE]
rename_existing: _REAL_PROBE
variable_count: 0
gcode:
  SET_GCODE_VARIABLE MACRO=PROBE VARIABLE=count VALUE={count + 1}

[gcode_macro _REPORT]
gcode:
  {% set c = printer['gcode_macro _HI_CLEAN'] %}
  {% set p = printer.toolhead.position %}
  {action_raise_error("touches=%d clean=%d busy=%d x=%.1f y=%.1f z=%.1f t=%d"
    % (printer['gcode_macro PROBE'].count, c.is_clean, c.busy,
       p.x, p.y, p.z, printer.extruder.target))}
"""


def run(tmp_path, gcode, extra=""):
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE4}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n" + STUBS + extra
    )
    gcode_file = tmp_path / "test.gcode"
    gcode_file.write_text(gcode + "_REPORT\n")
    proc, out = run_batch(gcode_file, tmp_path, config)
    assert "Unknown command" not in proc.stderr
    assert "Error evaluating 'gcode_macro NOZZLE" not in proc.stderr
    return proc, out


def test_full_wipe_ends_sealed_and_restores_target(tmp_path):
    proc, out = run(tmp_path, "M104 S215\nNOZZLE_WIPE\n")
    # Ends over the bed centre, 10 mm above the metal plate reference, with
    # the print temperature target restored and the busy flag released.
    assert (
        f"touches={FULL_TOUCHES} clean=1 busy=0 x=130.0 y=130.0 z=10.0 t=215"
        in proc.stderr
    )
    # Z is referenced on the metal plate, 20 mm left of the PTFE pad.
    refs = [x for x in proc.stderr.splitlines() if "pos=128.000,268.000" in x]
    assert len(refs) == 2
    assert parse_output(out, "mcu")


def test_nozzle_clean_is_full_first_and_quick_afterwards(tmp_path):
    proc, _ = run(tmp_path, "NOZZLE_CLEAN\nNOZZLE_CLEAN\n")
    assert f"touches={FULL_TOUCHES + QUICK_TOUCHES} clean=1 busy=0" in (
        proc.stderr
    )


def test_fine_cleaning_can_be_enabled(tmp_path):
    # Fine cleaning (purge over the waste bin) adds no touches.
    extra = "[gcode_macro _HI_CLEAN]\nvariable_fine_enable: True\n"
    proc, _ = run(tmp_path, "NOZZLE_WIPE\n", extra)
    assert f"touches={FULL_TOUCHES} clean=1 busy=0" in proc.stderr


def test_guard_skips_cleaning_until_reset(tmp_path):
    # busy=1 is the state a cleaning that failed midway leaves behind (a
    # command error cannot be replayed here: it ends the batch run).
    stuck = "SET_GCODE_VARIABLE MACRO=_HI_CLEAN VARIABLE=busy VALUE=1\n"
    proc, _ = run(tmp_path, stuck + "NOZZLE_CLEAN\n")
    assert "touches=0 clean=0 busy=1" in proc.stderr
    proc, _ = run(tmp_path, stuck + "NOZZLE_CLEAN_RESET\nNOZZLE_CLEAN\n")
    assert f"touches={FULL_TOUCHES} clean=1 busy=0" in proc.stderr


def test_phase4_does_not_hook_cleaning_into_z_homing():
    # The macros call G28 Z: the fork homing code must not call
    # NOZZLE_CLEAN back (it does when register_as_probe is set).
    for name in ("phase3.cfg", "phase4.cfg"):
        lines = (ROOT / "config" / "hi" / name).read_text().splitlines()
        assert not [x for x in lines if x.startswith("register_as_probe")]


SEAL_STUB = """
[gcode_macro M109]
rename_existing: M109.1
variable_last: 0
gcode:
  SET_GCODE_VARIABLE MACRO=M109 VARIABLE=last VALUE={params.S|float}
  M104 S{params.S}

[gcode_macro _SEAL]
gcode:
  {action_respond_info("seal=%.0f" % printer['gcode_macro M109'].last)}
"""


def test_sealing_temperature_is_a_parameter(tmp_path):
    # The last blocking wait of NOZZLE_WIPE is the sealing temperature.
    proc, _ = run(tmp_path, "NOZZLE_WIPE\n_SEAL\n", SEAL_STUB)
    assert "seal=160" in proc.stderr
    proc, _ = run(tmp_path, "NOZZLE_CLEAN SEAL_TEMP=130\n_SEAL\n", SEAL_STUB)
    assert "seal=130" in proc.stderr
