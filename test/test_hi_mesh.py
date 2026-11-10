# Batch-mode checks of the user's bed heating and bed mesh macros ported to
# the Hi (config/hi/heat.cfg, config/hi/mesh.cfg). Probing, homing and the
# config save cannot run in batch mode: they are replaced by recorders, so
# the tests cover the macro logic and its order.
from __future__ import annotations

from test_hi_config import ROOT, run_batch

PHASE6 = ROOT / "config" / "hi" / "phase6.cfg"


def _stub(name):
    return f"""
[gcode_macro {name}]
rename_existing: _REAL_{name}
gcode:
  {{% set log = printer['gcode_macro _REPORT'].log %}}
  SET_GCODE_VARIABLE MACRO=_REPORT VARIABLE=log VALUE='"{{log}} {name}({{rawparams}})"'
"""


STUBS = (
    """
[homing_override]
gcode:
  {% set log = printer['gcode_macro _REPORT'].log %}
  SET_GCODE_VARIABLE MACRO=_REPORT VARIABLE=log VALUE='"{log} G28({rawparams})"'
  SET_KINEMATIC_POSITION X=130 Y=130 Z=10

[gcode_macro NOZZLE_WIPE]
gcode:
  {% set log = printer['gcode_macro _REPORT'].log %}
  SET_GCODE_VARIABLE MACRO=_REPORT VARIABLE=log VALUE='"{log} NOZZLE_WIPE"'

[gcode_macro _REPORT]
variable_log: ""
gcode:
  {action_raise_error("log=%s bed=%.0f nozzle=%.0f accel=%.0f stage=%d"
    % (log, printer.heater_bed.target, printer.extruder.target,
       printer.toolhead.max_accel,
       printer['gcode_macro _SMART_BED_VARS'].stage))}
"""
    + _stub("BED_MESH_CALIBRATE")
    + _stub("BED_MESH_PROFILE")
    + _stub("Z_TILT_ADJUST")
    + _stub("SAVE_CONFIG")
)


def run(tmp_path, gcode):
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE6}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n" + STUBS
    )
    gcode_file = tmp_path / "test.gcode"
    gcode_file.write_text(gcode + "_REPORT\n")
    proc, _ = run_batch(gcode_file, tmp_path, config)
    assert "Unknown command" not in proc.stderr
    assert "Error evaluating 'gcode_macro _REPORT" in proc.stderr
    return proc.stderr


def test_m140_boosts_and_m190_ends_on_the_real_target(tmp_path):
    # M140 asks for target + boost (10) and arms the monitor (stage 1).
    assert "bed=70 nozzle=0" in run(tmp_path, "M140 S60\n")
    assert "stage=1" in run(tmp_path, "M140 S60\n")
    # The boost never goes above the bed limit.
    assert "bed=100 " in run(tmp_path, "M140 S95\n")
    # M190 waits and leaves the real target; M140 S0 switches off.
    assert "bed=60 " in run(tmp_path, "M190 S60\n")
    assert "bed=0 nozzle=0" in run(tmp_path, "M140 S60\nM140 S0\n")


def test_missing_profile_is_probed_once_and_loaded(tmp_path):
    err = run(tmp_path, "_MESH_IF_NEEDED BED_TEMP=60 SOAK_TIME=0\n")
    assert (
        "log= G28(X Y) G28(Z) NOZZLE_WIPE G28(Z ALIGN=1) Z_TILT_ADJUST()"
        " G28(Z) BED_MESH_CALIBRATE(PROFILE=60.0c) SAVE_CONFIG(RESTART=0)"
        " BED_MESH_PROFILE(LOAD=60.0c) bed=60 nozzle=140"
    ) in err
    # The acceleration lowered for the probing is restored (motors.cfg).
    assert " accel=12000 " in err


def test_mesh_keeps_the_stock_grid_and_centre_reference():
    cfg = (ROOT / "config" / "hi" / "mesh.cfg").read_text()
    assert "\nprobe_count: 9, 9\n" in cfg
    assert "\nzero_reference_position: 130, 130\n" in cfg
