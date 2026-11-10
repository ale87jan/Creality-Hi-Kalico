# Batch-mode checks of the user's print start/end macros ported to the Hi
# (config/hi/print.cfg). Homing, cleaning, the mesh and the CFS are replaced
# by recorders: the tests cover the order of the macro and the final state.
from __future__ import annotations

from test_hi_config import ROOT, run_batch

PHASE6 = ROOT / "config" / "hi" / "phase6.cfg"


def _rec(name, params="", header="", extra=""):
    return f"""
[gcode_macro {name}]
{header}
gcode:
  {{% set log = printer['gcode_macro _REPORT'].log %}}
  SET_GCODE_VARIABLE MACRO=_REPORT VARIABLE=log VALUE='"{{log}} {name}{params}"'
  {extra}
"""


STUBS = (
    """
[homing_override]
gcode:
  {% set log = printer['gcode_macro _REPORT'].log %}
  SET_GCODE_VARIABLE MACRO=_REPORT VARIABLE=log VALUE='"{log} G28({rawparams})"'
  SET_KINEMATIC_POSITION X=130 Y=130 Z=10

[gcode_macro _REPORT]
variable_log: ""
gcode:
  {% set p = printer.toolhead.position %}
  {action_raise_error("log=%s bed=%.0f nozzle=%.0f pos=%.0f,%.0f,%.0f"
    % (log, printer.heater_bed.target, printer.extruder.target,
       p.x, p.y, p.z))}
"""
    + _rec("NOZZLE_WIPE")
    + _rec("NOZZLE_QUICK_WIPE")
    + _rec("_MESH_IF_NEEDED", "({rawparams})")
    + _rec("_HI_CFS_GO_BIN")
    + _rec("CFS_RETRUDE", "({rawparams})", "rename_existing: _REAL_CFS_RETRUDE")
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


START = "START_PRINT BED_TEMP=60 EXTRUDER_TEMP=215\n"
PREPARE = (
    " _MESH_IF_NEEDED(BED_TEMP=60.0) G28(X Y) NOZZLE_WIPE"
    " G28(Z ALIGN=1) G28(Z) _HI_CFS_GO_BIN"
)


def test_start_print_prepares_once_and_ends_at_print_temperatures(tmp_path):
    err = run(tmp_path, START)
    assert f"log={PREPARE} bed=60 nozzle=215" in err
    # A second start in the same session skips homing and cleaning.
    err = run(tmp_path, START + START)
    assert (
        f"log={PREPARE} _MESH_IF_NEEDED(BED_TEMP=60.0) _HI_CFS_GO_BIN bed=60"
    ) in err


def test_start_print_prepares_again_after_the_motors_are_released(tmp_path):
    err = run(tmp_path, START + "M84\n" + START)
    assert f"log={PREPARE}{PREPARE} bed=60" in err


def test_end_print_lifts_parks_and_switches_the_heaters_off(tmp_path):
    err = run(tmp_path, START + "G1 X100 Y100 Z20 F6000\nEND_PRINT\n")
    # Z + 50, bed to the front, heaters off; the CFS unload is delayed.
    assert "bed=0 nozzle=0 pos=0,260,70" in err
    assert "CFS_RETRUDE" not in err.split("log=")[-1]


def test_print_end_unloads_only_a_loaded_filament(tmp_path):
    # The module reports no loaded tool after start-up: nothing to unload.
    err = run(tmp_path, "_HI_CFS_PRINT_END\n")
    assert "log= bed=0" in err


def test_no_creality_only_command_is_left():
    cfg = (ROOT / "config" / "hi" / "print.cfg").read_text()
    live = [x.split("#")[0].strip() + " " for x in cfg.splitlines()]
    for name in (
        "BOX_START_PRINT",
        "BOX_END_PRINT",
        "BOX_END",
        "NEXT_HOMEZ_NACCU",
        "FORCE_MOVE_E",
        "ZDOWN_SWITCH",
        "ENABLE_PRESSURE_ADVANCE",
    ):
        assert not any(x.startswith(name + " ") for x in live), name
    assert "custom_macro" not in "".join(live)
