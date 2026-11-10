# Checks of the Creality Hi dual-Z alignment module (z_align_hi): batch-mode
# runs against the real main MCU dictionary (test/hi/mcu.dict) and unit
# tests of the retry and timeout logic with a simulated MCU.
from __future__ import annotations

import pytest
from test_hi_config import ROOT, parse_output, run_batch, stepped_oids

from klippy.extras import z_align_hi

ALIGN = ROOT / "config" / "hi" / "align.cfg"
PHASE4 = ROOT / "config" / "hi" / "phase4.cfg"
# gd32f303 main MCU clock, 32 microsteps on an 8 mm lead screw
CLOCK_FREQ = 120000000.0
STEP_DIST = 8.0 / (200 * 32)


def batch_config(tmp_path, base):
    # The RS-485 port must not exist: the bus stays disconnected in batch
    # mode and motor_control_hi never starts talking.
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {base}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n"
    )
    return config


def fields_of(line):
    return dict(f.split("=") for f in line.split() if "=" in f)


def run(tmp_path, base, gcode):
    gcode_file = tmp_path / "test.gcode"
    gcode_file.write_text(gcode)
    proc, out = run_batch(gcode_file, tmp_path, batch_config(tmp_path, base))
    return proc, parse_output(out, "mcu")


@pytest.mark.parametrize("base", [ALIGN, PHASE4])
def test_config_sends_stock_pins_and_polarity(tmp_path, base):
    proc, main = run(tmp_path, base, "M105\n")
    assert proc.returncode == 0, proc.stderr[-2000:]
    for name in ("config_z_align", "config_z_align_up"):
        adds = [fields_of(x) for x in main if name + "_add " in x]
        assert [a["z_indx"] for a in adds] == ["0", "1"]
        assert [a["zs_pin"] for a in adds] == ["PB4", "PB6"]
        assert [a["zd_pin"] for a in adds] == ["PA5", "PB5"]
        assert [a["zes_pin"] for a in adds] == ["PB15", "PB14"]
        assert {a["zd_up"] for a in adds} == {"0"}
        assert {a["zes_untrig"] for a in adds} == {"0"}
        oids = {a["oid"] for a in adds}
        assert len(oids) == 1
        assert sum(1 for x in main if x.startswith(name + " oid=")) == 1
    assert stepped_oids(main) == set()


def test_zdown_and_up_safe_z_requests(tmp_path):
    proc, main = run(tmp_path, ALIGN, "ZDOWN\nUP_SAFE_Z\n")
    assert proc.returncode == 0, proc.stderr[-2000:]
    down = [fields_of(x) for x in main if x.startswith("query_z_align ")]
    up = [fields_of(x) for x in main if x.startswith("query_z_align_up ")]
    assert len(down) == 1 and len(up) == 1
    # stock: 20 mm/s and 5 mm/s, 5 mm back-off, 15 mm safe rise, filter 10
    for req in down + up:
        assert req["enable"] == "1"
        assert int(req["quickSpeed"]) == int(STEP_DIST / 20 * CLOCK_FREQ)
        assert int(req["slowSpeed"]) == int(STEP_DIST / 5 * CLOCK_FREQ)
        assert req["filterCnt"] == "10"
    assert int(down[0]["risingDist"]) == int(5 / STEP_DIST)
    assert int(up[0]["risingDist"]) == int(15 / STEP_DIST)
    assert "ZDOWN done, Z set to 310.000" in proc.stderr
    # The host never steps Z itself during an alignment.
    assert stepped_oids(main) == set()


def test_align_config_keeps_z_homing_blocked(tmp_path):
    proc, main = run(tmp_path, ALIGN, "G28 Z\n")
    assert "Only G28 X and G28 Y are enabled" in proc.stderr
    assert not any("query_z_align" in x for x in main)
    assert stepped_oids(main) == set()


def test_phase4_g28_lifts_then_stops_without_xy_homing(tmp_path):
    # With startup_writes: none X/Y cannot home: the macro lifts the gantry
    # (MCU move, away from the bed) and stops there. No ZDOWN, no probing.
    proc, main = run(tmp_path, PHASE4, "G28\n")
    assert "startup_writes: stall" in proc.stderr
    assert sum(1 for x in main if x.startswith("query_z_align_up ")) == 1
    assert not any(x.startswith("query_z_align ") for x in main)
    assert stepped_oids(main) == set()


# --- simulated MCU ---------------------------------------------------------


class FakeReactor:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = max(self.now, waketime)
        return self.now


class FakeMcu:
    def is_fileoutput(self):
        return False

    def get_constant_float(self, name):
        assert name == "CLOCK_FREQ"
        return CLOCK_FREQ


class FakeCommand:
    "Answers each request with the next scripted deltaError1 (None: never)"

    def __init__(self, align, oid, deltas):
        self.align, self.oid, self.deltas = align, oid, list(deltas)
        self.sent = []

    def send(self, args):
        self.sent.append(list(args))
        delta = self.deltas.pop(0)
        if delta is None:
            self.align._handle_status({"oid": self.oid, "flag": 0})
        else:
            self.align._handle_status(
                {"oid": self.oid, "flag": 1, "deltaError1": delta}
            )


class FakeGcmd:
    class error(Exception):
        pass

    def __init__(self):
        self.info = []

    def respond_info(self, msg):
        self.info.append(msg)


class FakeToolhead:
    def __init__(self):
        self.pos = [10.0, 20.0, 33.0, 0.0]
        self.homed = ""

    def get_position(self):
        return list(self.pos)

    def set_position(self, pos, homing_axes=""):
        self.pos = list(pos)
        self.homed = "".join(sorted(set(self.homed) | set(homing_axes)))

    def get_status(self, eventtime):
        return {"homed_axes": self.homed}

    def get_kinematics(self):
        return self

    def clear_homing_state(self, axes):
        self.homed = "".join(a for a in self.homed if a not in axes)

    def wait_moves(self):
        pass


class FakeHeater:
    def get_status(self, eventtime):
        return {"target": 215.0}


class FakeExtruder:
    def get_heater(self):
        return FakeHeater()


class FakeStepperEnable:
    def __init__(self, toolhead):
        self.toolhead, self.off = toolhead, 0

    def motor_off(self):
        self.off += 1
        self.toolhead.homed = ""


class FakeGcode:
    def __init__(self):
        self.scripts = []

    def run_script_from_command(self, script):
        self.scripts.append(script)


class FakePrinter:
    def __init__(self, objects):
        self.objects = objects

    def lookup_object(self, name):
        return self.objects[name]


def make_align(deltas, retries=7):
    align = object.__new__(z_align_hi.ZAlignHi)
    toolhead = FakeToolhead()
    align.reactor = FakeReactor()
    align.gcode = FakeGcode()
    align.mcu = FakeMcu()
    align.stepper_enable = FakeStepperEnable(toolhead)
    align.printer = FakePrinter(
        {
            "toolhead": toolhead,
            "extruder": FakeExtruder(),
            "stepper_enable": align.stepper_enable,
        }
    )
    align.quick_speed, align.slow_speed = 20.0, 5.0
    align.rising_dist, align.safe_rising_dist = 5.0, 15.0
    align.filter_cnt, align.timeout = 10, 30.0
    align.retries, align.retry_tolerance = retries, 10
    align.zmax = 310.0
    align.step_dist = STEP_DIST
    align.oid_down, align.oid_up = 2, 3
    align.status = {}
    align.limit_state = [False, False]
    align.zdown_switch_enable = 1
    align.aligned = False
    align.last_delta = None
    align.cmd_down = FakeCommand(align, 2, deltas)
    align.cmd_up = FakeCommand(align, 3, deltas)
    return align, toolhead


def test_zdown_retries_until_within_tolerance():
    align, toolhead = make_align([40, -12, 9])
    gcmd = FakeGcmd()
    align.cmd_ZDOWN(gcmd)
    assert len(align.cmd_down.sent) == 3
    assert align.aligned and align.last_delta == 9
    # Z is homed at zmax with the gantry at the top; X/Y are untouched.
    assert toolhead.pos == [10.0, 20.0, 310.0, 0.0]
    assert toolhead.homed == "z"
    # The hotend target is switched off during the move and then restored.
    assert align.gcode.scripts[0] == "M104 S0"
    assert align.gcode.scripts[-1] == "M104 S215.0"
    assert align.stepper_enable.off == 0


def test_zdown_gives_up_after_retries_and_forgets_z():
    align, toolhead = make_align([40, 40, 40], retries=3)
    with pytest.raises(FakeGcmd.error, match="not aligned after 3 attempts"):
        align.cmd_ZDOWN(FakeGcmd())
    assert len(align.cmd_down.sent) == 3
    assert not align.aligned
    assert toolhead.homed == ""
    assert toolhead.pos[2] == 33.0
    assert align.gcode.scripts[-1] == "M104 S215.0"


def test_zdown_timeout_switches_motors_off():
    align, toolhead = make_align([None])
    with pytest.raises(FakeGcmd.error, match="Motors are off"):
        align.cmd_ZDOWN(FakeGcmd())
    assert align.reactor.now > align.timeout
    assert align.stepper_enable.off == 1
    assert not align.aligned
    assert toolhead.homed == ""
    assert align.gcode.scripts[-1] == "M104 S215.0"


def test_zdown_refuses_when_both_switches_are_triggered():
    align, toolhead = make_align([0])
    align.limit_state = [True, True]
    with pytest.raises(FakeGcmd.error, match="both Z limit switches"):
        align.cmd_ZDOWN(FakeGcmd())
    assert align.cmd_down.sent == []
    assert align.gcode.scripts == []


def test_check_zlimit_backs_off_once_then_fails():
    align, toolhead = make_align([0])
    align.limit_state = [False, True]
    with pytest.raises(FakeGcmd.error, match="stepper_z1"):
        align.cmd_CHECK_ZLIMIT(FakeGcmd())
    assert align.gcode.scripts == [
        "FORCE_MOVE STEPPER=stepper_z DISTANCE=-0.500 VELOCITY=10.0",
        "FORCE_MOVE STEPPER=stepper_z1 DISTANCE=-0.500 VELOCITY=10.0",
    ]


def test_up_safe_z_leaves_z_unhomed():
    align, toolhead = make_align([0])
    align.cmd_UP_SAFE_Z(FakeGcmd())
    assert align.cmd_up.sent[0][0] == 3
    assert align.cmd_up.sent[0][4] == int(15 / STEP_DIST)
    assert toolhead.homed == ""


def test_stale_status_is_not_reused():
    align, toolhead = make_align([5, None])
    align.cmd_ZDOWN(FakeGcmd())
    with pytest.raises(FakeGcmd.error, match="Motors are off"):
        align.cmd_ZDOWN(FakeGcmd())
