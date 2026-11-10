# Checks of the Hi motor cable check (tmc_line_check): a batch-mode run with
# the real MCU dictionaries and unit tests of the fault logic with simulated
# drivers.
from __future__ import annotations

import pytest
from test_hi_config import ROOT, parse_output, run_batch, stepper_oids

from klippy.extras import tmc_line_check

ALIGN = ROOT / "config" / "hi" / "align.cfg"
OLA = 1 << 6  # open load, phase A
STST = 1 << 31  # standstill: not a fault


def test_batch_moves_both_z_motors_and_back(tmp_path):
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {ALIGN}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n"
    )
    gcode = tmp_path / "test.gcode"
    gcode.write_text("CHECK_ZMOTOR_LINE\nREAD_REG_TMC STEP=z1 A=111\n")
    proc, out = run_batch(gcode, tmp_path, config)
    assert proc.returncode == 0, proc.stderr[-2000:]
    main = parse_output(out, "mcu")
    oids = stepper_oids(main)
    # 0.5 mm at 0.00125 mm per step, there and back, on each Z motor.
    for pin in ("PB4", "PB6"):
        steps = [
            int(x.split("count=")[1].split()[0])
            for x in main
            if x.startswith(f"queue_step oid={oids[pin]} ")
        ]
        assert sum(steps) == 2 * 400


class FakeTmc:
    name_to_reg = {"GCONF": 0x00, "DRV_STATUS": 0x6F}

    def __init__(self, value):
        self.value, self.read = value, []

    def get_register(self, name):
        self.read.append(name)
        return self.value


class FakeDriver:
    def __init__(self, value):
        self.mcu_tmc = FakeTmc(value)


class FakeGcode:
    def __init__(self):
        self.scripts, self.info = [], []

    def register_command(self, *args, **kwargs):
        pass

    def run_script_from_command(self, script):
        self.scripts.append(script)

    def respond_info(self, msg):
        self.info.append(msg)


class FakeReactor:
    now = 0.0

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.now = waketime


class FakeToolhead:
    def get_last_move_time(self):
        return 0.0


class FakePrinter:
    class command_error(Exception):
        pass

    def __init__(self, values):
        self.gcode = FakeGcode()
        self.objects = {"gcode": self.gcode, "toolhead": FakeToolhead()}
        for stepper, value in values.items():
            self.objects[f"tmc2208 stepper_{stepper}"] = FakeDriver(value)
        self.reactor = FakeReactor()
        self.shutdown = []

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def get_reactor(self):
        return self.reactor

    def is_shutdown(self):
        return bool(self.shutdown)

    def invoke_shutdown(self, msg):
        self.shutdown.append(msg)


class FakeConfig:
    def __init__(self, printer):
        self.printer = printer

    def get_printer(self):
        return self.printer


class FakeGcmd:
    class error(Exception):
        pass

    def __init__(self, **params):
        self.params, self.info = params, []

    def get(self, name, default=None):
        return self.params.get(name, default)

    def get_int(self, name, default=None, minval=None, maxval=None):
        return int(self.params.get(name, default))

    def respond_info(self, msg):
        self.info.append(msg)


def make(**values):
    printer = FakePrinter(values)
    return printer, tmc_line_check.load_config(FakeConfig(printer))


def moves(printer):
    return [
        (
            s.split("STEPPER=stepper_")[1].split()[0],
            s.split("DISTANCE=")[1].split()[0],
        )
        for s in printer.gcode.scripts
        if s.startswith("FORCE_MOVE")
    ]


def test_z_check_passes_and_ignores_standstill_flag():
    printer, check = make(z=STST, z1=0)
    gcmd = FakeGcmd()
    check.cmd_CHECK_ZMOTOR_LINE(gcmd)
    assert not printer.shutdown
    assert moves(printer) == [
        ("z", "0.5"),
        ("z1", "0.5"),
        ("z", "-0.5"),
        ("z1", "-0.5"),
    ]
    assert printer.gcode.scripts[-1] == "M84"
    assert gcmd.info == ["Z and Z1 motor lines are inserted ok"]


@pytest.mark.parametrize(
    "values, failed",
    [
        ({"z": OLA, "z1": 0}, "Z"),
        ({"z": 0, "z1": OLA}, "Z1"),
        ({"z": OLA, "z1": 1}, "Z-Z1"),
    ],
)
def test_z_check_reports_the_failed_motor_after_moving_back(values, failed):
    printer, check = make(**values)
    with pytest.raises(FakePrinter.command_error, match=f"{failed} motor line"):
        check.cmd_CHECK_ZMOTOR_LINE(FakeGcmd())
    # Unlike stock, each motor goes back exactly once, then motors off.
    assert moves(printer)[2:] == [("z", "-0.5"), ("z1", "-0.5")]
    assert printer.gcode.scripts[-1] == "M84"
    assert len(printer.shutdown) == 1


def test_single_motor_check_stops_without_moving_back_on_fault():
    printer, check = make(z=OLA)
    with pytest.raises(FakePrinter.command_error, match="Z motor line"):
        check.cmd_CHECK_MOTOR_LINE(FakeGcmd(STEP="z"))
    assert moves(printer) == [("z", "0.5")]
    assert printer.shutdown


def test_read_reg_maps_the_stock_address_to_a_register_name():
    printer, check = make(z1=0x12345678)
    check.cmd_READ_REG_TMC(FakeGcmd(STEP="z1", A=111))
    assert printer.objects["tmc2208 stepper_z1"].mcu_tmc.read == ["DRV_STATUS"]
    assert printer.gcode.info == ["tmc2208 stepper_z1 DRV_STATUS: 0x12345678"]
    with pytest.raises(FakeGcmd.error):
        check.cmd_READ_REG_TMC(FakeGcmd(STEP="z1", A=5))


def test_unknown_stepper_is_refused_before_any_move():
    printer, check = make(z=0)
    with pytest.raises(FakePrinter.command_error, match="stepper_z1"):
        check.cmd_CHECK_ZMOTOR_LINE(FakeGcmd())
    assert moves(printer) == []
