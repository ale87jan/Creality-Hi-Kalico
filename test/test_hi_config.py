# Batch-mode checks of the Creality Hi configs against the real MCU
# dictionaries of the printer (test/hi/*.dict).
from __future__ import annotations

import pathlib
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).parent.parent
HI = ROOT / "test" / "hi"
CONFIG = ROOT / "config" / "hi" / "printer.cfg"
MCUS = ("mcu", "nozzle_mcu", "bed_mcu")
# Closed-loop X/Y drivers: nothing may ever step them in phase 1.
XY_STEP_PINS = ("PB10", "PB8")


def run_batch(
    gcode: pathlib.Path, tmp_path: pathlib.Path, config: pathlib.Path = CONFIG
):
    out = tmp_path / "out"
    cmd = [sys.executable, "-m", "klippy", str(config)]
    cmd += ["-i", str(gcode), "-o", str(out), "-v"]
    cmd += ["-d", str(HI / "mcu.dict")]
    for name in MCUS[1:]:
        cmd += ["-d", f"{name}={HI / (name + '.dict')}"]
    proc = subprocess.run(
        cmd,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    return proc, out


def parse_output(out: pathlib.Path, mcu: str) -> list[str]:
    path = out if mcu == "mcu" else out.with_name(f"{out.name}-{mcu}")
    cmd = [sys.executable, "-m", "klippy.parsedump"]
    cmd += [str(HI / (mcu + ".dict")), str(path)]
    proc = subprocess.run(
        cmd,
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    return proc.stdout.splitlines()


def stepper_oids(lines: list[str]) -> dict[str, str]:
    "Map step pin -> oid from the config_stepper commands"
    oids = {}
    for line in lines:
        if "config_stepper " in line:
            fields = dict(f.split("=") for f in line.split()[1:] if "=" in f)
            oids[fields["step_pin"]] = fields["oid"]
    return oids


def stepped_oids(lines: list[str]) -> set[str]:
    return {
        f.split("=")[1]
        for line in lines
        if "queue_step " in line
        for f in line.split()
        if f.startswith("oid=")
    }


def test_phase1_config_loads_and_moves_only_z(tmp_path):
    proc, out = run_batch(HI / "phase1.gcode", tmp_path)
    assert proc.returncode == 0, proc.stderr[-2000:]
    main = parse_output(out, "mcu")
    oids = stepper_oids(main)
    assert set(oids) == {"PB10", "PB8", "PB4", "PB6"}
    stepped = stepped_oids(main)
    assert stepped == {oids["PB4"], oids["PB6"]}
    nozzle = parse_output(out, "nozzle_mcu")
    assert any("config_analog_in" in line and "PA0" in line for line in nozzle)
    assert parse_output(out, "bed_mcu")


@pytest.mark.parametrize(
    "gcode,error",
    [
        ("G28", "Homing is disabled in the phase 1 config"),
        ("G28 Z", "Homing is disabled in the phase 1 config"),
        ("G1 X10 F600", "Must home axis first"),
        ("G1 Y10 F600", "Must home axis first"),
        (
            "SET_KINEMATIC_POSITION Z=50 SET_HOMED=z\nG1 X10 Z51 F600",
            "Must home axis first",
        ),
    ],
)
def test_phase1_config_rejects_homing_and_xy(tmp_path, gcode, error):
    # The pytest session installs the klippy testing plugin, which turns a
    # g-code error into a shutdown: only the message and the absence of
    # steps are checked, not the exit code.
    gcode_file = tmp_path / "unsafe.gcode"
    gcode_file.write_text(gcode + "\n")
    proc, out = run_batch(gcode_file, tmp_path)
    assert error in proc.stderr
    main = parse_output(out, "mcu")
    assert set(XY_STEP_PINS) <= set(stepper_oids(main))
    assert stepped_oids(main) == set()


def stock_bed_correction(raw):
    "Bed temperature shown by the stock firmware for a raw thermistor value"
    # [heater_bed] adc_temp_list / actual_temp_list of the printer's
    # V1.1.0.60 config, applied as stock adc_temperature.py does: identity
    # up to the first point, the last segment extended above.
    adc = [25.0, 40.0, 53.0, 69.0, 83.0, 108.0]
    actual = [25.0, 38.0, 50.0, 65.0, 80.0, 110.0]
    if raw <= adc[0]:
        return raw
    i = max(j for j in range(5) if raw >= adc[j])
    slope = (actual[i + 1] - actual[i]) / (adc[i + 1] - adc[i])
    return (raw - adc[i]) * slope + actual[i]


def test_bed_sensor_table_follows_the_stock_correction():
    import configparser

    from klippy.extras import adc_temperature, thermistor

    def section(path, name):
        parser = configparser.RawConfigParser(
            strict=False, inline_comment_prefixes=("#",)
        )
        parser.read(path)
        return dict(parser.items(name))

    base = ROOT / "config" / "hi" / "base.cfg"
    assert section(base, "heater_bed")["sensor_type"] == "hi_bed"
    table = section(base, "adc_temperature hi_bed")
    count = len(table) // 2
    samples = [
        (float(table[f"resistance{i}"]), float(table[f"temperature{i}"]))
        for i in range(1, count + 1)
    ]
    assert len(table) == 2 * count and count >= 30
    corrected = adc_temperature.LinearInterpolate(samples)

    epcos = section(
        ROOT / "klippy" / "extras" / "temperature_sensors.cfg",
        "thermistor EPCOS 100K B57560G104F",
    )
    pullup = 4700.0
    raw = thermistor.Thermistor(pullup, 0.0)
    raw.setup_coefficients(
        *[
            float(epcos[f"{key}{i}"])
            for i in (1, 2, 3)
            for key in ("temperature", "resistance")
        ],
        name="EPCOS",
    )
    for tenth in range(-150, 1250):
        temp = tenth / 10.0
        adc = raw.calc_adc(temp)
        resistance = pullup * adc / (1.0 - adc)
        shown = corrected.interpolate(resistance)
        assert shown == pytest.approx(stock_bed_correction(temp), abs=0.25)
