# Batch-mode check of the [io_remap] module against the real nozzle MCU
# dictionary of the Creality Hi (test/hi/nozzle_mcu.dict).
from __future__ import annotations

from test_hi_config import CONFIG, parse_output, run_batch

# Values of the stock V1.1.0.60 printer.cfg
IO_REMAP = """
[include {config}]

[io_remap]
src_pin: PB0
remap_pin: PA15
src_pullup: 1
remap_def: 1
filterNum: 1
periodTicks: 0
"""


def test_io_remap_configures_nozzle_mcu(tmp_path):
    config = tmp_path / "printer.cfg"
    config.write_text(IO_REMAP.format(config=CONFIG))
    gcode = tmp_path / "io_remap.gcode"
    gcode.write_text("M105\n")
    proc, out = run_batch(gcode, tmp_path, config)
    assert proc.returncode == 0, proc.stderr[-2000:]
    lines = [
        line
        for line in parse_output(out, "nozzle_mcu")
        if "config_ioRemap" in line
    ]
    assert len(lines) == 1
    fields = dict(f.split("=") for f in lines[0].split() if "=" in f)
    assert fields["src_pin"] == "PB0"
    assert fields["remap_pin"] == "PA15"
    assert fields["src_pullup"] == "1"
    assert fields["remap_def"] == "1"
    assert not any(
        "config_ioRemap" in line for line in parse_output(out, "mcu")
    )


def test_set_ioremap_sends_operation(tmp_path):
    config = tmp_path / "printer.cfg"
    config.write_text(IO_REMAP.format(config=CONFIG))
    gcode = tmp_path / "io_remap.gcode"
    gcode.write_text("SET_IOREMAP S=1\nSET_IOREMAP S=0\n")
    proc, out = run_batch(gcode, tmp_path, config)
    assert proc.returncode == 0, proc.stderr[-2000:]
    operations = [
        dict(f.split("=") for f in line.split() if "=" in f)["operation"]
        for line in parse_output(out, "nozzle_mcu")
        if "operation_ioRemap" in line
    ]
    assert operations == ["1", "0"]
