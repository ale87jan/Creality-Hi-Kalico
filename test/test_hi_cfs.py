# Checks of the shared-bus transport added to creality_cfs (the open CFS
# module of gitstonelabs/creality-cfs-klipper) for the Creality Hi: the
# module must reach the CFS through [serial_485], never open the port, and
# never talk to the X/Y motor drivers that share the bus.
from __future__ import annotations

import pytest
from test_hi_config import ROOT, parse_output, run_batch, stepped_oids

from klippy.extras import creality_cfs, serial_485

PHASE5 = ROOT / "config" / "hi" / "phase5.cfg"
BOX = 0x01


def test_phase5_config_loads_and_sends_nothing_at_startup(tmp_path):
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE5}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n"
    )
    gcode = tmp_path / "test.gcode"
    gcode.write_text("M105\nCFS_ADDR_TABLE\n")
    proc, out = run_batch(gcode, tmp_path, config)
    assert proc.returncode == 0, proc.stderr[-2000:]
    assert "using shared bus [serial_485 serial485]" in proc.stderr
    # auto_init is off in cfs.cfg: no addressing, no pre-load at start-up.
    assert "auto-init" not in proc.stderr
    assert "TX addr" not in proc.stderr
    assert stepped_oids(parse_output(out, "mcu")) == set()


class FakeLock:
    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False


class FakeBus:
    "Stands in for serial_485: records request bodies, replays replies"

    def __init__(self, replies=()):
        self.replies, self.sent = list(replies), []

    def cmd_send_data_with_response(self, data, timeout=1.0, attempts=1):
        self.sent.append((bytes(data), timeout))
        reply = self.replies.pop(0) if self.replies else None
        if isinstance(reply, Exception):
            raise reply
        return reply


def make(replies=()):
    cfs = object.__new__(creality_cfs.CrealityCFS)
    cfs._bus = FakeBus(replies)
    cfs._fd = None
    cfs._bus_lock = FakeLock()
    cfs._shutdown = False
    cfs.is_connected = True
    cfs.timeout = 0.1
    cfs.retry_count = 3
    return cfs


@pytest.mark.parametrize(
    "addr, status, func, data",
    [
        (BOX, 0xFF, 0x0A, b""),
        (BOX, 0xFF, 0x10, bytes([0x02, 0x07, 0x03])),
        (0xFE, 0x00, 0xA1, bytes([0xFE, 0xFE])),
        (0xFE, 0x00, 0xA0, bytes([0x01]) + bytes(range(12))),
    ],
)
def test_shared_bus_puts_the_same_frame_on_the_wire(addr, status, func, data):
    # serial_485 takes the frame without head and CRC and adds both itself:
    # the result must be byte for byte the frame the module builds.
    frame = creality_cfs.build_message(addr, status, func, data)
    cfs = make()
    assert cfs._txn(frame, 0.5) is None
    body, timeout = cfs._bus.sent[0]
    assert timeout == 0.5
    assert body == frame[1:-1]
    assert body == serial_485.build_485_body(addr, func, data, status)
    wire = bytes([serial_485.PACK_HEAD]) + body
    wire += bytes([serial_485.crc8(body[1:])])
    assert wire == frame


def test_reply_from_the_shared_bus_is_parsed_as_upstream():
    # Box state as captured on a Hi: four data bytes, b3 = 0x02 (loaded).
    reply = creality_cfs.build_message(
        BOX, 0x00, 0x0A, bytes([0x1A, 0x20, 0x00, 0x02])
    )
    cfs = make([reply])
    parsed = cfs._send_command(BOX, 0xFF, 0x0A)
    assert parsed["crc_valid"] and parsed["addr"] == BOX
    assert parsed["data"] == bytes([0x1A, 0x20, 0x00, 0x02])
    assert len(cfs._bus.sent) == 1


def test_timeouts_are_retried_and_bus_errors_are_not():
    cfs = make([None, None, None])
    assert cfs._send_command(BOX, 0xFF, 0x05) is None
    assert len(cfs._bus.sent) == 3

    cfs = make([RuntimeError("serial485 session is not active")])
    assert cfs._send_command(BOX, 0xFF, 0x05) is None
    assert len(cfs._bus.sent) == 1


def test_nothing_is_sent_to_the_motor_drivers():
    cfs = make()
    for driver in (0x81, 0x82):
        assert cfs.read_buffer_block_0x0c(driver) is None
    assert cfs._bus.sent == []


def test_module_never_opens_the_port_on_the_shared_bus(monkeypatch):
    def refuse(*args, **kwargs):
        raise AssertionError("the tty must not be opened")

    monkeypatch.setattr(creality_cfs.os, "open", refuse)
    monkeypatch.setattr(creality_cfs.os, "write", refuse)

    class Printer:
        def lookup_object(self, name, default=None):
            return bus if name == "serial_485 serial485" else default

    bus = FakeBus()
    cfs = make()
    cfs._bus = None
    cfs.is_connected = False
    cfs.printer = Printer()
    cfs.serial_name = "serial_485 serial485"
    cfs._connect_serial()
    assert cfs._bus is bus and cfs.is_connected and cfs._fd is None
    cfs._txn(creality_cfs.build_message(BOX, 0xFF, 0x05), 0.1)
    assert len(bus.sent) == 1

    cfs.serial_name = "serial_485 missing"
    with pytest.raises(RuntimeError, match="serial_485 missing"):
        cfs._connect_serial()


PURGE_STUBS = """
[homing_override]
gcode:
  SET_KINEMATIC_POSITION X=130 Y=200 Z=10

[gcode_macro CFS_FLUSH]
rename_existing: _REAL_CFS_FLUSH
variable_at: ""
variable_args: ""
gcode:
  {% set p = printer.toolhead.position %}
  SET_GCODE_VARIABLE MACRO=CFS_FLUSH VARIABLE=at VALUE='"{"%.1f,%.1f" % (p.x, p.y)}"'
  SET_GCODE_VARIABLE MACRO=CFS_FLUSH VARIABLE=args VALUE='"{rawparams}"'
  _HI_CFS_SHAKE

[gcode_macro _REPORT]
gcode:
  {% set f = printer['gcode_macro CFS_FLUSH'] %}
  {% set p = printer.toolhead.position %}
  {action_raise_error("flush_at=%s args=%s end=%.1f,%.1f,%.1f"
    % (f.at, f.args, p.x, p.y, p.z))}
"""


def test_purge_runs_the_flush_over_the_waste_bin(tmp_path):
    # CFS_FLUSH itself extrudes in place: CFS_PURGE must be at the bin
    # before it starts, and the short pass must come back to the bin.
    config = tmp_path / "printer.cfg"
    config.write_text(
        f"[include {PHASE5}]\n"
        "[serial_485 serial485]\n"
        f"serial: {tmp_path / 'no-such-port'}\n" + PURGE_STUBS
    )
    gcode_file = tmp_path / "test.gcode"
    gcode_file.write_text("G28\nCFS_PURGE LEN=30\n_REPORT\n")
    proc, _ = run_batch(gcode_file, tmp_path, config)
    assert "flush_at=-18.5,130.0 args=LEN=30 end=-18.5,130.0,10.0" in (
        proc.stderr
    )


def test_flush_calls_the_hi_short_pass():
    cfg = (ROOT / "config" / "hi" / "cfs.cfg").read_text()
    assert "\nnozzle_clean_macro: _HI_CFS_SHAKE\n" in cfg
