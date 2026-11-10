# Mirror an MCU input pin onto an output pin of the same MCU (Creality Hi)
#
# The Creality Hi toolhead board forwards a sensor input to a line shared
# with the main board. Ported from the io_remap.py module of the stock
# Creality Hi firmware (V1.1.0.60).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

from klippy import mcu


class IORemap:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.src_pin = config.get("src_pin")
        self.remap_pin = config.get("remap_pin")
        # 1: pull-up on the input (reads 0 when triggered); 0: pull-down
        self.src_pullup = config.getint("src_pullup", minval=0, maxval=1)
        # Idle level of the output pin
        self.remap_def = config.getint("remap_def", minval=0, maxval=1)
        # The output follows the input once it has been stable for
        # filterNum * periodTicks (0 selects the firmware default)
        self.filter_num = config.getint("filterNum", 0, minval=0, maxval=255)
        self.period_ticks = config.getint("periodTicks", 0, minval=0)
        mcu_name = config.get("mcu", "nozzle_mcu")
        self.mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        self.oid = self.mcu.create_oid()
        self.operation_cmd = None
        self.mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_IOREMAP", self.cmd_SET_IOREMAP, desc=self.cmd_SET_IOREMAP_help
        )

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_ioRemap oid=%d src_pin=%s src_pullup=%d"
            " remap_pin=%s remap_def=%d"
            % (
                self.oid,
                self.src_pin,
                self.src_pullup,
                self.remap_pin,
                self.remap_def,
            )
        )
        self.operation_cmd = self.mcu.lookup_query_command(
            "operation_ioRemap oid=%c operation=%c filterNum=%c periodTicks=%u",
            "query_ioRemap oid=%c sta=%c",
            oid=self.oid,
        )

    cmd_SET_IOREMAP_help = "Enable (S=1) or disable (S=0) the pin remapping"

    def cmd_SET_IOREMAP(self, gcmd):
        operation = gcmd.get_int("S", 0, minval=0, maxval=255)
        self.operation_cmd.send(
            [self.oid, operation, self.filter_num, self.period_ticks]
        )


def load_config(config):
    return IORemap(config)
