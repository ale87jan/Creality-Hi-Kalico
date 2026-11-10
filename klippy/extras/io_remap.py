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
        # Stepper whose endstop arrives through the remapped pin: the remap
        # is switched on only while that stepper homes, as the stock homing
        # code does for X. The line is shared with the probe.
        self.homing_stepper = config.get("homing_stepper", None)
        self.enabled = False
        mcu_name = config.get("mcu", "nozzle_mcu")
        self.mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        self.oid = self.mcu.create_oid()
        self.operation_cmd = None
        self.mcu.register_config_callback(self._build_config)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_IOREMAP", self.cmd_SET_IOREMAP, desc=self.cmd_SET_IOREMAP_help
        )
        if self.homing_stepper is not None:
            self.printer.register_event_handler(
                "homing:homing_move_begin", self._handle_homing_move_begin
            )
            self.printer.register_event_handler(
                "homing:homing_move_end", self._handle_homing_move_end
            )
            self.printer.register_event_handler(
                "gcode:command_error", self._handle_command_error
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
        operation_format = (
            "operation_ioRemap oid=%c operation=%c filterNum=%c periodTicks=%u"
        )
        if self.mcu.is_fileoutput():
            # Batch mode has no MCU to answer: send without waiting
            self.operation_cmd = self.mcu.lookup_command(operation_format)
        else:
            self.operation_cmd = self.mcu.lookup_query_command(
                operation_format, "query_ioRemap oid=%c sta=%c", oid=self.oid
            )

    def set_remap(self, operation):
        self.operation_cmd.send(
            [self.oid, operation, self.filter_num, self.period_ticks]
        )
        self.enabled = bool(operation)

    def _uses_homing_stepper(self, hmove):
        return any(
            stepper.get_name() == self.homing_stepper
            for mcu_endstop in hmove.get_mcu_endstops()
            for stepper in mcu_endstop.get_steppers()
        )

    def _handle_homing_move_begin(self, hmove):
        if self._uses_homing_stepper(hmove):
            self.set_remap(1)

    def _handle_homing_move_end(self, hmove):
        if self.enabled:
            self.set_remap(0)

    def _handle_command_error(self):
        if self.enabled:
            self.set_remap(0)

    def get_status(self, eventtime=None):
        return {"enabled": self.enabled}

    cmd_SET_IOREMAP_help = "Enable (S=1) or disable (S=0) the pin remapping"

    def cmd_SET_IOREMAP(self, gcmd):
        self.set_remap(gcmd.get_int("S", 0, minval=0, maxval=255))


def load_config(config):
    return IORemap(config)
