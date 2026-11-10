# Motor cable check for the TMC2208 steppers of the Creality Hi (Z, Z1, E)
#
# Port of the stock Hi module of the same name: each motor moves 0.5 mm
# with FORCE_MOVE and the low byte of DRV_STATUS is read back. Any flag
# there (overtemperature, short to ground or supply, open load) is
# reported as a motor cable that is not plugged in.
#
# Differences from stock:
# - registers are read through the standard get_register() by name (stock
#   added test_get_register() by address to tmc_uart.py);
# - CHECK_ZMOTOR_LINE moves each Z motor back exactly once (stock moved
#   stepper_z back twice when Z1 failed);
# - plain error messages, without the Creality screen key codes.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

DRV_STATUS = "DRV_STATUS"
FAULT_MASK = 0xFF
MOVE_DIST = 0.5
MOVE_SPEED = 10.0
SETTLE_TIME = 0.5


class TMCLineCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_command(
            "READ_REG_TMC", self.cmd_READ_REG_TMC, desc=self.cmd_READ_REG_help
        )
        self.gcode.register_command(
            "CHECK_MOTOR_LINE",
            self.cmd_CHECK_MOTOR_LINE,
            desc=self.cmd_CHECK_MOTOR_LINE_help,
        )
        self.gcode.register_command(
            "CHECK_ZMOTOR_LINE",
            self.cmd_CHECK_ZMOTOR_LINE,
            desc=self.cmd_CHECK_ZMOTOR_LINE_help,
        )

    def _pause(self, delay):
        if self.printer.is_shutdown():
            return
        self.printer.lookup_object("toolhead").get_last_move_time()
        reactor = self.printer.get_reactor()
        reactor.pause(reactor.monotonic() + delay)

    def _fail(self, names):
        msg = "CHECK_MOTOR_LINE: %s motor line is not inserted" % (
            "-".join(names),
        )
        self.printer.invoke_shutdown("Shutdown due to " + msg)
        raise self.printer.command_error(msg)

    def _mcu_tmc(self, stepper):
        name = "tmc2208 stepper_%s" % (stepper,)
        driver = self.printer.lookup_object(name, None)
        if driver is None:
            raise self.printer.command_error("No [%s] section" % (name,))
        return driver.mcu_tmc

    def _read(self, stepper, reg_name=DRV_STATUS):
        val = self._mcu_tmc(stepper).get_register(reg_name)
        self.gcode.respond_info(
            "tmc2208 stepper_%s %s: 0x%08x" % (stepper, reg_name, val)
        )
        return val

    def _move(self, stepper, dist):
        self.gcode.run_script_from_command(
            "FORCE_MOVE STEPPER=stepper_%s DISTANCE=%.1f VELOCITY=%.0f"
            % (stepper, dist, MOVE_SPEED)
        )

    cmd_READ_REG_help = "Read a TMC2208 register (STEP=z, A=address)"

    def cmd_READ_REG_TMC(self, gcmd):
        addr = gcmd.get_int("A", 0x6F, minval=0, maxval=0x7F)
        stepper = gcmd.get("STEP", "z").lower()
        mcu_tmc = self._mcu_tmc(stepper)
        names = [n for n, a in mcu_tmc.name_to_reg.items() if a == addr]
        if not names:
            raise gcmd.error("No TMC2208 register at address 0x%02x" % (addr,))
        self._read(stepper, names[0])

    cmd_CHECK_MOTOR_LINE_help = "Check the cable of one motor (STEP=z)"

    def cmd_CHECK_MOTOR_LINE(self, gcmd):
        stepper = gcmd.get("STEP", "z").lower()
        self._mcu_tmc(stepper)
        self._move(stepper, MOVE_DIST)
        self._pause(SETTLE_TIME)
        if self._read(stepper) & FAULT_MASK:
            self._pause(SETTLE_TIME)
            self.gcode.run_script_from_command("M84")
            self._fail([stepper.upper()])
        self._move(stepper, -MOVE_DIST)
        self.gcode.run_script_from_command("M84")
        gcmd.respond_info("%s motor line is inserted ok" % (stepper.upper(),))

    cmd_CHECK_ZMOTOR_LINE_help = "Check the cables of both Z motors"

    def cmd_CHECK_ZMOTOR_LINE(self, gcmd):
        steppers = ("z", "z1")
        for stepper in steppers:
            self._mcu_tmc(stepper)
        for stepper in steppers:
            self._move(stepper, MOVE_DIST)
        failed = []
        for stepper in steppers:
            self._pause(SETTLE_TIME)
            if self._read(stepper) & FAULT_MASK:
                failed.append(stepper.upper())
        self._pause(SETTLE_TIME)
        for stepper in steppers:
            self._move(stepper, -MOVE_DIST)
        self.gcode.run_script_from_command("M84")
        if failed:
            self._fail(failed)
        gcmd.respond_info("Z and Z1 motor lines are inserted ok")


def load_config(config):
    return TMCLineCheck(config)
