"""
Created 2019-10-01

Control of a Trinamic TMCM-6110 motoraxis. It differs from TMCM-610 in that
it is not possible to set the limit switch polarity individually for each axis.
It is done globally in the mother Trinamic_TMCM6110_DS device server.

Extra functionality is load level attribute.

@author: Filip Lindau
"""

import PyTango as pt
import numpy as np
import copy
import time
import threading
from PyTango.server import Device, DeviceMeta
from PyTango.server import attribute, command
from PyTango.server import device_property
from TMCM6110_motoraxis_controller import TMCM6110MotoraxisController

import logging
logger = logging.getLogger("TMCM6110_motoraxisDS")
logger.setLevel(logging.DEBUG)
while len(logger.handlers):
    logger.removeHandler(logger.handlers[0])

# f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
f = logging.Formatter("%(asctime)s - %(name)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
logger.addHandler(fh)


class TMCM6110MotoraxisDS(Device):
    __metaclass__ = DeviceMeta

    # --- Operator attributes
    #
    Position = attribute(label="position",
                         dtype=float,
                         access=pt.AttrWriteType.READ_WRITE,
                         unit="mm",
                         format="%6.2f",
                         min_value=-100000.0,
                         max_value=100000.0,
                         fget="get_position",
                         fset="set_position",
                         doc="Motor position in mm",
                         memorized=True,
                         hw_memorized=True)

    Velocity = attribute(label="velocity",
                         dtype=float,
                         access=pt.AttrWriteType.READ_WRITE,
                         unit="mm / s",
                         format="%6.2f",
                         min_value=-100000.0,
                         max_value=100000.0,
                         fget="get_velocity",
                         fset="set_velocity",
                         doc="Motor velocity in mm/s",
                         memorized=True,
                         hw_memorized=True)

    DialPosition = attribute(label="dial position",
                             dtype=float,
                             access=pt.AttrWriteType.READ,
                             unit="mm",
                             format="%6.2f",
                             min_value=-100000.0,
                             max_value=100000.0,
                             fget="get_dialposition",
                             fset="set_dialposition",
                             doc="Absolute motor position in mm with offset applied", )

    Offset = attribute(label="offset",
                       dtype=float,
                       access=pt.AttrWriteType.READ_WRITE,
                       unit="mm",
                       format="%6.2f",
                       min_value=-100000.0,
                       max_value=100000.0,
                       fget="get_offset",
                       fset="set_offset",
                       doc="Motor offset in mm",
                       memorized=True,
                       hw_memorized=True)

    Acceleration = attribute(label="acceleration",
                             dtype=float,
                             access=pt.AttrWriteType.READ_WRITE,
                             unit="mm / s**2",
                             format="%6.2f",
                             min_value=-1000000.0,
                             max_value=1000000.0,
                             fget="get_acceleration",
                             fset="set_acceleration",
                             doc="Motor acceleration in mm/s**2",
                             memorized=True,
                             hw_memorized=True)

    Deceleration = attribute(label="deceleration",
                             dtype=float,
                             access=pt.AttrWriteType.READ_WRITE,
                             unit="mm / s**2",
                             format="%6.2f",
                             min_value=-1000000.0,
                             max_value=1000000.0,
                             fget="get_deceleration",
                             fset="set_deceleration",
                             doc="Motor deceleration in mm/s**2",
                             memorized=True,
                             hw_memorized=True)

    Base_rate = attribute(label="base rate",
                          dtype=float,
                          access=pt.AttrWriteType.READ_WRITE,
                          unit="a.u.",
                          format="%6.2f",
                          min_value=-1000000.0,
                          max_value=1000000.0,
                          fget="get_baserate",
                          fset="set_baserate",
                          doc="Motor base rate... whatever that is",
                          memorized=True,)

    SimulationMode = attribute(label="simulation mode",
                               dtype=bool,
                               access=pt.AttrWriteType.READ,
                               unit="",
                               fget="get_simulationmode",
                               doc="Motor simulation mode",)

    Step_per_unit = attribute(label="step per unit",
                              dtype=float,
                              access=pt.AttrWriteType.READ_WRITE,
                              unit="steps/mm",
                              format="%6.2f",
                              min_value=-1000000.0,
                              max_value=1000000.0,
                              fget="get_stepperunit",
                              fset="set_stepperunit",
                              doc="Motor steps per unit",
                              memorized=True,
                              hw_memorized=True)

    Backlash = attribute(label="backlash",
                         dtype=bool,
                         access=pt.AttrWriteType.READ_WRITE,
                         unit="",
                         format="%d",
                         fget="get_backlash",
                         fset="set_backlash",
                         doc="Motor backlash",
                         memorized=True,
                         hw_memorized=True)

    Limit_switches = attribute(label="limit switches",
                               dtype=(bool, ),
                               max_dim_x=3,
                               access=pt.AttrWriteType.READ,
                               unit="",
                               format="%d",
                               fget="get_limitswitches",
                               doc="Limit switches state",)

    Load_level = attribute(label="load level",
                             dtype=float,
                             access=pt.AttrWriteType.READ,
                             unit="",
                             format="%6.2f",
                             min_value=-1000000.0,
                             max_value=1000000.0,
                             fget="get_loadlevel",
                             doc="Motor load level in normalized units",)


    # --- Device properties
    #
    TMCM6110_device = device_property(dtype=str,
                                      doc="Tango name of mother Trinamic_TMCM6110_DS",
                                      )

    Axis = device_property(dtype=int,
                           doc="Motor axis",)

    Motor_current = device_property(dtype=float,
                                    doc="Motor maxiumum current",)

    Microsteps = device_property(dtype=int,
                                 doc="Microstep resolution: 1, 2, 4, 8, or 16",)

    Limit0Enable = device_property(dtype=bool,
                                   doc="Enable limit switch 0?",)

    Limit1Enable = device_property(dtype=bool,
                                   doc="Enable limit switch 1?",)

    def __init__(self, klass, name):
        self.controller = None  # type: TMCM6110MotoraxisController
        self.setup_attr_params = dict()
        self.state_dispatcher = None  # type: StateDispatcher
        self.state_thread = None      # type: threading.Thread
        self.stop_state_thread = False

        self.offset_value = 0
        self.baserate_value = 0
        self.backlash_value = False
        self.simulation_value = False

        Device.__init__(self, klass, name)

    def init_device(self):
        self.debug_stream("In init_device:")
        Device.init_device(self)
        if self.controller is not None:
            self.stop_state_thread = True
            self.state_thread.join(1.0)
            self.controller.close()
        self.debug_stream("Device: {0}, axis {1}, motor_current {2}, microsteps {3}".format(self.TMCM6110_device,
                                                                                            self.Axis,
                                                                                            self.Motor_current,
                                                                                            self.Microsteps))
        self.controller = TMCM6110MotoraxisController(self.TMCM6110_device, self.Axis, self.Motor_current,
                                                      1, self.Microsteps, self.Limit0Enable, self.Limit1Enable)
        self.state_thread = threading.Thread(target=self._read_state)
        self.state_thread.daemon = True
        self.state_thread.start()

    def get_position(self):
        # self.debug_stream("In get_position:")
        if self.controller is None:
            self.error_stream("No controller")
            return None
        data = self.controller.get_position()
        try:
            value = data.value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def set_position(self, pos):
        self.debug_stream("In set_position: {0}".format(pos))
        if self.controller is None:
            self.error_stream("set_position: NO CONTROLLER")
            return
        self.controller.set_position(pos)
        return 0

    def get_velocity(self):
        # self.debug_stream("In get_velocity:")
        data = self.controller.get_speed()
        try:
            value = data.value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def set_velocity(self, value):
        self.debug_stream("In set_velocity: {0}".format(value))
        if self.controller is None:
            self.error_stream("set_velocity: NO CONTROLLER")
            return
        self.controller.set_speed(value)
        return 0

    def get_dialposition(self):
        # self.debug_stream("In get_dialposition:")
        data = self.controller.get_position()
        try:
            value = data.value - self.offset_value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def get_offset(self):
        # self.debug_stream("In get_offset:")
        value = self.offset_value
        quality = pt.AttrQuality.ATTR_VALID
        t = time.time()
        return value, t, quality

    def set_offset(self, value):
        self.debug_stream("In set_offset: {0}".format(value))
        self.offset_value = value
        return 0

    def get_acceleration(self):
        # self.debug_stream("In get_acceleration:")
        data = self.controller.get_acceleration()
        try:
            value = data.value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def set_acceleration(self, value):
        self.debug_stream("In set_acceleration: {0}".format(value))
        if self.controller is None:
            self.error_stream("set_acceleration: NO CONTROLLER")
            return
        self.controller.set_acceleration(value)

        return 0

    def get_deceleration(self):
        # self.debug_stream("In get_deceleration:")
        data = self.controller.get_acceleration()
        try:
            value = data.value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def set_deceleration(self, value):
        self.debug_stream("In set_deceleration: {0}".format(value))
        if self.controller is None:
            self.error_stream("set_deceleration: NO CONTROLLER")
            return
        self.controller.set_acceleration(value)

        return 0

    def get_baserate(self):
        # self.debug_stream("In get_baserate:")
        value = self.baserate_value
        quality = pt.AttrQuality.ATTR_VALID
        t = time.time()
        return value, t, quality

    def set_baserate(self, value):
        self.debug_stream("In set_baserate: {0}".format(value))
        self.baserate_value = value
        return 0

    def get_simulationmode(self):
        # self.debug_stream("In get_simulationmode:")
        value = self.simulation_value
        quality = pt.AttrQuality.ATTR_VALID
        t = time.time()
        return value, t, quality

    def set_simulationmode(self, value):
        self.debug_stream("In set_simulationmode: {0}".format(value))
        self.simulation_value = value
        return 0

    def get_stepperunit(self):
        # self.debug_stream("In get_stepperunit:")
        if self.controller is None:
            return None

        value = self.controller.get_stepperunit()
        quality = pt.AttrQuality.ATTR_VALID
        t = time.time()
        return value, t, quality

    def set_stepperunit(self, value):
        self.debug_stream("In set_stepperunit: {0}".format(value))
        if self.controller is None:
            return
        self.controller.set_stepperunit(value)
        return 0

    def get_backlash(self):
        # self.debug_stream("In get_backlash:")
        value = self.backlash_value
        quality = pt.AttrQuality.ATTR_VALID
        t = time.time()
        return value, t, quality

    def set_backlash(self, value):
        self.debug_stream("In set_backlash: {0}".format(value))
        self.backlash_value = value
        return 0

    def get_limitswitches(self):
        # self.debug_stream("In get_limitswitches:")
        data = self.controller.get_limitswitches()
        try:
            value = [data[0].value, data[1].value]
            t = data[0].time.totime()
            quality = pt.AttrQuality.ATTR_VALID

        except AttributeError:
            value = [False, False]
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def get_loadlevel(self):
        # self.debug_stream("In get_loadlevel:")
        data = self.controller.get_loadlevel()
        try:
            value = data.value
            t = data.time.totime()
            quality = pt.AttrQuality.ATTR_VALID
        except AttributeError:
            value = 0
            quality = pt.AttrQuality.ATTR_INVALID
            t = time.time()
        return value, t, quality

    def _read_state(self):
        self.debug_stream("Starting read state thread")
        old_state = "--"
        while not self.stop_state_thread:
            state, status = self.controller.get_state()
            if state in ["unknown"]:
                if old_state != state:
                    self.info_stream("Changing state to UNKNOWN")
                    self.set_state(pt.DevState.UNKNOWN)
                    self.set_status(status)
            elif state in ["init"]:
                if old_state != state:
                    self.info_stream("Changing state to INIT")
                    self.set_state(pt.DevState.INIT)
                    self.set_status(status)
            elif state in ["on"]:
                if old_state != state:
                    self.info_stream("Changing state to ON")
                    self.set_state(pt.DevState.ON)
                    self.set_status(status)
            elif state in ["moving"]:
                if old_state != state:
                    self.info_stream("Changing state to MOVING")
                    self.set_state(pt.DevState.MOVING)
                    self.set_status(status)
            else:
                self.info_stream("Unknown state,setting UNKNOWN")
                self.set_state(pt.DevState.UNKNOWN)
            old_state = state
            time.sleep(0.1)


if __name__ == "__main__":
    pt.server.server_run((TMCM6110MotoraxisDS,))
