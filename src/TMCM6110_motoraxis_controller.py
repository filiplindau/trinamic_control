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
from tasks.GenericTasks import *

import logging
logger = logging.getLogger("TMCM6110_motoraxis_controller")
logger.setLevel(logging.DEBUG)
while len(logger.handlers):
    logger.removeHandler(logger.handlers[0])

# f = logging.Formatter("%(asctime)s - %(module)s.   %(funcName)s - %(levelname)s - %(message)s")
f = logging.Formatter("%(asctime)s - %(name)s.   %(funcName)s - %(levelname)s - %(message)s")
fh = logging.StreamHandler()
fh.setFormatter(f)
logger.addHandler(fh)


class TangoDeviceConnectTask(Task):
    def __init__(self, device_name, name=None, timeout=None, trigger_dict=dict(), callback_list=list()):
        Task.__init__(self, name, timeout=timeout, trigger_dict=trigger_dict, callback_list=callback_list)
        self.device_name = device_name
        self.logger.setLevel(logging.INFO)

    def action(self):
        self.logger.info("{0} entering action. ".format(self))
        # Exceptions are caught in the parent run thread.
        self.logger.debug("Connecting to {0}".format(self.device_name))
        try:
            dev = pt.DeviceProxy(self.device_name)
        except pt.DevFailed as e:
            self.result = e
            self.cancel()
            return
        self.result = dev


class TangoReadAttributeTask(Task):
    def __init__(self, attribute_name, device_name, device_handler, name=None, timeout=None,
                 trigger_dict=dict(), callback_list=list(), ignore_tango_error=True):
        Task.__init__(self, name, timeout=timeout, trigger_dict=trigger_dict, callback_list=callback_list)
        self.device_name = device_name
        self.attribute_name = attribute_name
        self.device_handler = device_handler
        self.ignore_tango_error = ignore_tango_error

        self.logger.setLevel(logging.WARNING)

    def action(self):
        self.logger.info("{0} reading {1} on {2}. ".format(self, self.attribute_name, self.device_name))
        try:
            dev = self.device_handler.get_device(self.device_name)
        except pt.DevFailed as e:
            self.logger.error("{0}: Could not connect. {1}".format(self, e))
            self.result = e
            self.cancel()
            return
        retries = 0
        while retries < 3:
            try:
                attr = dev.read_attribute(self.attribute_name)
                break
            except AttributeError as e:
                self.logger.exception("{0}: Attribute error reading {1} on {2}: ".format(self,
                                                                                         self.attribute_name,
                                                                                         self.device_name))
                attr = None
                self.result = e
                self.cancel()
                return
            except pt.DevFailed as e:
                self.logger.exception("{0}: Tango error reading {1} on {2}: ".format(self,
                                                                                     self.attribute_name,
                                                                                     self.device_name))
                attr = None
                self.result = e
                if not self.ignore_tango_error:
                    self.cancel()
                    return

            retries += 1
        self.result = attr


class TangoWriteAttributeTask(Task):
    def __init__(self, attribute_name, device_name, device_handler, value, name=None, timeout=None,
                 trigger_dict=dict(), callback_list=list()):
        Task.__init__(self, name, timeout=timeout, trigger_dict=trigger_dict, callback_list=callback_list)
        self.device_name = device_name
        self.attribute_name = attribute_name
        self.device_handler = device_handler
        self.value = value
        self.logger.setLevel(logging.INFO)

    def action(self):
        self.logger.info("{0} writing {1} to {2} on {3}. ".format(self,
                                                                  self.value,
                                                                  self.attribute_name,
                                                                  self.device_name))
        try:
            dev = self.device_handler.get_device(self.device_name)
        except pt.DevFailed as e:
            self.logger.error("{0}: Could not connect. {1}".format(self, e))
            self.result = e
            self.cancel()
            return
        try:
            res = dev.write_attribute(self.attribute_name, self.value)
        except pt.DevFailed as e:
            self.logger.error("{0}: Could not write attribute {1} with {2}. {3}".format(self, self.attribute_name,
                                                                                        self.value, e))
            self.result = e
            self.cancel()
            return
        self.result = res


class TangoCommandTask(Task):
    def __init__(self, command_name, device_name, device_handler, value=None, name=None, timeout=None,
                 trigger_dict=dict(), callback_list=list()):
        Task.__init__(self, name, timeout=timeout, trigger_dict=trigger_dict, callback_list=callback_list)
        self.device_name = device_name
        self.command_name = command_name
        self.device_handler = device_handler
        self.value = value
        self.logger.setLevel(logging.INFO)

    def action(self):
        self.logger.info("{0} sending command {1} with {2} on {3}. ".format(self,
                                                                            self.command_name,
                                                                            self.value,
                                                                            self.device_name))
        try:
            dev = self.device_handler.get_device(self.device_name)
        except pt.DevFailed as e:
            self.logger.error("{0}: Could not connect. {1}".format(self, e))
            self.result = e
            self.cancel()
            return
        try:
            res = dev.command_inout(self.command_name, self.value)
        except pt.DevFailed as e:
            self.logger.error("{0}: Could not write command {1} with {2}. {3}".format(self, self.command_name,
                                                                                      self.value, e))
            self.result = e
            self.cancel()
            return
        self.result = res


class TangoMonitorAttributeTask(Task):
    def __init__(self, attribute_name, device_name, device_handler, target_value, interval=0.5, tolerance=0.01,
                 tolerance_type="abs", name=None, timeout=None, trigger_dict=dict(), callback_list=list()):
        Task.__init__(self, name, timeout=timeout, trigger_dict=trigger_dict, callback_list=callback_list)
        self.device_name = device_name
        self.attribute_name = attribute_name
        self.device_handler = device_handler
        self.target_value = target_value
        self.interval = interval
        self.tolerance = tolerance
        if tolerance_type == "rel":
            self.tol_div = self.target_value
            if self.tol_div == 0.0:
                raise AttributeError("Target value = 0 with relative tolerance type not possible.")
        else:
            self.tol_div = 1.0
        self.logger.setLevel(logging.INFO)

    def action(self):
        self.logger.info("{0} monitor reading {1} from {2}. ".format(self, self.attribute_name, self.device_name))
        current_value = float("inf")
        read_task = TangoReadAttributeTask(self.attribute_name, self.device_name,
                                           self.device_handler, timeout=self.timeout,
                                           name="read_monitor_{0}".format(self.attribute_name))
        wait_time = -1
        while abs((current_value - self.target_value) / self.tol_div) > self.tolerance:
            if wait_time > 0:
                time.sleep(wait_time)
            t0 = time.time()
            read_task.start()
            current_value = read_task.get_result(wait=True, timeout=self.timeout).value
            if read_task.is_cancelled() is True:
                self.result = current_value
                self.cancel()
                return
            t1 = time.time()
            wait_time = self.interval - (t1 - t0)

        self.result = read_task.get_result(wait=False)


class DeviceHandler(object):
    """
    Handler for open devices.
    Devices are stored in a dict for easy retrieval.
    New devices are added asynchronously with add_device method.
    """
    def __init__(self, tango_host=None, name=None):
        self.devices = dict()
        self.tango_host = tango_host
        self.timeout = 10.0
        if name is None:
            self.name = self.__repr__()
        else:
            self.name = name
        self.logger = logging.getLogger("Task.{0}".format(self.name.upper()))
        self.logger.setLevel(logging.INFO)

    def get_device(self, device_name):
        self.logger.debug("{0} Returning device {1}".format(self, device_name))
        try:
            dev = self.devices[device_name]
        except KeyError:
            # Maybe this should just raise an exception instead of auto-adding:
            task = self.add_device(device_name)
            dev = task.get_result(wait=True, timeout=self.timeout)
            if task.is_cancelled():
                raise pt.DevFailed(dev)

        return dev

    def add_device(self, device_name):
        """
        Add a device to the open devices dictionary.
        A device connect task is created and started.

        :param device_name: Tango name of device
        :return: opened device proxy
        """
        self.logger.info("{0} Adding device {1} to device handler".format(self, device_name))
        if device_name in self.devices:
            self.logger.debug("Device already in dict. No need")
            return True
        if self.tango_host is not None:
            full_dev_name = "{0}/{1}".format(self.tango_host, device_name)
        else:
            full_dev_name = device_name
        task = TangoDeviceConnectTask(full_dev_name, name="CONNECT_{0}".format(device_name))
        task.start()

        task_call = CallableTask(self._dev_connect_done, (device_name, task),
                                 name="ADD_{0}".format(device_name))
        task_call.add_trigger(task)
        task_call.start()

        return task_call

    def add_devices(self, device_names):
        agg_task = DelayTask(0.0)
        for dn in device_names:
            t = self.add_device(dn)
            agg_task.add_trigger(t)
        agg_task.start()
        return agg_task

    def _dev_connect_done(self, device_name, task):
        dev = task.get_result(wait=True, timeout=self.timeout)
        self.logger.info("{0} {1} Device connection completed. Returned {1}".format(self, device_name, dev))
        self.devices[device_name] = dev
        return dev

    def __str__(self):
        s = "{0} {1}".format(type(self).__name__, self.name)
        return s


class TMCM6110MotoraxisController(object):
    def __init__(self, dev_name, axis, max_current, step_per_unit, microstep=None,
                 limit0enable=None, limit1enable=None, speed=None, acceleration=None, position=None):
        self.dev_name = dev_name
        self.device = None
        self.device_handler = DeviceHandler(name="TMCM6110_dev_handler")
        self.axis = axis
        self.max_current = max_current
        self.microstep = microstep
        self.limit0enable = limit0enable
        self.limit1enable = limit1enable
        self.step_per_unit = step_per_unit
        self.step_pos = None
        self.position = None
        self.target_position = position
        self.speed = None
        self.target_speed = speed
        self.acceleration = None
        self.target_acceleration = acceleration
        self.limitswitches = [None, None]
        self.load_level = None
        self.motor_state = None

        self.current_state = "unknown"
        self.status = ""
        self.task_list = list()
        self.task_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.connect(self.dev_name)

    def connect(self, dev_name, delay=0):
        logger.info("Connecting to {0}".format(dev_name))
        # Cancel all running tasks before connecting:
        with self.task_lock:
            for t in self.task_list:
                t.cancel()
        with self.data_lock:
            self.current_state = "unknown"
            self.status = "Connecting to motor device {0}".format(self.dev_name)
        delay_task = DelayTask(delay, "Reconnect delay")
        connect_task = TangoDeviceConnectTask(dev_name, "Motor connect", callback_list=[self._connect_done])
        seq_task = SequenceTask([delay_task, connect_task], "Connect sequence")
        with self.task_lock:
            self.task_list.append(connect_task)
            self.dev_name = dev_name
        seq_task.start()

    def close(self):
        with self.task_lock:
            for t in self.task_list:
                t.cancel()
        with self.data_lock:
            self.current_state = "closed"

    def _connect_done(self, result):
        logger.debug("Connect to motor done. Result: {0}".format(result.get_result(False)))
        with self.task_lock:
            tr = None
            for t in self.task_list:
                if id(t) == id(result):
                    tr = t
            self.task_list.remove(tr)
        if result.is_cancelled():
            logger.error("Could not connect, reconnecting in 3 s")
            self.connect(self.dev_name, 3.0)
            return
        dev = result.get_result(wait=False)
        with self.data_lock:
            self.device = dev
            self.device_handler.add_device(dev)
            self.current_state = "connected"
        self.init_motor_settings()

    def init_motor_settings(self, delay=0):
        logger.info("Initializing motor with settings")
        with self.data_lock:
            self.current_state = "init"
            self.status = "Initializing motor parameters"

            task_list = list()

            delay_task = DelayTask(delay, "init delay task")
            task_list.append(delay_task)

            attr_name = "maxcurrentm{0}".format(self.axis)
            t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, self.max_current)
            task_list.append(t)

            if self.microstep is not None:
                attr_name = "microstepresolutionm{0}".format(self.axis)
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, self.microstep)
                task_list.append(t)

            if self.limit0enable is not None:
                attr_name = "limit0enablem{0}".format(self.axis)
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, self.limit0enable)
                task_list.append(t)

            if self.limit1enable is not None:
                attr_name = "limit1enablem{0}".format(self.axis)
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, self.limit0enable)
                task_list.append(t)

            if self.target_speed is not None:
                attr_name = "speedm{0}".format(self.axis)
                value = self.step_per_unit * self.target_speed
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, value)
                task_list.append(t)

            if self.target_acceleration is not None:
                attr_name = "accelerationm{0}".format(self.axis)
                value = self.step_per_unit * self.target_acceleration
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, value)
                task_list.append(t)

            if self.target_position is not None:
                attr_name = "positionm{0}".format(self.axis)
                value = self.step_per_unit * self.target_position
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, value)
                task_list.append(t)

            seq_task = SequenceTask(task_list, "init motor task", callback_list=[self._init_motor_settings_done])
            seq_task.start()
        with self.task_lock:
            self.task_list.append(seq_task)

    def _init_motor_settings_done(self, result):
        logger.debug("Init motor done. Result: {0}".format(result.get_result(False)))
        with self.task_lock:
            self.task_list.remove(result)
        if result.is_cancelled():
            logger.error("Could not init, testing again in 3 s")
            self.init_motor_settings(3.0)
            return
        with self.data_lock:
            self.current_state = "on"
            self.status = "On"
        self.monitor_attributes(0.2)

    def monitor_attributes(self, delay):
        logger.info("Starting monitor of motor attributes")

        attr_name = "positionm{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "pos_read",
                                   callback_list=[self._read_position])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "speedm{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "speed_read",
                                   callback_list=[self._read_speed])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "accelerationm{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "acc_read",
                                   callback_list=[self._read_acceleration])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "loadm{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "load_read",
                                   callback_list=[self._read_loadlevel])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "limit0m{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "limit0_read",
                                   callback_list=[self._read_limit0])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "limit1m{0}".format(self.axis)
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "limit1_read",
                                   callback_list=[self._read_limit1])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

        attr_name = "state"
        t = TangoReadAttributeTask(attr_name, self.dev_name, self.device_handler, "state_read",
                                   callback_list=[self._read_state])
        tr = RepeatTask(t, -1, delay)
        with self.task_lock:
            self.task_list.append(tr)
        tr.start()

    def _read_position(self, result):
        # logger.debug("Position: {0}".format(result.get_result(wait=False)))
        with self.data_lock:
            self.position = result.get_result(wait=False)
            self.position.value = self.position.value / self.step_per_unit

    def get_position(self):
        with self.data_lock:
            value = self.position
        return value

    def set_position(self, value):
        logger.info("New position: {0}".format(value))
        with self.data_lock:
            if self.current_state in ["unknown"]:
                self.target_position = value
            else:
                attr_name = "positionm{0}".format(self.axis)
                step_value = value * self.step_per_unit
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, step_value, "write_pos")
                t.start()
        return True

    def _read_state(self, result):
        # logger.debug("State: {0}".format(result.get_result(wait=False)))
        motor_state = result.get_result(wait=False)
        with self.data_lock:
            self.motor_state = motor_state

    def _read_speed(self, result):
        with self.data_lock:
            self.speed = result.get_result(wait=False)
            self.speed.value = self.speed.value / self.step_per_unit
            if abs(self.speed.value) > 0:
                self.current_state = "moving"
            else:
                self.current_state = "on"

    def get_speed(self):
        with self.data_lock:
            value = self.speed
        return value

    def set_speed(self, value):
        logger.info("New speed: {0}".format(value))
        with self.data_lock:
            if self.current_state in ["unknown"]:
                self.target_speed = value
            else:
                attr_name = "speedm{0}".format(self.axis)
                step_value = value * self.step_per_unit
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, step_value, "write_speed")
                t.start()
        return True

    def _read_acceleration(self, result):
        with self.data_lock:
            self.acceleration = result.get_result(wait=False)
            self.acceleration.value = self.acceleration.value / self.step_per_unit

    def get_acceleration(self):
        with self.data_lock:
            value = self.acceleration
        return value

    def set_acceleration(self, value):
        logger.info("New acceleration: {0}".format(value))
        with self.data_lock:
            if self.current_state in ["unknown"]:
                self.target_acceleration = value
            else:
                attr_name = "accelerationm{0}".format(self.axis)
                step_value = value * self.step_per_unit
                t = TangoWriteAttributeTask(attr_name, self.dev_name, self.device_handler, step_value, "write_acceleration")
                t.start()
        return True

    def _read_loadlevel(self, result):
        with self.data_lock:
            self.load_level = result.get_result(wait=False)

    def get_loadlevel(self):
        with self.data_lock:
            value = self.load_level
        return value

    def _read_limit0(self, result):
        with self.data_lock:
            self.limitswitches[0] = result.get_result(wait=False)

    def _read_limit1(self, result):
        with self.data_lock:
            self.limitswitches[1] = result.get_result(wait=False)

    def get_limitswitches(self):
        with self.data_lock:
            value = self.limitswitches
        return value

    def get_stepperunit(self):
        with self.data_lock:
            value = self.step_per_unit
        return value

    def set_stepperunit(self, value):
        logger.info("New step_per_unit: {0}".format(value))
        with self.data_lock:
            self.step_per_unit = value

    def get_state(self):
        with self.data_lock:
            value = copy.copy(self.current_state)
            status = copy.copy(self.status)
        return value, status


if __name__ == "__main__":
    tmc = TMCM6110MotoraxisController("testgun/motors/trinamic210", 0, 0.14, 1)
