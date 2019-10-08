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
        self.logger.setLevel(logging.DEBUG)

    def action(self):
        self.logger.info("{0} entering action. ".format(self))
        # Exceptions are caught in the parent run thread.
        self.logger.debug("Connecting to {0}".format(self.device_name))
        try:
            dev = pt.DeviceProxy(self.device_name)
        except pt.DevFailed as e:
            self.logger.error("{0} cancelling due to DevFailed".format(self))
            self.result = e
            self.cancel()
            return
        self.result = dev

    def emit(self):
        self.logger.info("{0} emitting to {1}".format(self, self.callback_list))
        Task.emit(self)


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
                # attr = None
                attr = e
                self.result = e
                if not self.ignore_tango_error:
                    self.cancel()
                    return

            retries += 1
            time.sleep(0.3)
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
            try:
                dev = task.get_result(wait=True, timeout=self.timeout)
                if task.is_cancelled():
                    raise pt.DevFailed(dev)
            except AttributeError:
                pass

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


dev_name = "testgun/motors/trinammic210"
device = None
device_handler = DeviceHandler(name="TMCM6110_dev_handler")
current_state = "unknown"
status = ""
task_list = list()

connect_done_count = 0
connect_count = 0


def connect(dev_name, delay=1):
    global current_state
    global status
    global connect_count
    global connect_done_count

    connect_count += 1

    logger.info("Connecting to {0} count {1}, {2}".format(dev_name, connect_count, connect_done_count))

    if connect_count > 2:
        logger.error("Too many retries. Stopping")
        return
    current_state = "unknown"
    status = "Connecting to motor device {0}".format(dev_name)
    delay_task = DelayTask(delay, "Reconnect delay {0}, {1}".format(connect_count, connect_done_count))
    connect_task = TangoDeviceConnectTask(dev_name, "Motor connect {0}".format(connect_count), callback_list=[connect_done])
    seq_task = SequenceTask([delay_task, connect_task], "Connect sequence {0}".format(connect_count))
    seq_task.start()
    return seq_task


def close():
    global task_list
    global current_state

    for t in task_list:
        t.cancel()
    current_state = "closed"


def connect_done(result):
    logger.debug("Connect to motor done from {0}. Result: {1}".format(result.get_name(), result.get_result(False)))
    global current_state
    global status
    global device
    global device_handler
    global connect_count
    global connect_done_count

    connect_done_count += 1

    if result.is_cancelled():
        logger.error("Could not connect, reconnecting in 3 s. Retries: {0}, {1}".format(connect_count, connect_done_count))
        connect(dev_name, 3.0)
        return
    dev = result.get_result(wait=False)
    device = dev
    device_handler.add_device(dev)
    current_state = "connected"


def init_motor_settings(delay=0):
    logger.info("Initializing motor with settings")
    global current_state
    global status
    global device
    global device_handler
    global task_list

    if current_state == "unknown":
        status = "Initializing motor parameters"
    current_state = "init"

    task_list = list()

    delay_task = DelayTask(delay, "init delay task")
    task_list.append(delay_task)

    attr_name = "maxcurrentm0"
    t = TangoWriteAttributeTask(attr_name, dev_name, device_handler, 0.14)
    task_list.append(t)

    seq_task = SequenceTask(task_list, "init_motor_task", callback_list=[init_motor_settings_done])
    seq_task.start()

    task_list.append(seq_task)
    return seq_task


def init_motor_settings_done(result):
    logger.debug("Init motor done. Result: {0}".format(result.get_result(False)))
    global current_state
    global status
    global device
    global device_handler
    global task_list

    if result.is_cancelled():
        # with self.task_lock:
        #     pop_list = list()
        #     for t in self.task_list:
        #         if t.get_name() == "init_motor_task":
        #             logger.debug("Found init task {0}".format(t))
        #             pop_list.append(t)
        #     for t in pop_list:
        #         t.cancel()
        #         self.task_list.remove(t)
        logger.error("Could not init, testing again in 3 s")

        if result.get_result(False) is None:
            logger.error("Init returned NONE, re-init")
            init_motor_settings(3.5)
            return

        e = result.get_result(False)[1]
        try:
            logger.debug("Result: {0}".format(e[0].reason))
            # logger.debug("Result: {0}".format(e[0].reason == "API_CantConnectToDevice"))
            if e[0].reason in ["API_DeviceNotExported", "API_CantConnectToDevice"]:
                logger.debug("Parent device not started")
                status = "Parent device not started"
        except AttributeError:
            logger.error("Attr error {0}".format(e))
            status = "Attr error {0}".format(e)
        init_motor_settings(3.0)
        return

    current_state = "on"
    status = "On"
    task_list = list()
    logger.info("Starting MONITOR ATTRIBUTES")
