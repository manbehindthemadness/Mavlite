#!/usr/bin/env python
"""
mavlink python utility functions

Copyright Andrew Tridgell 2011-2019
Released under GNU LGPL version 3 or later
"""
from builtins import object

import math
import struct
import time
import os
import sys
import select
import json
import gc

try:
    mp = True
    from machine import UART as uart  # noqa
except ImportError:
    mp = False
    from busio import UART as uart  # noqa


class UART:
    # noinspection GrazieInspection
    """
        Wrapper class for machine.UART and busio.UART
        """

    def __init__(
            self,
            s_id: int = 1,
            baudrate: int = 57600,
            bits: int = 8,
            parity: [None, int] = None,
            stop: int = 1,
            timeout: int = 1,
            rxbuf: int = 64,
            tx: any = None,
            rx: any = None
    ):
        if mp:
            self.uart = uart(s_id, baudrate)
            self.uart.init(baudrate, bits, parity, stop, timeout=timeout, rxbuf=rxbuf, rx=rx, tx=tx)
        else:
            self.uart = uart(tx, rx, baudrate=baudrate, bits=bits,
                             parity=parity, stop=stop, timeout=timeout, receiver_buffer_size=rxbuf)

    def read(self, n_bytes: int = 8) -> uart.read:
        """
        Read wrapper.
        """
        return self.uart.read(n_bytes)

    def readinto(self, buf: bytes) -> uart.readinto:
        """
        Readinto wrapper.
        """
        return self.readinto(buf)

    def readline(self) -> uart.readline:
        """
        Readline wrapper.
        """
        return self.uart.readline()

    def write(self, buf: [str, bytes]) -> uart.write:
        """
        Write wrapper
        """
        if not isinstance(buf, bytes):
            buf = buf.encode()
        return self.uart.write(buf)

    def deinit(self) -> uart.deinit:
        """
        Deinit wrapper.
        """
        return self.uart.deinit()

    def setbaudrate(self, baudrate: int):
        """
        Set baudrate wrapper.
        """
        if mp:
            return self.uart.setbaudrate(baudrate)
        else:
            self.uart.baudrate = baudrate
            return self.uart

    def any(self):
        """
        Any wrapper.
        """
        if mp:
            result = self.uart.any()
        else:
            result = self.uart.in_waiting
        if result:
            print('bytes waiting', result)
        return result


is_py3 = sys.version_info >= (3, 0)

# adding these extra imports allows pymavlink to be used directly with pyinstaller
# without having complex spec files. To allow for installs that don't have ardupilotmega
# at all we avoid throwing an exception if it isn't installed

# maximum packet length for a single receive call - use the UDP limit
UDP_MAX_PACKET_LEN = 65535

# Store the MAVLink library for the currently-selected dialect
# (set by set_dialect())
# mavlink = None
# current_dialect = None

# Store the mavlink file currently being operated on
# (set by mavlink_connection())
mavfile_global = None

# If the caller hasn't specified a particular native/legacy version, use this
default_native = False

# link_id used for signing
global_link_id = 0

# Use a globally-set MAVLink dialect if one has been specified as an environment variable.
Dialect = 'mavgen_python_dialect'

eval_errs = (
    NameError,
    ZeroDivisionError,
    IndexError
)


def _evaluate_expression(expression, m_vars, nocondition=False):
    """
    evaluation an expression
    """
    # first check for conditions which take the form EXPRESSION{CONDITION}
    if expression[-1] == '}':
        startidx = expression.rfind('{')
        if startidx == -1:
            return None
        condition = expression[startidx + 1:-1]
        expression = expression[:startidx]
        try:
            v = eval(condition, globals(), m_vars)
        except eval_errs:
            return None
        if not nocondition and not v:
            return None
    try:
        v = eval(expression, globals(), m_vars)
    except eval_errs:
        return None
    return v


def evaluate_expression(expression, m_vars, nocondition=False):
    """
    evaluation an expression
    """
    return _evaluate_expression(expression, m_vars, nocondition)


def evaluate_condition(condition, m_vars):
    """
    evaluation a conditional (boolean) statement
    """
    if condition is None:
        return True
    v = evaluate_expression(condition, m_vars)
    if v is None:
        return False
    return v


def u_ord(c):
    """
    Needs population.
    """
    if is_py3:
        return c
    return ord(c)


class Location(object):
    """
    represent a GPS coordinate
    """

    def __init__(self, lat, lng, alt=0, heading=0):
        self.lat = lat  # in degrees
        self.lng = lng  # in degrees
        self.alt = alt  # in metres
        self.heading = heading

    def __str__(self):
        return "lat=%.6f,lon=%.6f,alt=%.1f" % (self.lat, self.lng, self.alt)


def add_message(messages, mtype, msg):
    """
    add a msg to array of messages, taking account of instance messages
    """
    if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:  # noqa
        # simple case, no instance field
        messages[mtype] = msg
        return
    instance_value = getattr(msg, msg._instance_field)  # noqa
    if mtype not in messages:
        messages[mtype] = msg.copy()
        messages[mtype]._instances = {}  # noqa
        messages[mtype]._instances[instance_value] = msg  # noqa
        messages["%s[%s]" % (mtype, str(instance_value))] = msg.copy()
        return
    messages[mtype]._instances[instance_value] = msg  # noqa
    prev_instances = messages[mtype]._instances  # noqa
    messages[mtype] = msg.copy()
    messages[mtype]._instances = prev_instances  # noqa
    messages["%s[%s]" % (mtype, str(instance_value))] = msg.copy()


def set_dialect(dialect) -> any:
    """
    set the MAVLink dialect to work with. For example, set_dialect("ardupilotmega")
    """
    # global mavlink, current_dialect
    # HardCoded for now for optimization purposes. Should be reverted to full checking later

    # wire_protocol = "2.0"

    # Should eventually be "micropymavlink.dialects.v20."
    modname = "micropymavlink." + dialect

    gc.enable()
    gc.collect()
    print('available initial memory:', gc.mem_free())  # noqa
    print('importing', modname)  # noqa
    mod = __import__(modname)
    print('memory remaining after import:', gc.mem_free())  # noqa

    components = modname.split('.')
    for comp in components[1:]:
        mod = getattr(mod, comp)
    _current_dialect = dialect
    _mavlink = mod
    return _mavlink, _current_dialect


# Set the default dialect. This is done here as it needs to be after the function declaration
mavlink, current_dialect = set_dialect(Dialect)


class MavFileState(object):
    """state for a particular system id"""

    def __init__(self):
        self.messages = {'MAV': self}
        self.flightmode = "UNKNOWN"
        self.vehicle_type = "UNKNOWN"
        self.mav_type = mavlink.MAV_TYPE_FIXED_WING
        self.mav_autopilot = mavlink.MAV_AUTOPILOT_GENERIC
        self.base_mode = 0
        self.armed = False  # canonical arm state for the vehicle as a whole

        if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
            self.messages['HOME'] = mavlink.MAVLinkGpsRawIntMessage(0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
            mavlink.MAVLink_waypoint_message = mavlink.MAVLinkMissionItemMessage
        else:
            self.messages['HOME'] = mavlink.MAVLink_gps_raw_message(0, 0, 0, 0, 0, 0, 0, 0, 0)


class ParamState(object):
    """
    state for a particular system id/component id pair
    """

    def __init__(self):
        self.params = {}


class MavFile(object):
    """
    a generic mavlink port
    """

    def __init__(self, fd, address, source_system=255, source_component=0, notimestamps=False, _input=True,
                 use_native=default_native):
        global mavfile_global
        if _input:
            mavfile_global = self
        self.fd = fd
        self.sysid = 0
        self.param_sysid = (0, 0)
        self.address = address
        self.timestamp = 0
        self.last_seq = {}
        self.mav_loss = 0
        self.mav_count = 0
        self.param_fetch_start = 0

        # state for each sysid
        self.sysid_state = {self.sysid: MavFileState()}

        # param state for each sysid/compid tuple
        self.param_state = {self.param_sysid: ParamState()}

        # status of param fetch, indexed by sysid,compid tuple
        self.source_system = source_system
        self.source_component = source_component
        self.first_byte = True
        self.robust_parsing = True
        self.mav = mavlink.MAVLink(self, srcsystem=self.source_system, srccomponent=self.source_component,
                                   use_native=use_native)
        self.mav.robust_parsing = self.robust_parsing
        self.logfile = None
        self.logfile_raw = None
        self.start_time = time.time()
        self.message_hooks = []
        self.idle_hooks = []
        self.uptime = 0.0
        self.notimestamps = notimestamps
        self._timestamp = None
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        self.stop_on_EOF = False
        self.portdead = False

    @property
    def target_system(self):
        """
        Needs population.
        """
        return self.sysid

    @property
    def target_component(self):
        """
        Needs population.
        """
        return self.param_sysid[1]

    @target_system.setter
    def target_system(self, value):
        self.sysid = value
        if self.sysid not in self.sysid_state:
            self.sysid_state[self.sysid] = MavFileState()
        if self.sysid != self.param_sysid[0]:
            self.param_sysid = (self.sysid, self.param_sysid[1])
            if self.param_sysid not in self.param_state:
                self.param_state[self.param_sysid] = ParamState()

    @target_component.setter
    def target_component(self, value):
        if value != self.param_sysid[1]:
            self.param_sysid = (self.param_sysid[0], value)
            if self.param_sysid not in self.param_state:
                self.param_state[self.param_sysid] = ParamState()

    @property
    def params(self):
        """
        Needs population.
        """
        if self.param_sysid[1] == 0:
            eff_tuple = (self.sysid, 1)
            if eff_tuple in self.param_state:
                return getattr(self.param_state[eff_tuple], 'params')
        return getattr(self.param_state[self.param_sysid], 'params')

    @property
    def messages(self):
        """
        Needs population.
        """
        return getattr(self.sysid_state[self.sysid], 'messages')

    @property
    def flightmode(self):
        """
        Needs population.
        """
        return getattr(self.sysid_state[self.sysid], 'flightmode')

    @flightmode.setter
    def flightmode(self, value):
        setattr(self.sysid_state[self.sysid], 'flightmode', value)

    @property
    def vehicle_type(self):
        """
        Needs population.
        """
        return getattr(self.sysid_state[self.sysid], 'vehicle_type')

    @vehicle_type.setter
    def vehicle_type(self, value):
        setattr(self.sysid_state[self.sysid], 'vehicle_type', value)

    @property
    def mav_type(self):
        """
        Needs population.
        """
        return getattr(self.sysid_state[self.sysid], 'mav_type')

    @mav_type.setter
    def mav_type(self, value):
        setattr(self.sysid_state[self.sysid], 'mav_type', value)

    @property
    def base_mode(self):
        """
        Needs population.
        """
        return getattr(self.sysid_state[self.sysid], 'base_mode')

    @base_mode.setter
    def base_mode(self, value):
        setattr(self.sysid_state[self.sysid], 'base_mode', value)

    def auto_mavlink_version(self, buf):
        """auto-switch mavlink protocol version"""
        global mavlink
        if len(buf) == 0:
            return
        try:
            magic = ord(buf[0])
        except (IndexError, TypeError):
            magic = buf[0]
        if magic not in [85, 254, 253]:
            return
        self.first_byte = False
        if self.WIRE_PROTOCOL_VERSION == "0.9" and magic == 254:
            self.WIRE_PROTOCOL_VERSION = "1.0"
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION == "1.0" and magic == 85:
            self.WIRE_PROTOCOL_VERSION = "0.9"
            set_dialect(current_dialect)
        elif self.WIRE_PROTOCOL_VERSION != "2.0" and magic == 253:
            self.WIRE_PROTOCOL_VERSION = "2.0"
            set_dialect(current_dialect)
        else:
            return
        # switch protocol 
        (callback, callback_args, callback_kwargs) = (self.mav.callback,
                                                      self.mav.callback_args,
                                                      self.mav.callback_kwargs)
        self.mav = mavlink.MAVLink(self, srcSystem=self.source_system, srcComponent=self.source_component)
        self.mav.robust_parsing = self.robust_parsing
        self.WIRE_PROTOCOL_VERSION = mavlink.WIRE_PROTOCOL_VERSION
        (self.mav.callback, self.mav.callback_args, self.mav.callback_kwargs) = (callback,
                                                                                 callback_args,
                                                                                 callback_kwargs)

    def recv(self, n=None):
        """default recv method"""
        raise RuntimeError('no recv() method supplied')

    def close(self, n=None):
        """default close method"""
        raise RuntimeError('no close() method supplied')

    def write(self, buf):
        """default write method"""
        raise RuntimeError('no write() method supplied')

    def select(self, timeout):
        """wait for up to timeout seconds for more data"""
        if self.fd is None:
            time.sleep(min(timeout, 0.5))
            return True
        try:
            (rin, win, xin) = select.select([self.fd], [], [], timeout)
        except select.error:
            return False
        return len(rin) == 1

    def pre_message(self):
        """default pre message call"""
        return

    def set_rtscts(self, enable):
        """enable/disable RTS/CTS if applicable"""
        return

    @staticmethod
    def probably_vehicle_heartbeat(msg):
        """
        Needs population.
        """
        if msg.get_srccomponent() == mavlink.MAV_COMP_ID_GIMBAL:
            return False
        if msg.type in (mavlink.MAV_TYPE_GCS,
                        mavlink.MAV_TYPE_GIMBAL,
                        mavlink.MAV_TYPE_ADSB,
                        mavlink.MAV_TYPE_ONBOARD_CONTROLLER):
            return False
        return True

    def r_tuple(self, msg):
        """
        Removing duplicates.
        """

    def post_message(self, msg):
        """
        default post message call
        """

        if '_posted' in msg.__dict__:
            return
        msg._posted = True
        msg._timestamp = time.time()
        _type = msg.get_type()

        if 'usec' in msg.__dict__:
            self.uptime = msg.usec * 1.0e-6
        if 'time_boot_ms' in msg.__dict__:
            self.uptime = msg.time_boot_ms * 1.0e-3

        if self._timestamp is not None:
            if self.notimestamps:
                msg._timestamp = self.uptime
            else:
                msg._timestamp = self._timestamp

        src_system = msg.get_srcsystem()
        src_component = msg.get_srccomponent()
        src_tuple = (src_system, src_component)

        radio_tuple = (ord('3'), ord('D'))

        if src_system not in self.sysid_state:
            # we've seen a new system
            self.sysid_state[src_system] = MavFileState()

        add_message(self.sysid_state[src_system].messages, _type, msg)

        if src_tuple == radio_tuple:
            # as a special case radio msgs are added for all sysids
            for s in self.sysid_state.keys():
                self.sysid_state[s].messages[_type] = msg

        if not (src_tuple == radio_tuple or msg.get_type() == 'BAD_DATA'):
            if src_tuple not in self.last_seq:
                last_seq = -1
            else:
                last_seq = self.last_seq[src_tuple]
            seq = (last_seq + 1) % 256
            seq2 = msg.get_seq()
            if seq != seq2 and last_seq != -1:
                diff = (seq2 - seq) % 256
                self.mav_loss += diff
                # print("lost %u seq=%u seq2=%u last_seq=%u src_tupe=%s %s" % (diff, seq, seq2, last_seq, str(src_tuple), msg.get_type()))
            self.last_seq[src_tuple] = seq2
            self.mav_count += 1

        self.timestamp = msg._timestamp
        if _type == 'HEARTBEAT' and self.probably_vehicle_heartbeat(msg):
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            if float(mavlink.WIRE_PROTOCOL_VERSION) >= 1:
                self.flightmode = mode_string_v10(msg)
                self.mav_type = msg.type
                self.base_mode = msg.base_mode
                self.sysid_state[self.sysid].armed = (msg.base_mode & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.sysid_state[self.sysid].mav_type = msg.type
                self.sysid_state[self.sysid].mav_autopilot = msg.autopilot
        elif _type == 'HIGH_LATENCY2':
            if self.sysid == 0:
                # lock onto id tuple of first vehicle heartbeat
                self.sysid = src_system
            self.flightmode = mode_string_v10(msg)
            self.mav_type = msg.type
            if msg.autopilot == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.base_mode = msg.custom0
                self.sysid_state[self.sysid].armed = (msg.custom0 & mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.sysid_state[self.sysid].mav_type = msg.type
            self.sysid_state[self.sysid].mav_autopilot = msg.autopilot

        elif _type == 'PARAM_VALUE':
            if src_tuple not in self.param_state:
                self.param_state[src_tuple] = ParamState()
            self.param_state[src_tuple].params[msg.param_id] = msg.param_value
        elif _type == 'SYS_STATUS' and mavlink.WIRE_PROTOCOL_VERSION == '0.9':
            self.flightmode = mode_string_v09(msg)
        elif _type == 'GPS_RAW':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 2:
                self.sysid_state[src_system].messages['HOME'] = msg
        elif _type == 'GPS_RAW_INT':
            if self.sysid_state[src_system].messages['HOME'].fix_type < 3:
                self.sysid_state[src_system].messages['HOME'] = msg
        for hook in self.message_hooks:
            hook(self, msg)

        if (msg.get_signed() and
                self.mav.signing.link_id == 0 and
                msg.get_link_id() != 0 and
                self.target_system == msg.get_srcSystem() and
                self.target_component == msg.get_srcComponent()):
            # change to link_id from incoming packet
            self.mav.signing.link_id = msg.get_link_id()

    def packet_loss(self):
        """
        packet loss as a percentage
        """
        if self.mav_count == 0:
            return 0
        return (100.0 * self.mav_loss) / (self.mav_count + self.mav_loss)

    def recv_msg(self):
        """
        message receive routine
        """
        self.pre_message()
        while True:
            n = self.mav.bytes_needed()
            s = self.recv(n)
            numnew = len(s)

            if numnew != 0:
                if self.logfile_raw:
                    if is_py3:
                        self.logfile_raw.write(s)
                    else:
                        self.logfile_raw.write(str(s))
                if self.first_byte:
                    self.auto_mavlink_version(s)

            # We always call parse_char even if the new string is empty, because the existing message buf might already have some valid packet
            # we can extract
            msg = self.mav.parse_char(s)
            if msg:
                print('message', msg)
            if msg:
                if self.logfile and msg.get_type() != 'BAD_DATA':
                    usec = int(time.time() * 1.0e6) & ~3
                    if is_py3:
                        self.logfile.write(struct.pack('>Q', usec) + msg.get_msgbuf())
                    else:
                        self.logfile.write(str(struct.pack('>Q', usec) + msg.get_msgbuf()))
                self.post_message(msg)
                return msg
            else:
                # if we failed to parse any messages _and_ no new bytes arrived, return immediately so the client has the option to
                # timeout
                if numnew == 0:
                    return None

    def recv_match(self, condition=None, _type=None, blocking=False, timeout=None):
        """
        recv the next MAVLink message that matches the given condition
        type can be a string or a list of strings
        """
        if _type is not None and not isinstance(_type, list) and not isinstance(_type, set):
            _type = [_type]
        start_time = time.time()
        while True:
            if timeout is not None:
                now = time.time()
                if now < start_time:
                    start_time = now  # If an external process rolls back system time, we should not spin forever.
                if start_time + timeout < time.time():
                    return None
            m = self.recv_msg()
            if m is None:
                if blocking:
                    for hook in self.idle_hooks:
                        hook(self)
                    if timeout is None:
                        self.select(0.05)
                    else:
                        self.select(timeout / 2)
                    continue
                return None
            if _type is not None and not m.get_type() in _type:
                continue
            if not evaluate_condition(condition, self.messages):
                continue
            return m

    def check_condition(self, condition):
        """check if a condition is true"""
        return evaluate_condition(condition, self.messages)

    def mavlink10(self):
        """return True if using MAVLink 1.0 or later"""
        return float(self.WIRE_PROTOCOL_VERSION) >= 1

    def mavlink20(self):
        """return True if using MAVLink 2.0 or later"""
        return float(self.WIRE_PROTOCOL_VERSION) >= 2

    def setup_logfile(self, logfile, mode='wb'):
        """start logging to the given logfile, with timestamps"""
        self.logfile = open(logfile, mode=mode)

    def setup_logfile_raw(self, logfile, mode='wb'):
        """start logging raw bytes to the given logfile, without timestamps"""
        self.logfile_raw = open(logfile, mode=mode)

    def wait_heartbeat(self, blocking=True, timeout=None):
        # noinspection GrazieInspection
        """wait for a heartbeat so we know the target system IDs"""
        return self.recv_match(_type='HEARTBEAT', blocking=blocking, timeout=timeout)

    def param_fetch_all(self):
        """initiate fetch of all parameters"""
        if time.time() - self.param_fetch_start < 2.0:
            # don't fetch too often
            return
        self.param_fetch_start = time.time()
        self.mav.param_request_list_send(self.target_system, self.target_component)

    def param_fetch_one(self, name):
        """initiate fetch of one parameter"""
        try:
            idx = int(name)
            self.mav.param_request_read_send(self.target_system, self.target_component, b"", idx)
        except Exception:  # TODO: Exception clause too broad
            if sys.version_info.major >= 3 and not isinstance(name, bytes):
                name = bytes(name, 'ascii')
            self.mav.param_request_read_send(self.target_system, self.target_component, name, -1)

    def time_since(self, mtype):
        """return the time since the last message of type mtype was received"""
        if mtype not in self.messages:
            return time.time() - self.start_time
        return time.time() - self.messages[mtype]._timestamp  # noqa

    def param_set_send(self, parm_name, parm_value, parm_type=None):
        """wrapper for parameter set"""
        if self.mavlink10():
            if parm_type is None:
                parm_type = mavlink.MAVLINK_TYPE_FLOAT
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value, parm_type)
        else:
            self.mav.param_set_send(self.target_system, self.target_component,
                                    parm_name.encode('utf8'), parm_value)

    def distance_sensor_send(self, min_distance, max_distance, current_distance, orientation, _id, covariance=255):
        """send distance sensor values
        For orientation refer to:
        https://mavlink.io/en/messages/common.html#MAV_SENSOR_ROTATION_NONE
        """
        self.mav.distance_sensor_send(min_distance, max_distance, current_distance, 0, _id, orientation, covariance)

    def waypoint_request_list_send(self):
        """wrapper for waypoint_request_list_send"""
        if self.mavlink10():
            self.mav.mission_request_list_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_request_list_send(self.target_system, self.target_component)

    def waypoint_clear_all_send(self):
        """wrapper for waypoint_clear_all_send"""
        if self.mavlink10():
            self.mav.mission_clear_all_send(self.target_system, self.target_component)
        else:
            self.mav.waypoint_clear_all_send(self.target_system, self.target_component)

    def waypoint_request_send(self, seq):
        """wrapper for waypoint_request_send"""
        if self.mavlink10():
            self.mav.mission_request_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_request_send(self.target_system, self.target_component, seq)

    def waypoint_set_current_send(self, seq):
        """wrapper for waypoint_set_current_send"""
        if self.mavlink10():
            self.mav.mission_set_current_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_set_current_send(self.target_system, self.target_component, seq)

    def waypoint_current(self):
        """return current waypoint"""
        if self.mavlink10():
            m = self.recv_match(_type='MISSION_CURRENT', blocking=True)
        else:
            m = self.recv_match(_type='WAYPOINT_CURRENT', blocking=True)
        return m.seq

    def waypoint_count_send(self, seq):
        """wrapper for waypoint_count_send"""
        if self.mavlink10():
            self.mav.mission_count_send(self.target_system, self.target_component, seq)
        else:
            self.mav.waypoint_count_send(self.target_system, self.target_component, seq)

    def set_mode_flag(self, flag, enable):
        # noinspection GrazieInspection
        """
                Enables/ disables MAV_MODE_FLAG
                @param flag The mode flag,
                  see MAV_MODE_FLAG enum
                @param enable Enable the flag, (True/False)
                """
        if self.mavlink10():
            mode = self.base_mode
            if enable:
                mode = mode | flag
            elif not enable:
                mode = mode & ~flag
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mode,
                                       0, 0, 0, 0, 0, 0)
        else:
            print("Set mode flag not supported")

    def set_mode_auto(self):
        """
        enter auto mode
        """
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_SET_AUTO = 13
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_SET_AUTO)

    def mode_mapping(self):
        """
        return dictionary mapping mode names to numbers, or None if unknown
        """
        _mav_type = self.sysid_state[self.sysid].mav_type
        mav_autopilot = self.sysid_state[self.sysid].mav_autopilot
        if mav_autopilot == mavlink.MAV_AUTOPILOT_PX4:
            return px4_map
        if _mav_type is None:
            return None
        return mode_mapping_byname(_mav_type)

    def set_mode_apm(self, mode, custom_mode=0, custom_sub_mode=0):  # noqa
        """
        enter arbitrary mode
        """
        if isinstance(mode, str):
            _mode_map = self.mode_mapping()
            if _mode_map is None or mode not in _mode_map:
                print("Unknown mode '%s'" % mode)
                return
            mode = _mode_map[mode]
        # set mode by integer mode number for ArduPilot
        self.mav.command_long_send(self.target_system,
                                   self.target_component,
                                   mavlink.MAV_CMD_DO_SET_MODE,
                                   0,
                                   mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   mode,
                                   0,
                                   0,
                                   0,
                                   0,
                                   0)

    def set_mode_px4(self, mode, custom_mode, custom_sub_mode):
        """
        enter arbitrary mode
        """
        if isinstance(mode, str):
            _mode_map = self.mode_mapping()
            if _mode_map is None or mode not in _mode_map:
                print("Unknown mode '%s'" % mode)
                return
            # PX4 uses two fields to define modes
            mode, custom_mode, custom_sub_mode = px4_map[mode]
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_DO_SET_MODE, 0, mode, custom_mode, custom_sub_mode, 0, 0, 0, 0)

    def set_mode(self, mode, custom_mode=0, custom_sub_mode=0):
        """set arbitrary flight mode"""
        mav_autopilot = self.field('HEARTBEAT', 'autopilot')
        if mav_autopilot == mavlink.MAV_AUTOPILOT_PX4:
            self.set_mode_px4(mode, custom_mode, custom_sub_mode)
        else:
            self.set_mode_apm(mode)

    def set_mode_rtl(self):
        """enter RTL mode"""
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_RETURN = 3
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_RETURN)

    def set_mode_manual(self):
        """enter MANUAL mode"""
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavlink.MAV_MODE_MANUAL_ARMED,
                                       0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_SET_MANUAL = 12
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_SET_MANUAL)

    def set_mode_fbwa(self):
        """enter FBWA mode"""
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_DO_SET_MODE, 0,
                                       mavlink.MAV_MODE_STABILIZE_ARMED,
                                       0, 0, 0, 0, 0, 0)
        else:
            print("Forcing FBWA not supported")

    def set_mode_loiter(self):
        """enter LOITER mode"""
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 0, 0, 0, 0, 0)
        else:
            MAV_ACTION_LOITER = 27
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_LOITER)

    def set_servo(self, channel, pwm):
        """set a servo value"""
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_DO_SET_SERVO, 0,
                                   channel, pwm,
                                   0, 0, 0, 0, 0)

    def set_relay(self, relay_pin=0, state=True):
        """Set relay_pin to value of state"""
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,  # target_component
                mavlink.MAV_CMD_DO_SET_RELAY,  # command
                0,  # Confirmation
                relay_pin,  # Relay Number
                int(state),  # state (1 to indicate arm)
                0,  # param3 (all other params meaningless)
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
        else:
            print("Setting relays not supported.")

    def calibrate_level(self):
        """calibrate accels (1D version)"""
        self.mav.command_long_send(self.target_system, self.target_component,
                                   mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                   1, 1, 0, 0, 0, 0, 0)

    def calibrate_pressure(self):
        """calibrate pressure"""
        if self.mavlink10():
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_CALIBRATION, 0,
                                       0, 0, 1, 0, 0, 0, 0)
        else:
            MAV_ACTION_CALIBRATE_PRESSURE = 20
            self.mav.action_send(self.target_system, self.target_component, MAV_ACTION_CALIBRATE_PRESSURE)

    def reboot_autopilot(self, hold_in_bootloader=False):
        """reboot the autopilot"""
        if self.mavlink10():
            if hold_in_bootloader:
                param1 = 3
            else:
                param1 = 1
            self.mav.command_long_send(self.target_system, self.target_component,
                                       mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                       param1, 0, 0, 0, 0, 0, 0)

    def wait_gps_fix(self):
        """
        Needs population.
        """
        self.recv_match(_type='VFR_HUD', blocking=True)
        if self.mavlink10():
            self.recv_match(_type='GPS_RAW_INT', blocking=True,
                            condition='GPS_RAW_INT.fix_type>=3 and GPS_RAW_INT.lat != 0')
        else:
            self.recv_match(_type='GPS_RAW', blocking=True,
                            condition='GPS_RAW.fix_type>=2 and GPS_RAW.lat != 0')

    def location(self, relative_alt=False):
        """return current location"""
        self.wait_gps_fix()
        # wait for another VFR_HUD, to ensure we have correct altitude
        self.recv_match(_type='VFR_HUD', blocking=True)
        self.recv_match(_type='GLOBAL_POSITION_INT', blocking=True)
        if relative_alt:
            alt = self.messages['GLOBAL_POSITION_INT'].relative_alt * 0.001
        else:
            alt = self.messages['VFR_HUD'].alt
        return Location(self.messages['GPS_RAW_INT'].lat * 1.0e-7,
                        self.messages['GPS_RAW_INT'].lon * 1.0e-7,
                        alt,
                        self.messages['VFR_HUD'].heading)

    def arducopter_arm(self):
        """arm motors (arducopter only)"""
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                0,  # confirmation
                1,  # param1 (1 to indicate arm)
                0,  # param2 (all other params meaningless)
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7

    def arducopter_disarm(self):
        """disarm motors (arducopter only)"""
        if self.mavlink10():
            self.mav.command_long_send(
                self.target_system,  # target_system
                self.target_component,
                mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                0,  # confirmation
                0,  # param1 (0 to indicate disarm)
                0,  # param2 (all other params meaningless)
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7

    def motors_armed(self):
        """return true if motors armed"""
        return self.sysid_state[self.sysid].armed

    def motors_armed_wait(self):
        """wait for motors to be armed"""
        while True:
            self.wait_heartbeat()
            if self.motors_armed():
                return

    def motors_disarmed_wait(self):
        """wait for motors to be disarmed"""
        while True:
            self.wait_heartbeat()
            if not self.motors_armed():
                return

    def field(self, _type, field, default=None):
        """convenient function for returning an arbitrary MAVLink
           field with a default"""
        if _type not in self.messages:
            return default
        return getattr(self.messages[_type], field, default)

    def param(self, name, default=None):
        """convenient function for returning an arbitrary MAVLink
           parameter with a default"""
        if name not in self.params:
            return default
        return self.params[name]

    def setup_signing(self, secret_key, sign_outgoing=True, allow_unsigned_callback=None, initial_timestamp=None,
                      link_id=None):
        """setup for MAVLink2 signing"""
        self.mav.signing.secret_key = secret_key
        self.mav.signing.sign_outgoing = sign_outgoing
        self.mav.signing.allow_unsigned_callback = allow_unsigned_callback
        if link_id is None:
            # auto-increment the link_id for each link
            global global_link_id
            link_id = global_link_id
            global_link_id = min(global_link_id + 1, 255)
        self.mav.signing.link_id = link_id
        if initial_timestamp is None:
            # timestamp is time since 1/1/2015
            epoch_offset = 1420070400
            now = max(time.time(), epoch_offset)
            initial_timestamp = now - epoch_offset
            initial_timestamp = int(initial_timestamp * 100 * 1000)
        # initial_timestamp is in 10usec units
        self.mav.signing.timestamp = initial_timestamp

    def disable_signing(self):
        """disable MAVLink2 signing"""
        self.mav.signing.secret_key = None
        self.mav.signing.sign_outgoing = False
        self.mav.signing.allow_unsigned_callback = None
        self.mav.signing.link_id = 0
        self.mav.signing.timestamp = 0


def set_close_on_exec(fd):
    """set the clone on exec flag on a file descriptor. Ignore exceptions"""
    try:
        import fcntl
        flags = fcntl.fcntl(fd, fcntl.F_GETFD)
        flags |= fcntl.FD_CLOEXEC
        fcntl.fcntl(fd, fcntl.F_SETFD, flags)
    except Exception:  # TODO: Exception clause too broad
        pass


class FakeSerial:
    """
    Needs population.
    """

    def __init__(self):
        pass

    @staticmethod
    def read(**args):  # noqa
        """
        Needs population.
        """
        return str()

    def write(self, buf):
        """
        Needs population.
        """
        raise Exception("write always fails")

    @staticmethod
    def inwaiting():
        """
        Needs population.
        """
        return 0

    def close(self):
        """
        Needs population.
        """
        pass


class MavSerial(MavFile):
    """a serial mavlink port"""

    def __init__(self, device, baud=57600, autoreconnect=False, source_system=255, source_component=0,
                 use_native=default_native, force_connected=False):
        if ',' in device:
            device, baud = device.split(',')
        self.baud = baud
        self.device = device
        self.autoreconnect = autoreconnect
        self.force_connected = force_connected
        # we rather strangely set the baudrate initially to 1200, then change to the desired
        # baudrate. This works around a kernel bug on some Linux kernels where the baudrate
        # is not set correctly

        # ATTEMPTS TO OPEN UART CONNECTION
        tx = 17
        rx = 16
        if isinstance(device, tuple):
            tx, rx = device
        uart = UART(baudrate=self.baud, tx=tx, rx=rx)
        self.port = uart
        # print("Attempted to establish UART connection")
        MavFile.__init__(self, None, device, source_system=source_system, source_component=source_component,
                         use_native=use_native)
        self.rtscts = False

    def set_baudrate(self, baudrate):
        """set baudrate"""
        try:
            self.port.setbaudrate(baudrate)
        except Exception as err:  # TODO: Exception clause too broad
            print(err)
            # for pySerial 3.0, which doesn't have setBaudrate()
            self.port.baudrate = baudrate

    def close(self, **kwargs):
        """
        Needs population.
        """
        self.port.deinit()

    def recv(self, n=None):
        """
        Needs population.
        """
        if n is None:
            n = self.mav.bytes_needed()
        if self.fd is None:
            waiting = self.port.any()
            if waiting < n:
                n = waiting
        ret = self.port.read(n)
        if ret != b'':
            print('in buff', ret)
        return ret

    def write(self, buf):
        """
        Needs population.
        """
        try:
            # print('out buff', buf)
            return self.port.write(bytes(buf))
        except Exception:  # TODO: Exception clause too broad
            if not self.portdead:
                print("Device %s is dead" % self.device)
            self.portdead = True
            if self.autoreconnect:
                self.reset()
            return -1

    def reset(self):
        """
        Needs population.
        """
        try:
            try:
                uart = UART(baudrate=self.baud, tx=17, rx=16)
                newport = uart
            except (AttributeError, RuntimeError):
                return False
            self.port.deinit()
            self.port = newport
            print("Device %s reopened OK" % self.device)
            self.portdead = False
            self.fd = None
            self.set_baudrate(self.baud)
            return True
        except Exception:  # TODO: Exception clause too broad
            return False


# noinspection PyUnusedLocal
def mavlink_connection(device, baud=57600, source_system=255, source_component=0,
                       planner_format=None, write=False, append=False,
                       robust_parsing=True, notimestamps=False, _input=True,
                       dialect=None, autoreconnect=False, zero_time_base=False,
                       retries=3, use_native=default_native,
                       force_connected=False, progress_callback=None,
                       udp_timeout=0, **opts):
    """open a serial, UDP, TCP or file mavlink connection"""
    global mavfile_global

    if force_connected:
        # force_connected implies autoreconnect
        autoreconnect = True

    if dialect is not None:
        set_dialect(dialect)
    return MavSerial(device,
                     baud=baud,
                     source_system=source_system,
                     source_component=source_component,
                     autoreconnect=autoreconnect,
                     use_native=use_native,
                     force_connected=force_connected)


class PeriodicEvent(object):
    """a class for fixed frequency events"""

    def __init__(self, frequency):
        self.frequency = float(frequency)
        self.last_time = time.time()

    def force(self):
        """force immediate triggering"""
        self.last_time = 0

    def trigger(self):
        """return True if we should trigger now"""
        tnow = time.time()

        if tnow < self.last_time:
            print("Warning, time moved backwards. Restarting timer.")
            self.last_time = tnow

        if self.last_time + (1.0 / self.frequency) <= tnow:
            self.last_time = tnow
            return True
        return False


try:
    from curses import ascii

    have_ascii = True
except ImportError:
    have_ascii = False


def is_printable(c):
    """see if a character is printable"""
    global have_ascii
    if have_ascii:
        return ascii.isprint(c)
    if isinstance(c, int):
        ic = c
    else:
        ic = ord(c)
    return 32 <= ic <= 126


def all_printable(buf):
    """see if a string is all printable"""
    for c in buf:
        if not is_printable(c) and c not in ['\r', '\n', '\t'] and c not in [ord('\r'), ord('\n'), ord('\t')]:
            return False
    return True


class SerialPort(object):
    """auto-detected serial port"""

    def __init__(self, device, description=None, hwid=None):
        self.device = device
        self.description = description
        self.hwid = hwid

    def __str__(self):
        ret = self.device
        if self.description is not None:
            ret += " : " + self.description
        if self.hwid is not None:
            ret += " : " + self.hwid
        return ret


# noinspection PyUnusedLocal
def mode_string_v09(msg):
    """mode string for 0.9 protocol"""
    mode = msg.mode
    nav_mode = msg.nav_mode

    MAV_MODE_UNINIT = 0
    MAV_MODE_MANUAL = 2
    MAV_MODE_GUIDED = 3
    MAV_MODE_AUTO = 4
    MAV_MODE_TEST1 = 5
    MAV_MODE_TEST2 = 6
    MAV_MODE_TEST3 = 7

    MAV_NAV_GROUNDED = 0
    MAV_NAV_LIFTOFF = 1
    MAV_NAV_HOLD = 2
    MAV_NAV_WAYPOINT = 3
    MAV_NAV_VECTOR = 4
    MAV_NAV_RETURNING = 5
    MAV_NAV_LANDING = 6
    MAV_NAV_LOST = 7
    MAV_NAV_LOITER = 8

    cmode = (mode, nav_mode)
    mapping = {
        (MAV_MODE_UNINIT, MAV_NAV_GROUNDED): "INITIALISING",
        (MAV_MODE_MANUAL, MAV_NAV_VECTOR): "MANUAL",
        (MAV_MODE_TEST3, MAV_NAV_VECTOR): "CIRCLE",
        (MAV_MODE_GUIDED, MAV_NAV_VECTOR): "GUIDED",
        (MAV_MODE_TEST1, MAV_NAV_VECTOR): "STABILIZE",
        (MAV_MODE_TEST2, MAV_NAV_LIFTOFF): "FBWA",
        (MAV_MODE_AUTO, MAV_NAV_WAYPOINT): "AUTO",
        (MAV_MODE_AUTO, MAV_NAV_RETURNING): "RTL",
        (MAV_MODE_AUTO, MAV_NAV_LOITER): "LOITER",
        (MAV_MODE_AUTO, MAV_NAV_LIFTOFF): "TAKEOFF",
        (MAV_MODE_AUTO, MAV_NAV_LANDING): "LANDING",
        (MAV_MODE_AUTO, MAV_NAV_HOLD): "LOITER",
        (MAV_MODE_GUIDED, MAV_NAV_VECTOR): "GUIDED",
        (MAV_MODE_GUIDED, MAV_NAV_WAYPOINT): "GUIDED",
        (100, MAV_NAV_VECTOR): "STABILIZE",
        (101, MAV_NAV_VECTOR): "ACRO",
        (102, MAV_NAV_VECTOR): "ALT_HOLD",
        (107, MAV_NAV_VECTOR): "CIRCLE",
        (109, MAV_NAV_VECTOR): "LAND",
    }
    if cmode in mapping:
        return mapping[cmode]
    return "Mode(%s,%s)" % cmode


mode_mapping_apm = {
    0: 'MANUAL',
    1: 'CIRCLE',
    2: 'STABILIZE',
    3: 'TRAINING',
    4: 'ACRO',
    5: 'FBWA',
    6: 'FBWB',
    7: 'CRUISE',
    8: 'AUTOTUNE',
    10: 'AUTO',
    11: 'RTL',
    12: 'LOITER',
    13: 'TAKEOFF',
    14: 'AVOID_ADSB',
    15: 'GUIDED',
    16: 'INITIALISING',
    17: 'QSTABILIZE',
    18: 'QHOVER',
    19: 'QLOITER',
    20: 'QLAND',
    21: 'QRTL',
    22: 'QAUTOTUNE',
    23: 'QACRO',
    24: 'THERMAL',
    25: 'LOITERALTQLAND',
}

mode_mapping_acm = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    5: 'LOITER',
    6: 'RTL',
    7: 'CIRCLE',
    8: 'POSITION',
    9: 'LAND',
    10: 'OF_LOITER',
    11: 'DRIFT',
    13: 'SPORT',
    14: 'FLIP',
    15: 'AUTOTUNE',
    16: 'POSHOLD',
    17: 'BRAKE',
    18: 'THROW',
    19: 'AVOID_ADSB',
    20: 'GUIDED_NOGPS',
    21: 'SMART_RTL',
    22: 'FLOWHOLD',
    23: 'FOLLOW',
    24: 'ZIGZAG',
    25: 'SYSTEMID',
    26: 'AUTOROTATE',
    27: 'AUTO_RTL',
}

mode_mapping_rover = {
    0: 'MANUAL',
    1: 'ACRO',
    2: 'LEARNING',
    3: 'STEERING',
    4: 'HOLD',
    5: 'LOITER',
    6: 'FOLLOW',
    7: 'SIMPLE',
    10: 'AUTO',
    11: 'RTL',
    12: 'SMART_RTL',
    15: 'GUIDED',
    16: 'INITIALISING'
}

mode_mapping_tracker = {
    0: 'MANUAL',
    1: 'STOP',
    2: 'SCAN',
    4: 'GUIDED',
    10: 'AUTO',
    16: 'INITIALISING'
}

mode_mapping_sub = {
    0: 'STABILIZE',
    1: 'ACRO',
    2: 'ALT_HOLD',
    3: 'AUTO',
    4: 'GUIDED',
    7: 'CIRCLE',
    9: 'SURFACE',
    16: 'POSHOLD',
    19: 'MANUAL',
}

mode_mapping_blimp = {
    0: 'LAND',
    1: 'MANUAL',
    2: 'VELOCITY',
    3: 'LOITER',
}

AP_MAV_TYPE_MODE_MAP_DEFAULT = {
    # copter
    mavlink.MAV_TYPE_HELICOPTER: mode_mapping_acm,
    mavlink.MAV_TYPE_TRICOPTER: mode_mapping_acm,
    mavlink.MAV_TYPE_QUADROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_HEXAROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_OCTOROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_DECAROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_DODECAROTOR: mode_mapping_acm,
    mavlink.MAV_TYPE_COAXIAL: mode_mapping_acm,
    # plane
    mavlink.MAV_TYPE_FIXED_WING: mode_mapping_apm,
    # rover
    mavlink.MAV_TYPE_GROUND_ROVER: mode_mapping_rover,
    # boat
    mavlink.MAV_TYPE_SURFACE_BOAT: mode_mapping_rover,  # for the time being
    # tracker
    mavlink.MAV_TYPE_ANTENNA_TRACKER: mode_mapping_tracker,
    # sub
    mavlink.MAV_TYPE_SUBMARINE: mode_mapping_sub,
    # blimp
    mavlink.MAV_TYPE_AIRSHIP: mode_mapping_blimp,
}

try:
    # Allow for using custom mode maps by importing a JSON dict from
    # "~/.pymavlink/custom_mode_map.json" and using it to extend the hard-coded
    # AP_MAV_TYPE_MODE_MAP_DEFAULT dict.
    from os.path import expanduser

    _custom_mode_map_path = os.path.join("~", ".pymavlink", "custom_mode_map.json")
    _custom_mode_map_path = expanduser(_custom_mode_map_path)
    try:
        with open(_custom_mode_map_path) as f:
            _json_mode_map = json.load(f)
    except json.decoder.JSONDecodeError as ex:
        # inform the user of a malformed custom_mode_map.json
        print("Error: pymavlink custom mode file ('" + _custom_mode_map_path + "') is not valid JSON.")
        raise
    except Exception:  # TODO: Exception clause too broad
        # file is not present, fall back to using default map
        raise

    try:
        _custom_mode_map = {}
        for mav_type, mode_map in _json_mode_map.items():
            # make sure the custom map has the right datatypes
            _custom_mode_map[int(mav_type)] = {int(mode_num): str(mode_name) for mode_num, mode_name in
                                               mode_map.items()}
    except Exception:  # TODO: Exception clause too broad
        # inform the user of invalid custom mode map
        print("Error: invalid pymavlink custom mode map dict in " + _custom_mode_map_path)
        raise

    AP_MAV_TYPE_MODE_MAP = AP_MAV_TYPE_MODE_MAP_DEFAULT.copy()
    AP_MAV_TYPE_MODE_MAP.update(_custom_mode_map)
except Exception:  # TODO: Exception clause too broad
    # revert to using default mode map
    AP_MAV_TYPE_MODE_MAP = AP_MAV_TYPE_MODE_MAP_DEFAULT

# map from a PX4 "main_state" to a string; see msg/commander_state.msg
# This allows us to map sdlog STAT.MainState to a simple "mode"
# string, used in DFReader and possibly other places.  These are
# related but distict from what is found in mavlink messages; see
# "Custom mode definitions", below.
mainstate_mapping_px4 = {
    0: 'MANUAL',
    1: 'ALTCTL',
    2: 'POSCTL',
    3: 'AUTO_MISSION',
    4: 'AUTO_LOITER',
    5: 'AUTO_RTL',
    6: 'ACRO',
    7: 'OFFBOARD',
    8: 'STAB',
    9: 'RATTITUDE',
    10: 'AUTO_TAKEOFF',
    11: 'AUTO_LAND',
    12: 'AUTO_FOLLOW_TARGET',
    13: 'MAX',
}


def mode_string_px4(mainstate):
    """
    Needs population.
    """
    return mainstate_mapping_px4.get(mainstate, "Unknown")


# Custom mode definitions from PX4
PX4_CUSTOM_MAIN_MODE_MANUAL = 1
PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
PX4_CUSTOM_MAIN_MODE_POSCTL = 3
PX4_CUSTOM_MAIN_MODE_AUTO = 4
PX4_CUSTOM_MAIN_MODE_ACRO = 5
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
PX4_CUSTOM_MAIN_MODE_STABILIZED = 7
PX4_CUSTOM_MAIN_MODE_RATTITUDE = 8

PX4_CUSTOM_SUB_MODE_OFFBOARD = 0
PX4_CUSTOM_SUB_MODE_AUTO_READY = 1
PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3
PX4_CUSTOM_SUB_MODE_AUTO_MISSION = 4
PX4_CUSTOM_SUB_MODE_AUTO_RTL = 5
PX4_CUSTOM_SUB_MODE_AUTO_LAND = 6
PX4_CUSTOM_SUB_MODE_AUTO_RTGS = 7
PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET = 8

auto_mode_flags = mavlink.MAV_MODE_FLAG_AUTO_ENABLED \
                  | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED \
                  | mavlink.MAV_MODE_FLAG_GUIDED_ENABLED

px4_map = {"MANUAL": (
    mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
    PX4_CUSTOM_MAIN_MODE_MANUAL, 0),
    "STABILIZED": (
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        PX4_CUSTOM_MAIN_MODE_STABILIZED, 0),
    "ACRO": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
             PX4_CUSTOM_MAIN_MODE_ACRO, 0),
    "RATTITUDE": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
                  PX4_CUSTOM_MAIN_MODE_RATTITUDE, 0),
    "ALTCTL": (
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        PX4_CUSTOM_MAIN_MODE_ALTCTL, 0),
    "POSCTL": (
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED | mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        PX4_CUSTOM_MAIN_MODE_POSCTL, 0),
    "LOITER": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
               PX4_CUSTOM_SUB_MODE_AUTO_LOITER),
    "MISSION": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
                PX4_CUSTOM_SUB_MODE_AUTO_MISSION),
    "RTL": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
            PX4_CUSTOM_SUB_MODE_AUTO_RTL),
    "LAND": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
             PX4_CUSTOM_SUB_MODE_AUTO_LAND),
    "RTGS": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
             PX4_CUSTOM_SUB_MODE_AUTO_RTGS),
    "FOLLOWME": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
                 PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET),
    "OFFBOARD": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0),
    "TAKEOFF": (mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | auto_mode_flags, PX4_CUSTOM_MAIN_MODE_AUTO,
                PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF)}


def interpret_px4_mode(base_mode, custom_mode):
    """
    Needs population.
    """
    custom_main_mode = (custom_mode & 0xFF0000) >> 16
    custom_sub_mode = (custom_mode & 0xFF000000) >> 24

    if base_mode & mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED != 0:  # manual modes
        if custom_main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL:
            return "MANUAL"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_ACRO:
            return "ACRO"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_RATTITUDE:
            return "RATTITUDE"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED:
            return "STABILIZED"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL:
            return "ALTCTL"
        elif custom_main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL:
            return "POSCTL"
    elif (base_mode & auto_mode_flags) == auto_mode_flags:  # auto modes
        if custom_main_mode & PX4_CUSTOM_MAIN_MODE_AUTO != 0:
            if custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_MISSION:
                return "MISSION"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF:
                return "TAKEOFF"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LOITER:
                return "LOITER"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET:
                return "FOLLOWME"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTL:
                return "RTL"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_LAND:
                return "LAND"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_AUTO_RTGS:
                return "RTGS"
            elif custom_sub_mode == PX4_CUSTOM_SUB_MODE_OFFBOARD:
                return "OFFBOARD"
    return "UNKNOWN"


def mode_mapping_byname(_mav_type):
    """
    return dictionary mapping mode names to numbers, or None if unknown
    """
    _mode_map = mode_mapping_bynumber(_mav_type)
    if _mode_map is None:
        return None
    inv_map = dict((a, b) for (b, a) in _mode_map.items())
    return inv_map


def mode_mapping_bynumber(_mav_type):
    """
    return dictionary mapping mode numbers to name, or None if unknown
    """
    return AP_MAV_TYPE_MODE_MAP[_mav_type] if _mav_type in AP_MAV_TYPE_MODE_MAP else None


def mode_string_v10(msg):
    """
    mode string for 1.0 protocol, from heartbeat
    """
    if msg.autopilot == mavlink.MAV_AUTOPILOT_PX4:
        return interpret_px4_mode(msg.base_mode, msg.custom_mode)
    if msg.get_type() != 'HIGH_LATENCY2' and not msg.base_mode & mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
        return "Mode(0x%08x)" % msg.base_mode

    _mode_map = mode_mapping_bynumber(msg.type)
    if _mode_map and msg.custom_mode in _mode_map:
        return _mode_map[msg.custom_mode]
    return "Mode(%u)" % msg.custom_mode


def mode_string_apm(mode_number):
    """
    return mode string for ArduPlane
    """
    if mode_number in mode_mapping_apm:
        return mode_mapping_apm[mode_number]
    return "Mode(%u)" % mode_number


def mode_string_acm(mode_number):
    """
    return mode string for ArduCopter
    """
    if mode_number in mode_mapping_acm:
        return mode_mapping_acm[mode_number]
    return "Mode(%u)" % mode_number


def decode_bitmask(messagetype, field, value):
    """
    Needs population.
    """
    try:
        _class = eval("mavlink.MAVLink_%s_message" % messagetype.lower())
    except AttributeError:
        raise AttributeError("No such message type")

    try:
        display = _class.fielddisplays_by_name[field]
    except KeyError:
        raise AttributeError("Not a bitmask field (none specified)")

    if display != "bitmask":
        raise ValueError("Not a bitmask field")

    try:
        enum_name = _class.fieldenums_by_name[field]
    except KeyError:
        raise AttributeError("No enum found for bitmask field")

    try:
        enum = mavlink.enums[enum_name]
    except KeyError:
        raise AttributeError("Did not find specified enumeration (%s)" % enum_name)

    class EnumBitInfo(object):
        """
        Needs population.
        """

        def __init__(self, offset, _value, name):
            self.offset = offset
            self.value = _value
            self.name = name

    ret = []
    for i in range(0, 64):
        bit_value = (1 << i)
        try:
            enum_entry = enum[bit_value]
            enum_entry_name = enum_entry.name
        except KeyError as e:  # noqa
            enum_entry_name = None
            if value == 0:
                continue
        if value & bit_value:
            ret.append(EnumBitInfo(i, True, enum_entry_name))
            value &= ~bit_value
        else:
            ret.append(EnumBitInfo(i, False, enum_entry_name))
    return ret


def dump_message_verbose(_f=None, m=None):
    """
    write an excruciatingly detailed dump of message m to file descriptor f
    """
    try:
        # __getattr__ may be overridden on m, thus this try/except
        timestamp = m._timestamp  # noqa
    except AttributeError:
        timestamp = ""
    if timestamp != "":
        timestamp = "%s.%02u: " % (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
            int(timestamp * 100.0) % 100)
    if _f:
        _f.write("%s%s (id=%u) (link=%s) (signed=%s) (seq=%u) (src=%u/%u)\n" % (
            timestamp, m.get_type(), m.get_msgId(), str(m.get_link_id()), str(m.get_signed()), m.get_seq(),
            m.get_srcSystem(),
            m.get_srcComponent()))
    else:
        print("%s%s (id=%u) (link=%s) (signed=%s) (seq=%u) (src=%u/%u)\n" % (
            timestamp, m.get_type(), m.get_msgId(), str(m.get_link_id()), str(m.get_signed()), m.get_seq(),
            m.get_srcSystem(),
            m.get_srcComponent()))
    for fieldname in m.get_fieldnames():

        # format in those most boring way possible:
        value = m.format_attr(fieldname)

        # try to add units:
        try:
            units = m.fieldunits_by_name[fieldname]
            # perform simple unit conversions:
            divisor = None
            if units == "d%":
                divisor = 10.0
                units = "%"
            if units == "c%":
                divisor = 100.0
                units = "%"

            if units == "cA":
                divisor = 100.0
                units = "A"

            elif units == "cdegC":
                divisor = 100.0
                units = "degC"

            elif units == "cdeg":
                divisor = 100.0
                units = "deg"

            elif units == "degE7":
                divisor = 10000000.0
                units = "deg"

            elif units == "mG":
                divisor = 1000.0
                units = "G"

            elif units == "mrad/s":
                divisor = 1000.0
                units = "rad/s"

            elif units == "mV":
                divisor = 1000.0
                units = "V"

            if divisor is not None:
                if type(value) == list:
                    value = [x / divisor for x in value]
                else:
                    value /= divisor

            # and give radians in degrees too:
            if units == "rad":
                value = "%s%s (%sdeg)" % (value, units, math.degrees(value))
            elif units == "rad/s":
                value = "%s%s (%sdeg/s)" % (value, units, math.degrees(value))
            elif units == "rad/s/s":
                value = "%s%s (%sdeg/s/s)" % (value, units, math.degrees(value))
            else:
                value = "%s%s" % (value, units)
        except AttributeError:
            # e.g. BAD_DATA
            pass
        except KeyError:
            pass

        # format any bitmask enumerations:
        try:
            enum_name = m.fieldenums_by_name[fieldname]
            display = m.fielddisplays_by_name[fieldname]
            if enum_name is not None and display == "bitmask":
                bits = decode_bitmask(m.get_type(), fieldname, value)
                if f:
                    f.write("    %s: %s\n" % (fieldname, value))
                else:
                    print("    %s: %s\n" % (fieldname, value))
                for bit in bits:
                    value = bit.value
                    name = bit.name
                    svalue = " "
                    if not value:
                        svalue = "!"
                    if name is None:
                        name = "[UNKNOWN]"
                    if f:
                        f.write("      %s %s\n" % (svalue, name))
                    else:
                        print("      %s %s\n" % (svalue, name))
                continue
        #            except NameError as e:
        #                pass
        except AttributeError:
            # e.g. BAD_DATA
            pass
        except KeyError:
            pass

        # add any enumeration name:
        try:
            enum_name = m.fieldenums_by_name[fieldname]
            try:
                enum_value = mavlink.enums[enum_name][value].name
                value = "%s (%s)" % (value, enum_value)
            except KeyError:
                value = "%s (%s)" % (value, "[UNKNOWN]")
        except AttributeError:
            # e.g. BAD_DATA
            pass
        except KeyError:
            pass

        f.write("    %s: %s\n" % (fieldname, value))


if __name__ == '__main__':
    # serial_list = auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*'])
    # for port in serial_list:
    #    print("%s" % port)
    pass
