#!/usr/bin/python3

"""
MIT LICENSE

Copyright (c) 2018-2019 David A. Krause, aka papamac

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

DESCRIPTION

IOMGR version 0.9.0 is an initial operational version that supports analog
input and digital input/output.  Documentation is incomplete.

Device characteristics for current and soon to be supported I/O devices:

  Device              Descriotion              Max No     Port    Channels
   Name                                        Devices    Types   per Port

BCM2835,6,7  Raspberry Pi Baseline GPIO           1     gg0, gp0     7
             GPIOs 17-18, 22-25, 27
BCM2835,6,7  Raspberry Pi Extended GPIO           1     gg1, gp1     9
             GPIOs 5-6, 12-13, 16, 19-21, 26
 MCP23008    8-Bit I/O Expander                   8        ga        8
 MCP23017    16-Bit I/O Expander (A Port)         8        ga        8
 MCP23017    16-Bit I/O Expander (B Port)         8        gb        8
 MCP3204*    4-Channel 12-Bit A/D Converter       8?       ad        4
 MCP3208*    8-Channel 12-Bit A/D Converter       8?       ae        8
 MCP3422,3   2-Channel 18-Bit A/D Converter       4        aa        2
 MCP3424     4-Channel 18-Bit A/D Converter       4        ab        4
 MCP4821*    1-Channel 12-Bit D/A Converter       8?       da        1
 MCP4822*    2-Channel 12-Bit D/A Converter       8?       db        2

 * Implementation in progress.
"""

__author__ = 'papamac'
__version__ = '0.9.5'
__date__ = 'August 9, 2019'

from argparse import ArgumentParser
from datetime import datetime
from logging import DEBUG, WARNING, ERROR
from logging import addLevelName, Formatter, getLogger, StreamHandler
from logging.handlers import TimedRotatingFileHandler
from math import fabs, log2
from pathlib import Path
from queue import *
from sys import argv
from threading import Thread
from time import sleep

from i2cbus import I2CBUS
from nbi import NBI
from pidacs_global import DATA_INT, DATA_CHG, DATA_REQ
from pidacs_global import MESSAGE_LENGTH, DATETIME_LENGTH, DEFAULT_PORT_NUMBER
import RPi.GPIO as GPIO


# Module iomgr global constants:

DEFAULT_PORT_NAMES = 'gg0 gg1'  # ports for Raspberry Pi built-in GPIO.
STATUS_INTERVAL = 600       # Interval for port status reporting (sec).

# Channel configuration constants are used to specify the operational
# characteristics of specific analog and digital channels.  The optional values
# are used in the IOMGR._REQUESTS dictionary to define the valid argument
# values for a user configuration request.  The DEFAULT values are used to set
# default channel attributes when the channels are instantiated. They are also
# used in the IOMGR._REQUESTS dictionary to specify default values to be used
# when a user request omits the argument.

# Options and DEFAULT value for digital channel direction:

INPUT = 1                   # Channel is a digital input.
OUTPUT = 0                  # Channel is a digital output.
DIRECTION = INPUT           # DEFAULT is INPUT

# Options and DEFAULT values for pulse width modulation (PWM):

DUTYCYCLE = 50              # PWM duty cycle in percent (0% - continuous off to
#                             100% - continuous on).  DEFAULT is 50% - half on,
#                             half off.
FREQUENCY = 100             # Pulse frequency in Hz for PWM digital outputs.
#                             DEFAULT is 100 Hz.
START = 1                   # Start PWM output.
STOP = 0                    # Stop PWM output and delete current PWM object.
OPERATION = START           # DEFAULT is START.

# Options and DEFAULT value for digital channel polarity:

NORMAL = 0                  # Normal logic polarity (logic low = 0, high = 1).
INVERTED = 1                # Inverted logic polarity (low = 1, high = 0).
POLARITY = NORMAL           # DEFAULT is NORMAL.

# Options and DEFAULT value for digital channel pullup resistor configuration.

OFF = DISABLED = 0          # No internal pullup resistor on digital input.
ON = ENABLED = UP = 1       # Digital input pulled up to logic high through an
#                             internal resistor.
DOWN = 2                    # Digital input pulled down to logic low through an
#                             internal resistor.  Applicable to BCM GPIO only.
PULLUP = DISABLED           # DEFAULT is DISABLED.

# Options and DEFAULT values for analog channel configuration parameters:

RESOLUTION = 12             # ADC resolution is 12, 14, 16, or 18 bits.
#                             DEFAULT is 12 bits.
GAIN = 1                    # ADC gain is 1, 2, 4, or 8.
#                             DEFAULT is 1.
SCALING = (6.8 + 10) / 6.8  # The ADC scaling factor can be any real number
#                             providing appropriate scaling for the sensor
#                             circuit being measured by an individual analog
#                             channel.
#                             DEFAULT is for 5V single ended channels on the
#                             AB Electronics ADC Pi Plus board.

# DEFAULT operational arguments for channel momentary contact and write
# methods.  These are used in the the IOMGR._REQUESTS dictionary to specify
# default values to be used when a user request omits the argument.

MOMENTARY = 0.2             # The delay time in seconds from a momentary
#                             channel turn-on to its subsequent turn-off.
#                             DEFAULT is 0.2 sec.
WRITE = OFF                 # DEFAULT is OFF.

# Options and DEFAULT values for change detection and interval reporting:

NONE = 0                    # No change detection or interval reporting.
YES = 1                     # Change detection or interval reporting is
#                             enabled.
#                             Any positive, non-zero number (integer or float)
#                             enables change detection for both analog and
#                             digital channels.  For analog channels the number
#                             is the percentage difference needed to trigger a
#                             change.
#                             Similarly, any positive non-zero number enables
#                             interval reporting.  In this case, the number is
#                             the reporting interval in seconds.
#                             Note that change detection and interval reporting
#                             operate independently.  Either or both can be
#                             independently enabled or disabled for each
#                             channel.
FALSE = 0                   # Same as NONE.
TRUE = 1                    # Same as YES.
CHANGE = NONE               # Change detection DEFAULT is NONE.
INTERVAL = NONE             # Interval reporting DEFAULT is NONE.

# Valid request dictionary used in the IOMGR.process_request method.

#           request_id      DEFAULT and optional argument values

REQUESTS = {'alias':       {'*': None},
            'change':      {'DEFAULT': CHANGE, 'none': NONE, 'yes': YES,
                            'false': FALSE, 'true': TRUE, '#': None},
            'direction':   {'DEFAULT': DIRECTION, 'input': INPUT,
                            'output': OUTPUT, '1': 1, '0': 0},
            'dutycycle':   {'DEFAULT': DUTYCYCLE, '<=': 100},
            'frequency':   {'DEFAULT': FREQUENCY, '<=': 1000},
            'gain':        {'DEFAULT': GAIN, '1': 1, '2': 2, '4': 4, '8': 8},
            'interval':    {'DEFAULT': INTERVAL, 'none': NONE, 'yes': YES,
                            'false': FALSE, 'true': TRUE, '<=': 86400},
            'momentary':   {'DEFAULT': MOMENTARY, '<=': 5},
            'polarity':    {'DEFAULT': POLARITY, 'normal': NORMAL,
                            'inverted': INVERTED, '0': 0, '1': 1},
            'pullup':      {'DEFAULT': PULLUP, 'disabled': DISABLED,
                            'enabled': ENABLED, 'off': OFF, 'on': ON,
                            'down': DOWN, 'up': UP, '0': 0, '1': 1, '2': 2},
            'pwm':         {'DEFAULT': OPERATION, 'start': START,
                            'stop': STOP, '1': 1, '0': 0},
            'read':        {'DEFAULT': None},
            'resolution':  {'DEFAULT': RESOLUTION, '12': 12, '14': 14,
                            '16': 16, '18': 18},
            'scaling':     {'DEFAULT': SCALING, '#': None},
            'write':       {'DEFAULT': WRITE, 'off': OFF, 'on': ON,
                            'false': FALSE, 'true': TRUE, '<=': 5}}


class Port(Thread):
    """
    """
    ports = []  # List of all Port instance objects.

    def __init__(self, name):
        Thread.__init__(self, name=name)
        self._channels = []  # All Channel objects for this Port instance.
        self._queue = Queue()  # Request queue for this Port instance.
        self._running = False

        self.ports.append(self)

    # Public methods:

    def start(self):
        self._running = True
        Thread.start(self)

    def stop(self):
        if self._running:
            self._running = False
            self.join()

    def queue_request(self, channel, request_id, argument):
        self._queue.put((channel, request_id, argument))

    # Private methods used only in the Port execution thread:

    def run(self):
        poll_count = 0
        dt_status = datetime.now()

        # Port execution thread run loop:

        while self._running:

            # Get all currently queued requests and execute them.

            while self._running:
                try:
                    request = self._queue.get(timeout=1)
                except Empty:
                    break
                self._execute_request(*request)

            # Perform periodic polling and status functions.

            if self._running:
                dt_now = datetime.now()
                self._poll(dt_now)
                poll_count += 1
                status_int = (dt_now - dt_status).total_seconds()
                if status_int >= STATUS_INTERVAL:
                    rate = poll_count / status_int
                    IOMGR.queue_message(DEBUG, '%s polling rate = %d '
                                        'per sec' % (self.name, rate))
                    poll_count = 0
                    dt_status = dt_now

    @staticmethod
    def _execute_request(channel, request_id, argument):
        attr = getattr(channel, request_id, None)
        if attr is None:  # Not a method or attribute for this channel.
            warning = ('request id "%s" is not valid for channel "%s"; '
                       'request ignored' % (request_id, channel.id))
            IOMGR.queue_message(WARNING, warning)
        else:  # request_id is a valid attribute or method for this channel.
            if hasattr(attr, '__call__'):  # It is a method; call it.
                try:
                    attr() if argument is None else attr(argument)
                except OSError as err:
                    err_msg = ('error in calling "%s" method for '
                               'channel "%s"; state unknown'
                               % (request_id, channel.id))
                    IOMGR.queue_message(ERROR, err_msg)
                    IOMGR.queue_message(ERROR, err)
            else:  # It is an attribute; set value to argument.
                setattr(channel, request_id, argument)

    def _poll(self, dt_now):
        for channel in self._channels:
            if channel.change or channel.interval:
                try:
                    value = channel.read_hw()
                except OSError as err:
                    self._running = False
                    err_msg = ('polling read error on channel "%s"; port '
                               '"%s" stopped' % (channel.id, self.name))
                    IOMGR.queue_message(ERROR, err_msg)
                    IOMGR.queue_message(ERROR, err)
                    value = '!ERROR'
                channel.prior_value = channel.value
                channel.value = value
                if channel.change and self._value_changed(channel):
                    IOMGR.queue_message(DATA_CHG, channel.id, channel.value)
                interval = (dt_now - channel.prior_report).total_seconds()
                if channel.interval and interval >= channel.interval:
                    IOMGR.queue_message(DATA_INT, channel.id, channel.value)
                    channel.prior_report = dt_now

    def _value_changed(self, channel):
        return channel.value != channel.prior_value


class Channel:
    """
    """
    channels = {}

    def __init__(self, port, name, alt_name=None):
        self.port = port
        self._name = name
        self._alt_name = alt_name
        self._alias = name
        self.id = name
        self.change = CHANGE
        self.interval = INTERVAL
        self.prior_report = datetime.now()
        self.prior_value = None
        self.value = None

        self._direction = INPUT

        self.channels[name] = self
        if alt_name:
            self.channels[alt_name] = self

    # Public methods called by Port.execute_request(channel, request_id,
    # argument):

    def alias(self, alias):
        self._alias = alias
        self.id = '%s[%s]' % (alias, self._name)
        self.channels[alias] = self

    def read(self):
        self.prior_value = self.value
        self.value = self.read_hw()
        IOMGR.queue_message(DATA_REQ, self.id, self.value)

    # Semi-private method called only by read() and the port method _poll();
    # must be replaced by the Channel subclass.

    def read_hw(self):
        return None

    # Private methods:

    @staticmethod
    def _check_bitval(bitval):
        ok = bitval in (0, 1)
        if not ok:
            warning = 'invalid bit value "%s"; request ignored' % bitval
            IOMGR.queue_message(WARNING, warning)
        return ok

    def _check_direction(self, direction):
        ok = self._direction is direction
        if not ok:
            warning = ('channel "%s" not configured for %s; request ignored'
                       % (self.id, ('output', 'input')[direction]))
            IOMGR.queue_message(WARNING, warning)
        return ok

    def _momentary(self, delay):
        if self._check_direction(OUTPUT):
            self._write(ON)
            sleep(delay)
            self._write(OFF)

    def _write(self, value):
        self._write_hw(value)
        self.prior_value = self.value
        self.value = value
        IOMGR.queue_message(DATA_REQ, self.id, value)

    def _write_hw(self, bitval):  # Must be replaced by subclass.
        pass


class BCM283X(Port):
    """
    """
    _CHANNELS = [{17: 11, 18: 12, 22: 15, 23: 16, 24: 18, 25: 22, 27: 13},
                 {5:  29,  6: 31, 12: 32, 13: 33, 16: 36, 19: 35, 20: 38,
                  21: 40, 26: 37}]

    def __init__(self, name):
        Port.__init__(self, name)

        bcm_type = name[:2]  # bcm_type is gg or gp.
        mode = GPIO.BCM if bcm_type == 'gg' else GPIO.BOARD
        GPIO.setmode(mode)
        GPIO.setwarnings(False)

        # Instantiate channels using both gpio number names and pin number
        # names.

        channel_set = int(name[2])  # channel set is 0 or 1.
        for gpio_number in self._CHANNELS[channel_set]:
            pin_number = self._CHANNELS[channel_set][gpio_number]
            gpio_name = 'gg%02i' % gpio_number
            pin_name = 'gp%02i' % pin_number
            if bcm_type == 'gg':
                channel = self._Channel(self, gpio_name, pin_name)
            else:
                channel = self._Channel(self, pin_name, gpio_name)
            self._channels.append(channel)

    class _Channel(Channel):
        """
        """
        def __init__(self, port, name, alt_name):
            Channel.__init__(self, port, name, alt_name)

            self.polarity = POLARITY

            self._number = int(name[2:])
            self._pullup = None
            self._dutycycle = DUTYCYCLE
            self._frequency = FREQUENCY
            self._pwm = None
            self._save_change = self.change
            self._save_interval = self.interval

            # Set default channel configuration and read channel value.

            self.pullup(PULLUP)
            self.direction(DIRECTION)
            self.read()

        # Public methods called by Port.execute_request(channel, request_id,
        # argument):

        def direction(self, direction):
            pullup = (GPIO.PUD_OFF if direction is OUTPUT else self._pullup)
            GPIO.setup(self._number, direction, pullup)
            self._direction = direction

        def dutycycle(self, dutycycle):
            if self._check_direction(OUTPUT):
                self._dutycycle = dutycycle
                if self._pwm:
                    self._pwm.ChangeDutyCycle(self._dutycycle)

        def frequency(self, frequency):
            if self._check_direction(OUTPUT):
                self._frequency = frequency
                if self._pwm:
                    self._pwm.ChangeFrequency(self._frequency)

        def momentary(self, delay):
            self._momentary(delay)

        def pullup(self, pullup):
            if self._check_direction(INPUT):
                pu = 0 if pullup is OFF else 1 if pullup is DOWN else 2
                self._pullup = GPIO.PUD_OFF + pu
                GPIO.setup(self._number, self._direction, self._pullup)

        def pwm(self, operation):
            if self._check_direction(OUTPUT):
                if operation is START:
                    if self._check_direction(OUTPUT):
                        if self._pwm:  # Stop/delete existing pwm, if active.
                            self._pwm.stop()
                            del self._pwm
                        self._save_change = self.change
                        self._save_interval = self.interval
                        self.change = self.interval = NONE
                        self._pwm = GPIO.PWM(self._number, self._frequency)
                        self._pwm.start(self._dutycycle)
                else:  # operation is STOP.
                    if self._pwm:  # Stop/delete existing pwm, if active.
                        self._pwm.stop()
                        del self._pwm
                        self._pwm = None
                        self.change = self._save_change
                        self.interval = self._save_interval

        def write(self, bitval):
            if self._check_direction(OUTPUT):
                if self._check_bitval(bitval):
                    if self._pwm:  # Stop/delete existing pwm, if active.
                        self._pwm.stop()
                        del self._pwm
                        self._pwm = None
                        self.change = self._save_change
                        self.interval = self._save_interval
                    self._write(bitval)

        # Semi-private method called only by read() and the port method
        # _poll().

        def read_hw(self):  # Replaces superclass method.
            return self._apply_polarity(GPIO.input(self._number))

        # Private methods:

        def _apply_polarity(self, bitval):
            if self.polarity:
                bitval = 0 if bitval else 1
            return bitval

        def _write_hw(self, bitval):  # Replaces superclass method.
            GPIO.output(self._number, bitval)


class MCP230XX(Port):
    """
    """
    _I2C_BASE_ADDRESS = 0x20
    _REGISTER_BASE_ADDRESSES = {'IODIR': 0x00, 'IPOL': 0x01,
                                'GPPU':  0x06, 'GPIO': 0x09}

    def __init__(self, name):
        Port.__init__(self, name)

        self.registers = {}

        mcp_type = name[:2]
        mcp_address = int(name[2])
        i2c_address = self._I2C_BASE_ADDRESS + mcp_address

        # Set the port's MCP230XX IO configuration register, IOCON to 0xA2.
        # This sets bit 7, BANK = 1, bit 5, SEQOP = 1, and bit 1, INTPOL = 1.
        # These settings provide byte-oriented register addressing and disable
        # sequential operation.  See the MCP23017 Data Sheet page 18 for
        # details.

        iocon = self._Register(i2c_address, 0x05)
        if iocon.value != 0xA2:
            iocon = self._Register(i2c_address, 0x0A)
            iocon.write(0xA2)

        # Instantiate registers.

        port_offset = 0x10 if mcp_type == 'gb' else 0
        for reg_name in self._REGISTER_BASE_ADDRESSES:
            reg_address = self._REGISTER_BASE_ADDRESSES[reg_name] + port_offset
            self.registers[reg_name] = self._Register(i2c_address, reg_address)

        self._gpio = self.registers['GPIO']

        # Instantiate channels.

        for channel_number in range(8):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(self, channel_name)
            self._channels.append(channel)

    def _poll(self, dt_now):  # Replaces superclass method.
        try:
            self._gpio.read()
            changes = self._gpio.value ^ self._gpio.prior_value
        except OSError as err:
            self._running = False
            err_msg = ('polling read error on port "%s"; port stopped'
                       % self.name)
            IOMGR.queue_message(ERROR, err_msg)
            IOMGR.queue_message(ERROR, err)
            changes = 0xff
        for channel in self._channels:
            mask = 1 << channel.number
            if self._running:  # gpio register read was good.
                bitval = 1 if self._gpio.value & mask else 0
            else:  # gpio register read error.
                bitval = '!ERROR'
            channel.prior_value = channel.value
            channel.value = bitval
            if channel.change and changes & mask:
                IOMGR.queue_message(DATA_CHG, channel.id, bitval)
            interval = (dt_now - channel.prior_report).total_seconds()
            if channel.interval and interval >= channel.interval:
                IOMGR.queue_message(DATA_INT, channel.id, bitval)
                channel.prior_report = dt_now

    class _Register:
        """
        """
        def __init__(self, i2c_address, reg_address):
            self.prior_value = 0
            self.value = 0

            self._i2c_address = i2c_address
            self._reg_address = reg_address

            self.read()

        def read(self):
            self.prior_value = self.value
            self.value = I2CBUS.read_byte_data(self._i2c_address,
                                               self._reg_address)
            return self.value

        def write(self, value):
            I2CBUS.write_byte_data(self._i2c_address, self._reg_address, value)
            self.prior_value = self.value
            self.value = value

        def update(self, channel_number, bitval):
            mask = 1 << channel_number
            value = self.value | mask if bitval else self.value & ~mask
            self.write(value)

    class _Channel(Channel):
        """
        The MCP230XX._Channel class provides state data and access methods to
        read and write individual MCP23008 or MCP23017 channels.  The following
        public methods are called by the IOMGR.process_request method to
        perform channel operations:

        alias:      designates an alias for the channel name
        change:     specifies continuous change reporting
        interval:   specifies interval reporting
        direction:  sets channel direction (input/output)
        polarity:   sets channel polarity
        pullup:     sets channel pullup configuration
        read:       reads a single bit value
        write:      writes a single bit value
        momentary:  momentarily turns an output channel on and then off

        The results of read, write, and momentary operations are saved in the
        value and prior_value attributes, queued to IOMGR._queue, and logged.
        MCP230XX._Channel methods accomplish their functions using
        MCP230XX._Register methods to read/write lower-level registers.
        """
        def __init__(self, port, name):
            Channel.__init__(self, port, name)

            self.number = int(name[3])

            self._iodir = port.registers['IODIR']
            self._ipol = port.registers['IPOL']
            self._gppu = port.registers['GPPU']
            self._gpio = port.registers['GPIO']

            # Set default channel configuration and read channel value.

            self.direction(DIRECTION)
            self.polarity(POLARITY)
            self.pullup(PULLUP)
            self.read()

        # Public methods called by Port.execute_request(channel, request_id,
        # argument):

        def direction(self, direction):
            self._iodir.update(self.number, direction)
            self._direction = direction

        def momentary(self, delay):
            self._momentary(delay)

        def polarity(self, polarity):
            if self._check_direction(INPUT):
                self._ipol.update(self.number, polarity)

        def pullup(self, pullup):
            if self._check_direction(INPUT):
                if self._check_bitval(pullup):
                    self._gppu.update(self.number, pullup)

        def write(self, bitval):
            if self._check_direction(OUTPUT):
                if self._check_bitval(bitval):
                    self._write(bitval)

        # Semi-private method called only by read() and the port method
        # _poll().

        def read_hw(self):  # Replaces superclass method.
            bitval = 1 if self._gpio.read() & (1 << self.number) else 0
            return bitval

        # Private method:

        def _write_hw(self, bitval):  # Replaces superclass method.
            self._gpio.update(self.number, bitval)


class MCP342X(Port):

    def __init__(self, name):
        Port.__init__(self, name)

        mcp_type = name[:2]
        num_channels = 2 if mcp_type == 'aa' else 4
        for channel_number in range(num_channels):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(self, channel_name)
            self._channels.append(channel)

    def _value_changed(self, channel):  # Replaces superclass method.
        pval = channel.prior_value
        val = channel.value
        if val == '!ERROR':
            return True
        changed = False
        if pval:
            change = 100.0 * fabs(val - pval) / pval
            changed = change > channel.change
        return changed

    class _Channel(Channel):
        """
        """
        _I2C_BASE_ADDRESS = 0x68

        def __init__(self, port, name):
            Channel.__init__(self, port, name)

            self.scaling = SCALING

            self._i2c_address = self._I2C_BASE_ADDRESS + int(name[2])
            self._number = int(name[3])
            self._resolution = RESOLUTION
            self._gain = GAIN
            self._config = None
            self._num_bytes = None

            # Set default channel configuration and read channel value.

            self._configure()
            self.read()

        # Public methods called by Port.execute_request(channel, request_id,
        # argument):

        def gain(self, gain):
            self._gain = gain
            self._configure()

        def resolution(self, resolution):
            self._resolution = resolution
            self._configure()

        # Semi-private method called only by read() and the port method
        # _poll().

        def read_hw(self):  # Replaces superclass method.
            I2CBUS.write_byte(self._i2c_address, self._config)
            config = self._config & 0x7F
            while True:
                byts = I2CBUS.read_i2c_block_data(self._i2c_address, config,
                                                  self._num_bytes)
                if byts[-1] < 128:
                    break
            counts = int.from_bytes(byts[:-1], byteorder='big',
                                    signed=True)
            if counts < 0:
                counts = 0
            lsb_value = 4.096 / 2 ** self._resolution
            return counts * lsb_value / self._gain * self.scaling

        # Private method:

        def _configure(self):
            resolution_index = int((self._resolution - 12) / 2)
            gain_index = int(log2(self._gain))
            self._config = (0x80 + 32 * self._number + 4 * resolution_index
                            + gain_index)
            self._num_bytes = 3 if self._resolution < 18 else 4


class MCP320X:
    pass


class MCP482X:
    pass


class IOMGR:
    """
    """
    _PORTS = {'aa': (4, MCP342X),  'ab': (4, MCP342X),
              'ad': (8, MCP320X),  'ae': (8, MCP320X),
              'da': (8, MCP482X),  'db': (8, MCP482X),
              'ga': (8, MCP230XX), 'gb': (8, MCP230XX),
              'gg': (2, BCM283X),  'gp': (2, BCM283X)}

    _queue = Queue()
    _gpio_cleanup = False
    running = False
    name = None
    args = None
    log = None

    @classmethod
    def init(cls):
        """
        Parse command line arguments and initialize printing/logging for main
        programs using the IOMGR class.
        """
        parser = ArgumentParser()
        parser.add_argument('port_names', nargs='?',
                            help='string of port names to be processed')
        parser.add_argument('-i', '--interactive', action='store_true',
                            help='run in interactive mode from a terminal')
        parser.add_argument('-I', '--IP_port',
                            default=str(DEFAULT_PORT_NUMBER),
                            help='server IP port number')
        parser.add_argument('-l', '--log', action='store_true',
                            help='log data and status to a file in '
                                 '/var/log/piDACS')
        parser.add_argument('-L', '--log_level', default='INFO',
                            choices=['DEBUG', 'DATA_INT', 'DATA_CHG',
                                     'DATA_REQ', 'INFO', 'WARNING', 'ERROR',
                                     'CRITICAL'],
                            help='logging level for optional file logging')
        parser.add_argument('-p', '--print', action='store_true',
                            help='print data and status to sys.stdout')
        parser.add_argument('-P', '--print_level', default='INFO',
                            choices=['DEBUG', 'DATA_INT', 'DATA_CHG',
                                     'DATA_REQ', 'INFO', 'WARNING', 'ERROR',
                                     'CRITICAL'],
                            help='logging level for optional printing')
        cls.name = parser.prog.replace('.py', '')
        cls.args = parser.parse_args()

        addLevelName(DATA_INT, 'Interval Data')
        addLevelName(DATA_CHG, 'Change Data')
        addLevelName(DATA_REQ, 'Request Data')

        cls.log = getLogger(cls.name)
        cls.log.setLevel(DEBUG)

        if cls.args.print:
            print_handler = StreamHandler()
            print_handler.setLevel(cls.args.print_level)
            print_formatter = Formatter('%(message)s')
            print_handler.setFormatter(print_formatter)
            cls.log.addHandler(print_handler)
            argstr = ''
            for arg in argv[1:]:
                argstr = argstr + arg + ' '
            cls.log.info('initializing %s with options %s'
                         % (cls.name, argstr))

        if cls.args.log:
            log_name = cls.name.lower()
            dir_path = Path('/var/local/log/%s' % log_name)
            dir_path.mkdir(exist_ok=True)  # Make directories if none exist.
            file_path = dir_path / Path('%s.log' % log_name)
            try:
                log_handler = TimedRotatingFileHandler(file_path,
                                                       when='midnight')
            except OSError as err:
                err_msg = ('error in opening "%s"; log option ignored'
                           % filename)
                cls.queue_message(ERROR, err_msg)
                cls.queue_message(ERROR, err)
                return
            else:
                log_handler.setLevel(cls.args.log_level)
                log_formatter = Formatter(
                    '%(asctime)s %(levelname)s %(message)s')
                log_handler.setFormatter(log_formatter)
                cls.log.addHandler(log_handler)

    @classmethod
    def start(cls):
        """
        Check for valid port names and instantiate/start valid ports.
        """
        port_names = cls.args.port_names
        if not port_names:
            try:
                pnfile = open('/usr/local/etc/pidacs/port_names', 'r')
                port_names = pnfile.read().rstrip()
                pnfile.close()
            except OSError:
                warning = ('no port name(s) in command line or port_names '
                           'file; defaults used')
                cls.queue_message(WARNING, warning)
                port_names = DEFAULT_PORT_NAMES
        port_names = port_names.replace(',', '').lower()
        cls.queue_message(DEBUG, "starting %s using port(s) '%s'"
                          % (cls.name, port_names))
        port_names = port_names.split()
        for port_name in port_names:
            if len(port_name) == 3 and port_name[2].isdecimal():
                port_type = port_name[:2]
                num = int(port_name[2])
                if port_type in cls._PORTS and num < cls._PORTS[port_type][0]:
                    if port_type == 'gg':
                        alt_port_name = 'gp' + str(num)
                    elif port_type == 'gp':
                        alt_port_name = 'gg' + str(num)
                    else:
                        alt_port_name = None
                    for port in Port.ports:
                        if port.name in (port_name, alt_port_name):
                            break
                    else:
                        cls.queue_message(DEBUG, 'starting port "%s"'
                                          % port_name)
                        try:
                            port = cls._PORTS[port_type][1](port_name)
                            port.start()
                        except OSError as err:
                            err_msg = ('I/O error on port "%s"; startup '
                                       'aborted' % port_name)
                            cls.queue_message(ERROR, err_msg)
                            cls.queue_message(ERROR, '%s' % err)
                        if port_type in ('gg', 'gp'):
                            cls._gpio_cleanup = True
                        continue
            warning = ('invalid or duplicate port name "%s"; port not started'
                       % port_name)
            cls.queue_message(WARNING, warning)
        if Port.ports:
            cls.running = True
        else:
            err = 'no port(s) started; %s terminated' % cls.name
            cls.queue_message(ERROR, err)

    @classmethod
    def stop(cls):
        for port in Port.ports:
            port.stop()
        if cls._gpio_cleanup:
            GPIO.cleanup()

    @classmethod
    def get_message(cls):
        try:
            message = cls._queue.get(timeout=1)
        except Empty:
            message = b''
        return message.decode().strip()

    @classmethod
    def queue_message(cls, level, *args):
        message = '%s %s' % (str(datetime.now()), level)
        for arg in args:
            message += ' %s' % arg
        cls._queue.put(message.ljust(MESSAGE_LENGTH).encode()[:MESSAGE_LENGTH])
        cls.log.log(level, message[DATETIME_LENGTH + 4:])

    @classmethod
    def process_request(cls, request):
        cls.queue_message(DEBUG, request)

        # A channel request has the form: "channel_name request_id argument".
        # Split the request into its three components and check for errors.

        rsplit = request.split()
        nsplit = len(rsplit)

        if nsplit not in (2, 3):
            warning = 'invalid syntax; request ignored'
            cls.queue_message(WARNING, warning)
            return

        channel_name = rsplit[0]
        channel = Channel.channels.get(channel_name)
        if channel is None:
            warning = 'channel "%s" not found; request ignored' % channel_name
            cls.queue_message(WARNING, warning)
            return

        request_id = rsplit[1]
        rid_match = request_id.lower()
        if rid_match == 'd':
            rid_match = 'di'
        if rid_match == 'r':
            rid_match = 'rea'
        for rid in REQUESTS:
            if rid.startswith(rid_match):
                request_id = rid
                break
        else:
            warning = 'invalid request id "%s"; request ignored' % request_id
            cls.queue_message(WARNING, warning)
            return

        argument = None
        arg_match = u'DEFAULT'
        arg_match_isanumber = False
        if nsplit == 3:
            argument = rsplit[2]
            arg_match = argument.lower()
            arg_match_isanumber = arg_match.replace('.', '', 1).isdecimal()
        for arg in REQUESTS[request_id]:
            if arg.startswith(arg_match):
                argument = REQUESTS[request_id][arg]
                break
            elif arg == '*' and arg_match != 'DEFAULT':
                break
            elif arg in ('#', '<=') and arg_match_isanumber:
                argument = (float(argument) if '.' in argument
                            else int(argument))
                if arg == '#':
                    break
                elif argument <= REQUESTS[request_id][arg]:
                    break
        else:
            warning = 'invalid argument "%s"; request ignored' % argument
            cls.queue_message(WARNING, warning)
            return

        # Valid request; queue it to the appropriate port thread for execution.

        channel.port.queue_request(channel, request_id, argument)


# IOMGR main program:

if __name__ == '__main__':
    IOMGR.init()
    IOMGR.start()
    interactive = IOMGR.args.interactive and IOMGR.running
    if interactive:
        NBI.start()
        IOMGR.log.setLevel(DEBUG)
        IOMGR.log.info('Begin Interactive Session')
        IOMGR.log.info('Enter requests: channel_name request_id argument '
                       'or quit')
    try:
        while IOMGR.running:
            if interactive:
                user_request = NBI.get_input()
                if user_request is None:
                    continue
                if user_request and 'quit'.startswith(user_request.lower()):
                    IOMGR.running = False
                    continue
                IOMGR.process_request(user_request)
    except KeyboardInterrupt:
        pass
    IOMGR.stop()
    if interactive:
        IOMGR.log.info('End Interactive Session')
