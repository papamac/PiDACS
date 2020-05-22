"""
 PACKAGE:  Raspberry Pi Data Acquisition and Control System (PiDACS)
  MODULE:  iomgr.py
   TITLE:  PiDACS input/output manager (iomgr)
FUNCTION:  iomgr provides classes and methods to perform input and output
           operations for a variety of data acquisition and control devices on
           a Raspberry Pi.
   USAGE:  iomgr is imported and used within main programs (e.g., pidacs).
           The module also includes an interactive main program that can be
           executed from the command line for testing purposes.  It is
           compatible with Python 2.7.16 and all versions of Python 3.x.
  AUTHOR:  papamac
 VERSION:  1.1.2
    DATE:  May 22, 2020


MIT LICENSE:

Copyright (c) 2018-2020 David A. Krause, aka papamac

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


DESCRIPTION:

****************************** needs work *************************************

iomgr version 1.0.7 is an initial operational version that supports analog
input and digital input/output.  Documentation is incomplete.

Device characteristics for current and soon to be supported I/O devices are as
follows:

-------------------------------------------------------------------------------
|  Device   |         Description             | Max No  |   Port   | Channels |
|   Name    |                                 | Devices |   Type   | per Port |
|-----------|---------------------------------|---------|----------|----------|
|BCM2835,6,7| Raspberry Pi Baseline GPIO      |    1    |    gp0   |    7     |
|           | GPIOs 17-18, 22-25, 27          |         |          |          |
|-----------|---------------------------------|---------|----------|----------|
|BCM2835,6,7| Raspberry Pi Extended GPIO      |    1    |    gp1   |    9     |
|           | GPIOs 5-6, 12-13, 16, 19-21, 26 |         |          |          |
|-----------|---------------------------------|---------|----------|----------|
| MCP23008  | 8-Bit I/O Expander              |    8    |    ga    |    8     |
|-----------|---------------------------------|---------|----------|----------|
| MCP23017  | 16-Bit I/O Expander (A Port)    |    8    |    ga    |    8     |
|-----------|---------------------------------|---------|----------|----------|
| MCP23017  | 16-Bit I/O Expander (B Port)    |    8    |    gb    |    8     |
|-----------|---------------------------------|---------|----------|----------|
| MCP3204*  | 4-Channel 12-Bit A/D Converter  |    8?   |    ad    |    4     |
|-----------|---------------------------------|---------|----------|----------|
| MCP3208*  | 8-Channel 12-Bit A/D Converter  |    8?   |    ae    |    8     |
|-----------|---------------------------------|---------|----------|----------|
| MCP3422,3 | 2-Channel 18-Bit A/D Converter  |    8    |    aa    |    2     |
|-----------|---------------------------------|---------|----------|----------|
| MCP3424   | 4-Channel 18-Bit A/D Converter  |    8    |    ab    |    4     |
|-----------|---------------------------------|---------|----------|----------|
| MCP4821*  | 1-Channel 12-Bit D/A Converter  |    8?   |    da    |    1     |
|-----------|---------------------------------|---------|----------|----------|
| MCP4822*  | 2-Channel 12-Bit D/A Converter  |    8?   |    db    |    2     |
-------------------------------------------------------------------------------

 * Implementation in progress.


DEPENDENCIES/LIMITATIONS:

****************************** needs work *************************************

"""

__author__ = 'papamac'
__version__ = '1.1.2'
__date__ = 'May 22, 2020'

from datetime import datetime
from logging import DEBUG, INFO, WARNING, ERROR
from math import fabs, log2
from queue import Queue, Empty
from threading import Thread
from time import sleep

from i2cbus import i2cbus
import RPi.GPIO as gpio

from papamaclib.argsandlogs import AL
from papamaclib.colortext import getLogger, DATA
from papamaclib.messagesocket import STATUS_INTERVAL
from papamaclib.nbi import NBI

###############################################################################
#
# Global constants
#
###############################################################################

LOG = getLogger('Plugin')   # Color logger.

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

# Options and DEFAULT value for digital channel pullup resistor configuration.

OFF = 0                     # No internal pullup resistor on digital input.
ON = UP = 1                 # Digital input pulled up to logic high through an
#                             internal resistor.
DOWN = 2                    # Digital input pulled down to logic low through an
#                             internal resistor.  Applicable to BCM GPIO only.
PULLUP = OFF                # DEFAULT is OFF.

# Options and DEFAULT value for digital channel polarity:

NORMAL = 0                  # Normal logic polarity (logic low = 0, high = 1).
INVERTED = 1                # Inverted logic polarity (low = 1, high = 0).
POLARITY = NORMAL           # DEFAULT is NORMAL.

# Options and DEFAULT values for pulse width modulation (PWM):

DUTYCYCLE = 50              # PWM duty cycle in percent (0% - continuous off to
#                             100% - continuous on).  DEFAULT is 50% - half on,
#                             half off.
FREQUENCY = 100             # Pulse frequency in Hz for PWM digital outputs.
#                             DEFAULT is 100 Hz.
START = 1                   # Start PWM output.
STOP = 0                    # Stop PWM output and delete current PWM object.
OPERATION = START           # DEFAULT is START.

# Options and DEFAULT values for analog channel configuration parameters:

RESOLUTION = 12             # ADC resolution is 12, 14, 16, or 18 bits.
#                             DEFAULT is 12 bits.
GAIN = 1                    # ADC gain is 1, 2, 4, or 8.
#                             DEFAULT is 1.
SCALING = 1.0               # The ADC scaling factor can be any real number
#                             providing appropriate scaling for the sensor
#                             circuit being measured by an individual analog
#                             channel.  DEFAULT is 1.0.

# Options and DEFAULT values for change detection and interval reporting:

NONE = FALSE = 0            # No change detection or interval reporting.
YES = TRUE = 1              # Change detection or interval reporting is
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
CHANGE = NONE               # Change detection DEFAULT is NONE.
INTERVAL = NONE             # Interval reporting DEFAULT is NONE.

# DEFAULT operational arguments for channel momentary contact and write
# methods.  These are used in the the IOMGR._REQUESTS dictionary to specify
# default values to be used when a user request omits the argument.

MOMENTARY = 0.2             # The delay time in seconds from a momentary
#                             channel turn-on to its subsequent turn-off.
#                             DEFAULT is 0.2 sec.
WRITE = OFF                 # DEFAULT is OFF.

# Valid request dictionary used in the IOMGR.process_request method.

#           request_id      DEFAULT and optional argument values

REQUESTS = {'alias':       {'*':        None},
            'change':      {'DEFAULT':  CHANGE,      'none':     NONE,
                            'yes':      YES,         'false':    FALSE,
                            'true':     TRUE,        '#':        None},
            'direction':   {'DEFAULT':  DIRECTION,   'input':    INPUT,
                            'output':   OUTPUT,      '1':        1,
                            '0':        0},
            'dutycycle':   {'DEFAULT':  DUTYCYCLE,   '<=':       100},
            'frequency':   {'DEFAULT':  FREQUENCY,   '<=':       1000},
            'gain':        {'DEFAULT':  GAIN,        '1':        1,
                            '2':        2,           '4':        4,
                            '8':        8},
            'interval':    {'DEFAULT':  INTERVAL,    'none':     NONE,
                            'yes':      YES,         'false':    FALSE,
                            'true':     TRUE,        '<=':       86400},
            'momentary':   {'DEFAULT':  MOMENTARY,   '<=':       5},
            'polarity':    {'DEFAULT':  POLARITY,    'normal':   NORMAL,
                            'inverted': INVERTED,    '0':        0,
                            '1': 1},
            'pullup':      {'DEFAULT':  PULLUP,      'off':      OFF,
                            'on':       ON,          'down':     DOWN,
                            'up':       UP,          '0':        0,
                            '1':        1,           '2':        2},
            'pwm':         {'DEFAULT':  OPERATION,   'start':    START,
                            'stop':     STOP,        '1':        1,
                            '0':        0},
            'read':        {'DEFAULT':  None},
            'reset':       {'DEFAULT':  None},
            'resolution':  {'DEFAULT':  RESOLUTION,  '12':       12,
                            '14':       14,          '16':       16,
                            '18':       18},
            'scaling':     {'DEFAULT':  SCALING,     '#':        None},
            'write':       {'DEFAULT':  WRITE,       'off':      OFF,
                            'on':       ON,          'false':    FALSE,
                            'true':     TRUE,        '<=':       5}}

SPECIAL_SHORTCUTS = {'d': 'direction', 'r': 'read', 'res': 'resolution'}


###############################################################################
#
# Base classes and methods: Port, Channel
#
###############################################################################

class Port(Thread):
    """
    **************************** needs work ***********************************
    """

    ports = []

    # Private methods:

    def __init__(self, name):
        LOG.threaddebug('Port.__init__ called "%s"', name)
        Thread.__init__(self, name=name)
        self._channels = []    # All Channel objects for this Port instance.
        self._queue = Queue()  # Request queue for this Port instance.
        self._running = False

    def _poll(self, dt_now):
        # LOG.threaddebug('Port._poll called "%s"', self.name)
        for channel in self._channels:
            if channel.change_ or channel.interval_:
                try:
                    value = channel.read_hw()
                except OSError as err:
                    self._running = False
                    err_msg = ('Port._poll: read error %s on channel "%s" %s; '
                               'port "%s" stopped' % (err.errno, channel.id,
                                                      err.strerror, self.name))
                    IOMGR.queue_message(ERROR, err_msg)
                    IOMGR.queue_message(DATA, '%s !ERROR' % channel.id)
                    value = '!ERROR'
                channel.prior_value = channel.value
                channel.value = value
                if channel.change_ and self._value_changed(channel):
                    IOMGR.queue_message(DATA, channel.id, channel.value)
                interval = (dt_now - channel.prior_report).total_seconds()
                if channel.interval_ and interval >= channel.interval_:
                    IOMGR.queue_message(DATA, channel.id, channel.value)
                    channel.prior_report = dt_now

    def _value_changed(self, channel):
        # LOG.threaddebug('Port._value_changed called "%s"', channel.name)
        return channel.value != channel.prior_value

# Public methods:

    def run(self):
        LOG.threaddebug('Port.run called "%s"', self.name)
        self._running = True
        poll_count = 0
        dt_status = datetime.now()

        # Port execution thread run loop:

        while self._running:

            # Get all currently queued requests and execute them.

            while self._running:
                try:
                    request = self._queue.get(block=False)
                except Empty:
                    break
                channel_name, channel, request_id, method, argument = request
                try:
                    method(argument)
                    LOG.debug('Port.run: executed [%s %s %s]', channel_name,
                              request_id, argument)
                except Exception as err:
                    err_msg = ('Port.run: execution error [%s %s %s] %s'
                               % (channel_name, request_id, argument, err))
                    IOMGR.queue_message(ERROR, err_msg)
                    IOMGR.queue_message(DATA, '%s !ERROR' % channel.id)

            # Perform periodic polling and status functions.

            dt_now = datetime.now()
            self._poll(dt_now)
            poll_count += 1
            status_int = (dt_now - dt_status).total_seconds()
            if status_int >= STATUS_INTERVAL:
                rate = poll_count / status_int
                IOMGR.queue_message(DEBUG, 'Port.run: polling "%s" %d per sec'
                                           % (self.name, rate))
                poll_count = 0
                dt_status = dt_now

    def stop(self):
        LOG.threaddebug('Port.stop called "%s"', self.name)
        if self._running:
            self._running = False
            self.join()

    def queue_request(self, *args):
        LOG.threaddebug('Port.queue_request called "%s"', self.name)
        self._queue.put(args)


class Channel:
    """
    **************************** needs work ***********************************
    """

    channels = {}

    # Private methods:

    def __init__(self, port, name):
        LOG.threaddebug('Channel.__init__ called "%s"', name)
        self.port = port
        self._name = name
        self.id = self.change_ = self.interval_ = None
        self.prior_report = self.prior_value = self.value = None
        self.__init()

    def __init(self):
        LOG.threaddebug('Channel.__init called "%s"', self._name)
        self.id = self._name
        self.change_ = CHANGE
        self.interval_ = INTERVAL
        self.prior_report = datetime.now()
        self.prior_value = None
        self.value = None

    def _reset(self):
        LOG.threaddebug('Channel._reset called "%s"', self.id)
        for channel_name in self.channels:
            if channel_name != self._name:
                if self.channels[channel_name] is self:
                    self.channels[channel_name] = None
        self.__init()

    def _check_bitval(self, bitval):
        LOG.threaddebug('Channel._check_bitval called "%s"', self.id)
        ok = bitval in (0, 1)
        if not ok:
            warning = ('Channel._check_bitval: invalid bit value "%s"; '
                       'request ignored' % bitval)
            IOMGR.queue_message(WARNING, warning)
        return ok

    def _check_direction(self, direction):
        LOG.threaddebug('Channel._check_direction called "%s"', self.id)
        ok = self._direction is direction
        if not ok:
            dir_txt = ('output', 'input')[direction]
            warning = ('Channel._check_direction: channel "%s" not configured '
                       'for %s; request ignored' % (self.id, dir_txt))
            IOMGR.queue_message(WARNING, warning)
        return ok

    def _update(self, value):
        LOG.threaddebug('Channel._update called "%s"', self.id)
        self.prior_value = self.value
        self.value = value
        IOMGR.queue_message(DATA, self.id, value)

    # Public methods:

    def alias(self, alias):
        LOG.threaddebug('Channel.alias called "%s"', self.id)
        self.id = '%s[%s]' % (alias, self._name)
        self.channels[alias] = self

    def change(self, change):
        LOG.threaddebug('Channel.change called "%s"', self.id)
        self.change_ = change

    def interval(self, interval):
        LOG.threaddebug('Channel.interval called "%s"', self.id)
        self.interval_ = interval


###############################################################################
#
# BCM283X classes and methods - support for Raspberry Pi GPIO's
#
###############################################################################

class BCM283X(Port):
    """
    **************************** needs work ***********************************
    """

    _CHANNEL_NAMES = [['gp17', 'gp18', 'gp22', 'gp23', 'gp24', 'gp25', 'gp27'],
                      ['gp05', 'gp06', 'gp12', 'gp13', 'gp16', 'gp19', 'gp20',
                       'gp21', 'gp26']]

    def __init__(self, name):
        LOG.threaddebug('BCM283X.__init__ called "%s"', name)
        Port.__init__(self, name)

        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)

        # Instantiate channels.

        channel_set = int(name[2])  # channel set is 0 or 1.
        for channel_name in self._CHANNEL_NAMES[channel_set]:
            channel = self._Channel(self, channel_name)
            self._channels.append(channel)
            Channel.channels[channel_name] = channel

    class _Channel(Channel):
        """
        ************************** needs work *********************************
        """

        # Private methods:

        def __init__(self, port, name):
            LOG.threaddebug('BCM283X._Channel.__init__ called "%s"', name)
            Channel.__init__(self, port, name)

            self._number = int(name[2:])
            self._direction = self._pullup = self._polarity = None
            self._dutycycle = self._frequency = self._pwm = None
            self._save_change = self._save_interval = None
            self._init()

        def _init(self):
            LOG.threaddebug('BCM283X._Channel._init called "%s"', self.id)
            self._pullup = PULLUP
            self._polarity = POLARITY
            self._dutycycle = DUTYCYCLE
            self._frequency = FREQUENCY
            self._pwm = None
            self._save_change = self.change_
            self._save_interval = self.interval_

            self.direction(DIRECTION)
            self.read()

        def _configure(self, direction, pullup):
            LOG.threaddebug('BCM283X._Channel._configure called "%s"', self.id)
            gppu = (gpio.PUD_OFF if pullup is OFF
                    else gpio.PUD_UP if pullup is UP
                    else gpio.PUD_DOWN)
            gpio.setup(self._number, direction, gppu)
            self._direction = direction
            self._pullup = pullup

        def _apply_polarity(self, bitval):
            # LOG.threaddebug('BCM283X._Channel._apply_polarity called "%s"',
            #                 self.id)
            if self._polarity:
                bitval = 0 if bitval else 1
            return bitval

        def _write_hw(self, bitval):
            LOG.threaddebug('BCM283X._Channel._write_hw called "%s"', self.id)
            gpio.output(self._number, bitval)

        # Public configuration methods: direction, pullup, polarity, reset

        def direction(self, direction):
            LOG.threaddebug('BCM283X._Channel.direction called "%s"', self.id)
            pullup = self._pullup if direction is INPUT else OFF
            self._configure(direction, pullup)

        def pullup(self, pullup):
            LOG.threaddebug('BCM283X._Channel.pullup called "%s"', self.id)
            if self._check_direction(INPUT):
                self._configure(self._direction, pullup)

        def polarity(self, polarity):
            LOG.threaddebug('BCM283X._Channel.polarity called "%s"', self.id)
            if self._check_direction(INPUT):
                self._polarity = polarity

        def reset(self, *args):
            LOG.threaddebug('BCM283X._Channel._init called "%s"', self.id)
            self._reset()
            self.pwm(STOP)
            self._init()

        # Public pwm methods: dutycycle, frequency, pwm

        def dutycycle(self, dutycycle):
            LOG.threaddebug('BCM283X._Channel.dutycycle called "%s"', self.id)
            if self._check_direction(OUTPUT):
                self._dutycycle = dutycycle
                if self._pwm:
                    self._pwm.ChangeDutyCycle(self._dutycycle)

        def frequency(self, frequency):
            LOG.threaddebug('BCM283X._Channel.frequency called "%s"', self.id)
            if self._check_direction(OUTPUT):
                self._frequency = frequency
                if self._pwm:
                    self._pwm.ChangeFrequency(self._frequency)

        def pwm(self, operation):
            LOG.threaddebug('BCM283X._Channel.pwm called "%s"', self.id)
            if self._check_direction(OUTPUT):
                if operation is START:
                    if self._pwm:  # Stop/delete existing pwm, if active.
                        self._pwm.stop()
                        del self._pwm
                    self._save_change = self.change_
                    self._save_interval = self.interval_
                    self.change_ = self.interval_ = NONE
                    self._pwm = gpio.PWM(self._number, self._frequency)
                    self._pwm.start(self._dutycycle)
                else:  # operation is STOP.
                    if self._pwm:  # Stop/delete existing pwm, if active.
                        self._pwm.stop()
                        del self._pwm
                        self._pwm = None
                        self.change_ = self._save_change
                        self.interval_ = self._save_interval

        # Public digital I/O methods: read_hw, read, write, momentary

        def read_hw(self):
            # LOG.threaddebug('BCM283X._Channel.read_hw called "%s"', self.id)
            return self._apply_polarity(gpio.input(self._number))

        def read(self, *args):
            LOG.threaddebug('BCM283X._Channel.read called "%s"', self.id)
            value = self.read_hw()
            self._update(value)

        def write(self, bitval):
            LOG.threaddebug('BCM283X._Channel.write called "%s"', self.id)
            if self._check_direction(OUTPUT):
                if self._check_bitval(bitval):
                    if self._pwm:  # Stop/delete existing pwm, if active.
                        self._pwm.stop()
                        del self._pwm
                        self._pwm = None
                        self.change_ = self._save_change
                        self.interval_ = self._save_interval
                    self._write_hw(bitval)
                    self._update(bitval)

        def momentary(self, delay):
            LOG.threaddebug('BCM283X._Channel.momentary called "%s"', self.id)
            if self._check_direction(OUTPUT):
                self._write_hw(ON)
                self._update(ON)
                sleep(delay)
                self._write_hw(OFF)
                self._update(OFF)


###############################################################################
#
# MCP230XX classes and methods - support for I/O Expander devices
#
###############################################################################

class MCP230XX(Port):
    """
    **************************** needs work ***********************************
    """

    _I2C_BASE_ADDRESS = 0x20

    def __init__(self, name):
        LOG.threaddebug('MCP230XX.__init__ called "%s"', name)
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

        iocon = self._Register('IOCON', i2c_address, 0x05)
        if iocon.value != 0xA2:
            iocon = self._Register('IOCON', i2c_address, 0x0A)
            iocon.write(0xA2)

        # Instantiate registers.

        port_offset = 0x10 if mcp_type == 'gb' else 0
        self.iodir = self._Register(name + ':IODIR', i2c_address,
                                    0x00 + port_offset)
        self.ipol = self._Register(name + ':IPOL', i2c_address,
                                   0x01 + port_offset)
        self.gppu = self._Register(name + ':GPPU', i2c_address,
                                   0x06 + port_offset)
        self.gpio = self._Register(name + ':GPIO', i2c_address,
                                   0x09 + port_offset)

        # Instantiate channels.

        for channel_number in range(8):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(self, channel_name)
            self._channels.append(channel)
            Channel.channels[channel_name] = channel

    def _poll(self, dt_now):  # Replaces superclass method.
        # LOG.threaddebug('MCP230XX._poll called "%s"', self.name)
        for channel in self._channels:
            if channel.change_ or channel.interval_:
                break
            return  # Return if no I/O required.
        good_gpio_read = True
        try:
            self.gpio.read()
            changes = self.gpio.value ^ self.gpio.prior_value
        except OSError as err:
            self._running = False
            good_gpio_read = False
            err_msg = ('MCP230XX._poll: polling read error %s on port '
                       '"%s" %s; port stopped' % (err.errno, self.name,
                                                  err.strerror))
            IOMGR.queue_message(ERROR, err_msg)
            changes = 0xff
        for channel in self._channels:
            mask = 1 << channel.number
            if good_gpio_read:  # gpio register read was good.
                bitval = 1 if self.gpio.value & mask else 0
            else:  # gpio register read error.
                bitval = '!ERROR'
                IOMGR.queue_message(DATA, '%s !ERROR' % channel.id)
            channel.prior_value = channel.value
            channel.value = bitval
            if channel.change_ and changes & mask:
                IOMGR.queue_message(DATA, channel.id, bitval)
            interval = (dt_now - channel.prior_report).total_seconds()
            if channel.interval_ and interval >= channel.interval_:
                IOMGR.queue_message(DATA, channel.id, bitval)
                channel.prior_report = dt_now

    class _Register:
        """
        ************************** needs work *********************************
        """

        def __init__(self, name, i2c_address, reg_address):
            LOG.threaddebug('MCP230XX._Register.__init__ called "%s"', name)
            self.prior_value = 0
            self.value = 0

            self._name = name
            self._i2c_address = i2c_address
            self._reg_address = reg_address

            self.read()

        def read(self):
            LOG.threaddebug('MCP230XX._Register.read called "%s"', self._name)
            self.prior_value = self.value
            self.value = i2cbus.read_byte_data(self._i2c_address,
                                               self._reg_address)
            return self.value

        def write(self, value):
            LOG.threaddebug('MCP230XX._Register.write called "%s"', self._name)
            i2cbus.write_byte_data(self._i2c_address, self._reg_address, value)
            self.prior_value = self.value
            self.value = value

        def update(self, channel_number, bitval):
            LOG.threaddebug('MCP230XX._Register.update called "%s"',
                            self._name)
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
        reset:      reset channel configuration to default attributes
        read:       reads a single bit value
        write:      writes a single bit value
        momentary:  momentarily turns an output channel on and then off

        The results of read, write, and momentary operations are saved in the
        value and prior_value attributes, queued to IOMGR._queue, and logged.
        MCP230XX._Channel methods accomplish their functions using
        MCP230XX._Register methods to read/write lower-level registers.

        ************************** needs work *********************************
        """

        # Private methods:

        def __init__(self, port, name):
            LOG.threaddebug('MCP230XX._Channel.__init__ called "%s"', name)
            Channel.__init__(self, port, name)
            self.number = int(name[3])
            self._port = port
            self._direction = None
            self._init()

        def _init(self):
            LOG.threaddebug('MCP230XX._Channel._init called "%s"', self.id)
            self.direction(DIRECTION)
            self.polarity(POLARITY)
            self.pullup(PULLUP)
            self.read()

        def _write_hw(self, bitval):
            LOG.threaddebug('MCP230XX._Channel._write_hw called "%s"', self.id)
            self._port.gpio.update(self.number, bitval)

        # Public configuration methods: direction, pullup, polarity, reset

        def direction(self, direction):
            LOG.threaddebug('MCP230XX._Channel.direction called "%s"', self.id)
            self._port.iodir.update(self.number, direction)
            self._direction = direction

        def pullup(self, pullup):
            LOG.threaddebug('MCP230XX._Channel.pullup called "%s"', self.id)
            if self._check_direction(INPUT):
                if self._check_bitval(pullup):
                    self._port.gppu.update(self.number, pullup)

        def polarity(self, polarity):
            LOG.threaddebug('MCP230XX._Channel.polarity called "%s"', self.id)
            if self._check_direction(INPUT):
                self._port.ipol.update(self.number, polarity)

        def reset(self, *args):
            LOG.threaddebug('MCP230XX._Channel.reset called "%s"', self.id)
            self._reset()
            self._init()

        # Public digital I/O methods: read_hw, read, write, momentary

        def read_hw(self):
            # LOG.threaddebug('MCP230XX._Channel.read_hw called "%s"', self.id)
            bitval = 1 if self._port.gpio.read() & (1 << self.number) else 0
            return bitval

        def read(self, *args):
            LOG.threaddebug('MCP230XX._Channel.read called "%s"', self.id)
            value = self.read_hw()
            self._update(value)

        def write(self, bitval):
            LOG.threaddebug('MCP230XX._Channel.write called "%s"', self.id)
            if self._check_direction(OUTPUT):
                if self._check_bitval(bitval):
                    self._write_hw(bitval)
                    self._update(bitval)

        def momentary(self, delay):
            LOG.threaddebug('MCP230XX._Channel.momentary called "%s"', self.id)
            if self._check_direction(OUTPUT):
                self._write_hw(ON)
                self._update(ON)
                sleep(delay)
                self._write_hw(OFF)
                self._update(OFF)


###############################################################################
#
# MCP342X classes and methods - support for 18-bit A/D converters
#
###############################################################################

class MCP342X(Port):
    """
    **************************** needs work ***********************************
    """

    def __init__(self, name):
        LOG.threaddebug('MCP342X.__init__ called "%s"', name)
        Port.__init__(self, name)

        mcp_type = name[:2]
        num_channels = 2 if mcp_type == 'aa' else 4
        for channel_number in range(num_channels):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(self, channel_name)
            self._channels.append(channel)
            Channel.channels[channel_name] = channel

    def _value_changed(self, channel):  # Replaces superclass method.
        # LOG.threaddebug('MCP342X._value_changed called "%s"', channel)
        pval = channel.prior_value
        val = channel.value
        if val == '!ERROR':
            return True
        changed = False
        if pval:
            change = 100.0 * fabs(val - pval) / pval
            changed = change > channel.change_
        return changed

    class _Channel(Channel):
        """
        ************************** needs work *********************************
        """

        _I2C_BASE_ADDRESS = 0x68

        # Private methods:

        def __init__(self, port, name):
            LOG.threaddebug('MCP342X._Channel.__init__ called "%s"', name)
            Channel.__init__(self, port, name)
            self._i2c_address = self._I2C_BASE_ADDRESS + int(name[2])
            self._number = int(name[3])
            self._gain = self._resolution = self._scaling = None
            self._config = self._num_bytes = self._multiplier = None
            self._init()

        def _init(self):
            LOG.threaddebug('MCP342X._Channel._init called "%s"', self.id)
            self._gain = GAIN
            self._resolution = RESOLUTION
            self._scaling = SCALING
            self._configure()
            self.read()

        def _configure(self):
            LOG.threaddebug('MCP342X._Channel._configure called "%s"', self.id)
            resolution_index = int((self._resolution - 12) / 2)
            gain_index = int(log2(self._gain))
            self._config = (0x80 + 32 * self._number + 4 * resolution_index
                            + gain_index)
            self._num_bytes = 3 if self._resolution < 18 else 4
            lsb_value = 4.096 / 2 ** self._resolution
            self._multiplier = self._scaling * lsb_value / self._gain

        # Public configuration methods: gain, resolution, scaling, reset

        def gain(self, gain):
            LOG.threaddebug('MCP342X._Channel.gain called "%s"', self.id)
            self._gain = gain
            self._configure()

        def resolution(self, resolution):
            LOG.threaddebug('MCP342X._Channel.resolution called "%s"', self.id)
            self._resolution = resolution
            self._configure()

        def scaling(self, scaling):
            LOG.threaddebug('MCP342X._Channel.scaling called "%s"', self.id)
            self._scaling = scaling
            self._configure()

        def reset(self, *args):
            LOG.threaddebug('MCP342X._Channel.reset called "%s"', self.id)
            self._reset()
            self._init()

        # Public analog input methods: read_hw, read

        def read_hw(self):
            # LOG.threaddebug('MCP342X._Channel.read_hw called "%s"', self.id)
            i2cbus.write_byte(self._i2c_address, self._config)
            config = self._config & 0x7F
            while True:
                bytes_ = i2cbus.read_i2c_block_data(self._i2c_address, config,
                                                    self._num_bytes)
                if bytes_[-1] < 128:
                    break
            counts = int.from_bytes(bytes_[:-1], byteorder='big', signed=True)
            return counts * self._multiplier

        def read(self, *args):
            LOG.threaddebug('MCP342X._Channel.read called "%s"', self.id)
            self._update(self.read_hw())


###############################################################################
#
# MCP320X classes and methods - support for 12-bit A/D converters
#
###############################################################################

class MCP320X(Port):
    """
    **************************** needs work ***********************************
    """

    def __init__(self, name):
        LOG.threaddebug('MCP320X.__init__ called "%s"', name)
        Port.__init__(self, name)


###############################################################################
#
# MCP482X classes and methods - support for 12-bit D/A converters
#
###############################################################################

class MCP482X(Port):
    """
    **************************** needs work ***********************************
    """

    def __init__(self, name):
        LOG.threaddebug('MCP482X.__init__ called "%s"', name)
        Port.__init__(self, name)


###############################################################################
#
# IOMGR class and methods
#
###############################################################################

class IOMGR:
    """
    **************************** needs work ***********************************
    """

    # Class attributes:

    _PORTS = {'aa': (8, MCP342X),  'ab': (8, MCP342X),
              'ad': (8, MCP320X),  'ae': (8, MCP320X),
              'da': (8, MCP482X),  'db': (8, MCP482X),
              'ga': (8, MCP230XX), 'gb': (8, MCP230XX),
              'gp': (2, BCM283X)}
    _queue = Queue()
    _gpio_cleanup = False
    running = False

    @classmethod
    def start(cls):
        """ Start IOMGR ports. """

        LOG.threaddebug('IOMGR.start called')

        # Check port names and instantiate/start valid ports.

        for port_name in AL.args.port_names:
            if len(port_name) == 3 and port_name[2].isdecimal():
                port_type = port_name[:2]
                num = int(port_name[2])
                if port_type in cls._PORTS and num < cls._PORTS[port_type][0]:
                    for port in Port.ports:
                        if port.name == port_name:
                            break
                    else:
                        info = 'starting port "%s"' % port_name
                        cls.queue_message(INFO, info)
                        try:
                            port = cls._PORTS[port_type][1](port_name)
                        except OSError as err:
                            err_msg = ('IOMGR.start: error %s on port '
                                       '"%s" %s; startup aborted'
                                       % (err.errno, port_name, err.strerror))
                            cls.queue_message(ERROR, err_msg)
                            continue
                        except Exception as err:
                            err_msg = ('IOMGR.start: error on port "%s" %s; '
                                       'startup aborted' % (port_name, err))
                            cls.queue_message(ERROR, err_msg)
                            continue
                        Port.ports.append(port)
                        port.start()
                        cls._gpio_cleanup = port_type == 'gp'
                        continue
            err_msg = ('IOMGR.start: invalid or duplicate port name "%s"; '
                       'port not started' % port_name)
            cls.queue_message(ERROR, err_msg)
        if Port.ports:
            cls.running = True
        else:
            err_msg = ('IOMGR.start: no port(s) started; %s terminated'
                       % AL.name)
            cls.queue_message(ERROR, err_msg)

    @classmethod
    def stop(cls):
        LOG.threaddebug('IOMGR.stop called')
        for port in Port.ports:
            port.stop()
        if cls._gpio_cleanup:
            gpio.cleanup()

    @classmethod
    def get_message(cls):
        # LOG.threaddebug('IOMGR.get_message called')
        try:
            message = cls._queue.get(timeout=1)
        except Empty:
            message = ''
        return message

    @classmethod
    def queue_message(cls, level, *args):
        LOG.threaddebug('IOMGR.queue_message called')
        message = ' '.join((str(arg) for arg in args))
        LOG.log(level, message)
        cls._queue.put('%02i %s' % (level, message))

    @classmethod
    def process_request(cls, source, request):
        LOG.threaddebug('IOMGR.process_request called')
        LOG.debug('IOMGR.process_request: received "%s" [%s]', source, request)

        # An IOMGR request 2 or 3 fields with the following form:

        # {channel_name alias} request_id [argument]

        # The first field must be either a hardware channel_name (e.g., gg17)
        # or a previously defined alias.  The second field is one of the keys
        # in the global REQUESTS dictionary or an abbreviation of the key.  The
        # last field is an optional argument (or abbreviation).  If omitted it
        # will be assigned the DEFAULT value from the global REQUESTS
        # dictionary.

        # Split the request into its fields and check for errors.

        rsplit = request.split()
        nsplit = len(rsplit)

        # Check for 2 or 3 fields.  Declare a syntax error and return if other.

        if nsplit not in (2, 3):
            warning = 'IOMGR.process_request: invalid syntax; request ignored'
            cls.queue_message(WARNING, warning)
            return

        # Check for a valid hardware channel_name or alias.  Declare an error
        # and return if none is found.

        channel_name = rsplit[0]
        channel = Channel.channels.get(channel_name)
        if channel is None:
            warning = ('IOMGR.process_request: channel "%s" not found; '
                       'request ignored' % channel_name)
            cls.queue_message(WARNING, warning)
            return

        # Attempt to match the second field with the leading characters of the
        # keys in the REQUESTS dictionary.  If there is a match, check to see
        # if the channel object has a method to process the matched request_id.
        # Declare an error and return if there is no match or no method.

        request_id = rsplit[1]
        rid_match = request_id.lower()
        if rid_match in SPECIAL_SHORTCUTS:
            rid_match = SPECIAL_SHORTCUTS[rid_match]

        for rid in REQUESTS:
            if rid.startswith(rid_match):
                request_id = rid
                method = getattr(channel, request_id, None)
                if method is not None:
                    break
        else:
            warning = ('IOMGR.process_request: invalid request id "%s"; '
                       'request ignored' % request_id)
            cls.queue_message(WARNING, warning)
            return

        # Attempt to match the third field if present, or the string 'DEFAULT'
        # with the leading characters of the argument dictionary keys for the
        # request_id within the REQUESTS dictionary.  Declare an error and
        # return if there is no match.

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
            warning = ('IOMGR.process_request: invalid argument "%s"; '
                       'request ignored' % argument)
            cls.queue_message(WARNING, warning)
            return

        # Valid request; all checks are complete and the variables
        # channel_name, channel, request_id, method, and argument all contain
        # valid data.

        LOG.debug('IOMGR.process_request: validated [%s %s %s]', channel_name,
                  request_id, argument)

        # Execute the request immediately if it does not require physical
        # hardware I/O (e.g., alias, change, interval).  These requests execute
        # quickly and will not delay sequential request processing with I/O
        # waits.  Immediate execution of alias requests also ensures that
        # subsequent requests for an aliased channel are recognized.

        # If the request may incur I/O delays, queue it to the appropriate port
        # thread for concurrent execution.  I/O requests will be executed in
        # sequence on each port, but will not delay requests queued to other
        # ports unless there is I/O bus contention.

        if hasattr(Channel, request_id):  # No I/O is required:
            method(argument)
            LOG.debug('IOMGR.process_request: executed [%s %s %s]',
                      channel_name, request_id, argument)
        else:  # Physical hardware I/O is required.
            channel.port.queue_request(channel_name, channel, request_id,
                                       method, argument)
            LOG.debug('IOMGR.process_request: queued to port [%s %s %s]',
                      channel_name, request_id, argument)


###############################################################################
#
# IOMGR main program
#
###############################################################################

"""
****************************** needs work *************************************
"""

if __name__ == '__main__':
    AL.parser.add_argument('port_names', nargs='+',
                           help='string of IO port names separated by spaces')
    AL.start(__version__)
    IOMGR.start()
    if IOMGR.running:
        NBI.start()
        LOG.blue('starting %s interactive session\nenter requests: '
                 '[channel_name request_id argument] or [quit]' % AL.name)
        try:
            while IOMGR.running:
                user_request = NBI.get_input()
                if user_request:
                    if 'quit'.startswith(user_request.lower()):
                        IOMGR.running = False
                    else:
                        IOMGR.process_request(AL.name, user_request)
        except KeyboardInterrupt:
            pass
        IOMGR.stop()
    AL.stop()
