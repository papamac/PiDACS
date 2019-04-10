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
             GPIOs 5-6, 12-13, 16, 19-21, 27
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
__version__ = '0.9.1'
__date__ = 'April 12, 2018'

from argparse import ArgumentParser
from datetime import datetime
from logging import DEBUG, INFO, WARNING, ERROR
from logging import Formatter, getLogger, StreamHandler
from logging.handlers import TimedRotatingFileHandler
from math import fabs, log2
from pathlib import Path
from queue import *
from threading import Thread, Lock
from time import sleep

from i2cbus import I2CBUS
from pidacs_global import MESSAGE_LENGTH, DEFAULT_PORT_NUMBER
import RPi.GPIO as GPIO


# Module iomgr global constants:

STATUS_INTERVAL = 600       # Interval for port status reporting (sec).

# Channel configuration constants are used to specify the operational
# characteristics of specific analog and digital channels.  The optional values
# are used in the IOMGR._REQUESTS dictionary to define the valid argument
# values for a user configuration request.  The DEFAULT values are used to set
# default channel attributes when the channels are instantiated. They are also
# used in the IOMGR._REQUESTS dictionary to specify default values to be used
# when a user request omits the argument.

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
CHANGE = 2                  # Change detection DEFAULT is enabled at 2%.
INTERVAL = NONE             # Interval reporting DEFAULT is NONE.

# Options and DEFAULT value for digital channel direction:

INPUT = 1                   # Channel is a digital input.
OUTPUT = 0                  # Channel is a digital output.
DIRECTION = INPUT           # DEFAULT is INPUT

# Options and DEFAULT value for digital channel polarity:

NORMAL = 0                  # Normal logic polarity (logic low = 0, high = 1).
INVERTED = 1                # Inverted logic polarity (low = 1, high = 0).
POLARITY = NORMAL           # DEFAULT is NORMAL.

# Options and DEFAULT value for digital channel pullup resistor configuration.

DISABLED = 0                # No internal pullup resistor on digital input.
OFF = DISABLED              # Same as DISABLED.
ENABLED = 1                 # Digital input pulled up to logic high through an
#                             internal resistor.
ON = ENABLED                # Same as ENABLED.
UP = ENABLED                # Same as ENABLED.
DOWN = 2                    # Digital input pulled down to logic low through an
#                             internal resistor.  Applicable to BCM GPIO only.
PULLUP = DISABLED           # DEFAULT is DISABLED.

# Options and DEFAULT values for analog channel resolution, gain and scaling
# factor:

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

MOMENTARY = 0.25            # The delay time in seconds from a momentary
#                             channel turn-on to its subsequent turn-off.
#                             DEFAULT is 0,25 sec.
WRITE = OFF                 # DEFAULT is OFF.

# Valid request dictionary used in the IOMGR.process_request method.

#           request_id      DEFAULT and optional argument values

REQUESTS = {'alias':       {'*': None},
            'change':      {'DEFAULT': CHANGE, 'none': NONE, 'yes': YES,
                            'false': FALSE, 'true': TRUE, '#': None},
            'direction':   {'DEFAULT': DIRECTION, 'input': INPUT,
                            'output': OUTPUT, '1': 1, '0': 0},
            'gain':        {'DEFAULT': GAIN, '1': 1, '2': 2, '4': 4, '8': 8},
            'interval':    {'DEFAULT': INTERVAL, 'none': NONE, 'yes': YES,
                            'false': FALSE, 'true': TRUE, '#': None},
            'momentary':   {'DEFAULT': MOMENTARY, '#': None},
            'polarity':    {'DEFAULT': POLARITY, 'normal': NORMAL,
                            'inverted': INVERTED, '0': 0, '1': 1},
            'pullup':      {'DEFAULT': PULLUP, 'disabled': DISABLED,
                            'enabled': ENABLED, 'off': OFF, 'on': ON,
                            'down': DOWN, 'up': UP, '0': 0, '1': 1, '2': 2},
            'read':        None,
            'resolution':  {'DEFAULT': RESOLUTION, '12': 12, '14': 14,
                            '16': 16, '18': 18},
            'scaling':     {'DEFAULT': SCALING, '#': None},
            'write':       {'DEFAULT': WRITE, 'off': OFF, 'on': ON,
                            'false': FALSE, 'true': TRUE, '#': None}}


class Port(Thread):
    """
    """
    def __init__(self, name):
        self._channels = []
        self._running = False
        Thread.__init__(self, name=name, target=self._polling_and_status)

    def _polling_and_status(self):
        poll_count = 0
        status_dt = datetime.now()
        while self._running:
            now = datetime.now()
            self._poll(now)
            poll_count += 1
            status_int = (now - status_dt).total_seconds()
            if status_int >= STATUS_INTERVAL:
                rate = poll_count / status_int
                IOMGR.queue_message('status', '%-4s port polling rate = %d '
                                    'per sec' % (self.name, rate))
                poll_count = 0
                status_dt = now

    def _poll(self, now):
        for channel in self._channels:
            with channel.lock:
                if channel._change or channel._interval:
                    channel.read_nolock()
                    if channel._change and self._value_changed(channel):
                        IOMGR.queue_message('change', channel.name,
                                            channel.value)
                    interval = (now - channel.prior_report).total_seconds()
                    if channel._interval and interval >= channel._interval:
                        IOMGR.queue_message('value', channel.name,
                                            channel.value)
                        channel.prior_report = now

    def _value_changed(self, channel):
        return channel.value != channel.prior_value

    def start(self):
        self._running = True
        Thread.start(self)

    def stop(self):
        self._running = False
        self.join()


class BCM283X(Port):
    """
    """
    _CHANNELS = [{17:11, 18:12, 22:15, 23:16, 24:18, 25:22, 27:13},
                 { 5:29,  6:31, 12:32, 13:33, 16:36, 19:35, 20:38, 21:40,
                  26:37}]

    def __init__(self, name):
        Port.__init__(self, name)
        bcm_type = name[:2]  # bcm_type is gg or gp.
        mode = GPIO.BCM if bcm_type == 'gg' else GPIO.BOARD
        GPIO.setmode(mode)
        GPIO.setwarnings(False)

        # Instantiate channels and add them to the IOMGR channels dictionary
        # using both gpio number names and pin number names.

        channel_set = int(name[2])  # channel set is 0 or 1.
        for gpio_number in self._CHANNELS[channel_set]:
            pin_number = self._CHANNELS[channel_set][gpio_number]
            gpio_name = 'gg%02i' % gpio_number
            pin_name = 'gp%02i' % pin_number
            channel_name = gpio_name if bcm_type == 'gg' else pin_name
            channel = self._Channel(channel_name)
            self._channels.append(channel)
            IOMGR.channels[gpio_name] = channel
            IOMGR.channels[pin_name] = channel

    class _Channel:
        """
        """
        def __init__(self, name):
            self._number = int(name[2:])
            self._change = CHANGE
            self._interval = INTERVAL
            self._direction = DIRECTION
            self._polarity = POLARITY
            self._pullup = GPIO.PUD_OFF + PULLUP

            self.name = name
            self.prior_report = datetime.now()
            self.prior_value = None
            self.value = None
            self.lock = Lock()

            # Set default channel configuration and read channel value.

            self.direction(DIRECTION)
            self.read('')

        def alias(self, alias):
            with self.lock:
                self.name = alias
                IOMGR.channels[self.name] = self

        def change(self, change):
            with self.lock:
                self._change = change

        def interval(self, interval):
            with self.lock:
                self._interval = interval

        def direction(self, direction):
            with self.lock:
                self._direction = direction
                GPIO.setup(self._number, self._direction, self._pullup)

        def polarity(self, polarity):
            with self.lock:
                self._polarity = polarity

        def pullup(self, pullup):
            with self.lock:
                if self._direction:
                    self._pullup = GPIO.PUD_OFF + pullup
                    GPIO.setup(self._number, self._direction, self._pullup)
                    return
                warning = ('channel "%s" not configured for input; request '
                           'ignored' % self.name)
                IOMGR.queue_message('warning', warning)

        def _apply_polarity(self, bitval):
            if self._polarity:
                bitval = 0 if bitval else 1
            return bitval

        def read_nolock(self):
            bitval = self._apply_polarity(GPIO.input(self._number))
            self.prior_value = self.value
            self.value = bitval

        def read(self, argument):
            with self.lock:
                self.read_nolock()
                IOMGR.queue_message('value', self.name, self.value)

        def _write(self, bitval):
            GPIO.output(self._number, self._apply_polarity(bitval))
            self.prior_value = self.value
            self.value = bitval
            IOMGR.queue_message('value', self.name, self.value)

        def write(self, bitval):
            if IOMGR.check_bitval(bitval):
                with self.lock:
                    if not self._direction:  # Channel is an output.
                        self._write(bitval)
                    else:
                        warning = ('channel "%s" not configured for output; '
                                   'request ignored' % self.name)
                        IOMGR.queue_message('warning', warning)

        def momentary(self, delay):
            with self.lock:
                if not self._direction:  # Channel is an output.
                    self._write(ON)
                    sleep(delay)
                    self._write(OFF)
                else:
                    warning = ('channel "%s" not configured for output; '
                               'request ignored' % self.name)
                    IOMGR.queue_message('warning', warning)

class MCP230XX(Port):
    """
    """
    _I2C_BASE_ADDRESS = 0x20
    _REGISTER_BASE_ADDRESSES = {'IODIR': 0x00, 'IPOL': 0x01,
                                'GPPU':  0x06, 'GPIO': 0x09}

    def __init__(self, name):
        Port.__init__(self, name)
        self.lock = Lock()
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
        registers = {}
        for reg_name in self._REGISTER_BASE_ADDRESSES:
            reg_address = self._REGISTER_BASE_ADDRESSES[reg_name] + port_offset
            registers[reg_name] = self._Register(i2c_address, reg_address)

        self._gpio = registers['GPIO']

        # Instantiate channels.

        for channel_number in range(8):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(channel_name, registers, self.lock)
            self._channels.append(channel)
            IOMGR.channels[channel.name] = channel

    def _poll(self, now):
        with self.lock:
            self._gpio.read()
            changes = self._gpio.value ^ self._gpio.prior_value
            for channel in self._channels:
                mask = 1 << channel.number
                bitval = 1 if self._gpio.value & mask else 0
                channel.prior_value = channel.value
                channel.value = bitval
                if channel._change and changes & mask:
                    IOMGR.queue_message('change', channel.name, bitval)
                interval = (now - channel.prior_report).total_seconds()
                if channel._interval and interval >= channel._interval:
                    IOMGR.queue_message('value', channel.name, bitval)
                    channel.prior_report = now

    class _Register:
        """
        """
        def __init__(self, i2c_address, reg_address):
            self._i2c_address = i2c_address
            self._reg_address = reg_address
            self.prior_value = 0
            self.value = 0
            self.read()

        def read(self):
            self.prior_value = self.value
            self.value = I2CBUS.read_byte_data(self._i2c_address,
                                               self._reg_address)

        def write(self, value):
            self.prior_value = self.value
            I2CBUS.write_byte_data(self._i2c_address, self._reg_address, value)
            self.value = value

        def update(self, channel_number, bitval):
            mask = 1 << channel_number
            value = self.value | mask if bitval else self.value & ~mask
            self.write(value)

    class _Channel:
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
        State data are thread-protected using the MCP230XX port lock.
        """
        def __init__(self, name, registers, lock):
            self._iodir = registers['IODIR']
            self._ipol = registers['IPOL']
            self._gppu = registers['GPPU']
            self._gpio = registers['GPIO']
            self._lock = lock
            self._change = CHANGE
            self._interval = INTERVAL

            self.name = name
            self.number = int(name[3])
            self.prior_report = datetime.now()
            self.prior_value = None
            self.value = None

            # Set default channel configuration and read channel value.

            self.direction(DIRECTION)
            self.polarity(POLARITY)
            self.pullup(PULLUP)
            self.read('')

        def _write(self, bitval):
            self._gpio.update(self.number, bitval)
            self.prior_value = self.value
            self.value = bitval
            IOMGR.queue_message('value', self.name, self.value)

        def alias(self, alias):
            with self._lock:
                self.name = alias
                IOMGR.channels[self.name] = self

        def change(self, change):
            with self._lock:
                self._change = change

        def interval(self, interval):
            with self._lock:
                self._interval = interval

        def direction(self, bitval):
            with self._lock:
                self._iodir.update(self.number, bitval)

        def polarity(self, bitval):
            with self._lock:
                self._ipol.update(self.number, bitval)

        def pullup(self, bitval):
            if IOMGR.check_bitval(bitval):
                with self._lock:
                    self._gppu.update(self.number, bitval)

        def read(self, argument):
            with self._lock:
                self._gpio.read()
                bitval = 1 if self._gpio.value & (1 << self.number) else 0
                self.prior_value = self.value
                self.value = bitval
                IOMGR.queue_message('value', self.name, self.value)

        def write(self, bitval):
            if IOMGR.check_bitval(bitval):
                with self._lock:
                    if not(self._iodir.value & (1 << self.number)):  # Output.
                        self._write(bitval)
                    else:
                        warning = ('channel "%s" not configured for output; '
                                   'request ignored' % self.name)
                        IOMGR.queue_message('warning', warning)

        def momentary(self, delay):
            with self._lock:
                if not(self._iodir.value & (1 << self.number)):  # Output.
                    self._write(ON)
                    sleep(delay)
                    self._write(OFF)
                else:
                    warning = ('channel "%s" not configured for output; '
                               'request ignored' % self.name)
                    IOMGR.queue_message('warning', warning)


class MCP342X(Port):

    def __init__(self, name):
        Port.__init__(self, name)
        self.lock = Lock()
        mcp_type = name[:2]
        num_channels = 2 if mcp_type == 'aa' else 4
        for channel_number in range(num_channels):
            channel_name = '%s%i' % (name, channel_number)
            channel = self._Channel(channel_name, self.lock)
            self._channels.append(channel)
            IOMGR.channels[channel_name] = channel

    def _value_changed(self, channel):
        pval = channel.prior_value
        val = channel.value
        changed = False
        if pval:
            change = 100.0 * fabs(val - pval) / pval
            changed = change > channel._change
        return changed

    class _Channel:
        """
        """
        _I2C_BASE_ADDRESS = 0x68

        def __init__(self, name, lock):
            self._i2c_address = self._I2C_BASE_ADDRESS + int(name[2])
            self._number = int(name[3])
            self._change = CHANGE
            self._interval = INTERVAL
            self._scaling = SCALING
            self._resolution = RESOLUTION
            self._gain = GAIN
            self._config = None
            self._num_bytes = None

            self.name = name
            self.prior_report = datetime.now()
            self.prior_value = None
            self.value = None
            self.lock = lock

            # Set default channel configuration and read channel value.

            self._configure()
            self.read('')

        def alias(self, alias):
            with self.lock:
                self.name = alias
                IOMGR.channels[self.name] = self

        def change(self, change):
            with self.lock:
                self._change = change

        def interval(self, interval):
            with self.lock:
                self._interval = interval

        def scaling(self, scaling):
            with self.lock:
                self._scaling = scaling

        def _configure(self):
            resolution_index = int((self._resolution - 12) / 2)
            gain_index = int(log2(self._gain))
            self._config = (0x80 + 32 * self._number + 4 * resolution_index
                            + gain_index)
            self._num_bytes = 3 if self._resolution < 18 else 4

        def resolution(self, resolution):
            with self.lock:
                self._resolution = resolution
                self._configure()

        def gain(self, gain):
            with self.lock:
                self._gain = gain
                self._configure()

        def read_nolock(self):
            I2CBUS.write_byte(self._i2c_address, self._config)
            config = self._config & 0x7F
            while True:
                byts = I2CBUS.read_i2c_block_data(self._i2c_address, config,
                                                  self._num_bytes)
                if byts[-1] < 128:
                    break
            counts = int.from_bytes(byts[:-1], byteorder='big', signed=True)
            if counts < 0:
                counts = 0
            lsb_value = 4.096 / 2 ** self._resolution
            self.prior_value = self.value
            self.value = counts * lsb_value / self._gain * self._scaling

        def read(self, argument):
            with self.lock:
                self.read_nolock()
                IOMGR.queue_message('value', self.name, self.value)


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

    _ports = []
    _queue = Queue()
    _gpio_cleanup = False

    channels = {}

    @classmethod
    def start(cls):
        """
        Check for valid port names and instantiate/start valid ports.
        """
        if not ARGS.port_names:
            error = 'no port port_name(s) specified; %s aborted' % NAME
            cls.queue_message('error', error)
            return
        cls.queue_message('status', 'initializing ports %s' % ARGS.port_names)
        port_names = ARGS.port_names.lower().split()
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
                    for port in cls._ports:
                        if port.name in (port_name, alt_port_name):
                            break
                    else:
                        port = cls._PORTS[port_type][1](port_name)
                        cls._ports.append(port)
                        port.start()
                        if port_type in ('gg', 'gp'):
                            cls._gpio_cleanup = True
                        continue
            warning = ('invalid or duplicate port name "%s"; '
                       'port not started' % port_name)
            cls.queue_message('warning', warning)

    @classmethod
    def stop(cls):
        for port in cls._ports:
            port.stop()
        if cls._gpio_cleanup:
            GPIO.cleanup()

    @classmethod
    def check_bitval(cls, bitval):
        if bitval in (0, 1):
            return True
        warning = 'invalid bit value argument "%s"; request ignored' % bitval
        cls.queue_message('warning', warning)
        return False

    @staticmethod
    def init():
        """
        Parse command line arguments and initialize printing/logging for main
        programs using the IOMGR class.
        """
        parser = ArgumentParser()
        parser.add_argument('port_names', nargs='?', default='gg0 gg1',
                            help='string of port names to be processed')
        parser.add_argument('-i', '--interactive', action='store_true',
                            help='run in interactive mode from a terminal')
        parser.add_argument('-I', '--IP_port', default=DEFAULT_PORT_NUMBER,
                            help='server IP port number')
        parser.add_argument('-l', '--log', action='store_true',
                            help='log data and status to a file in '
                                 '/var/log/piDACS')
        parser.add_argument('-L', '--log_level', default='INFO',
                            choices=['DEBUG', 'INFO', 'WARNING', 'ERROR',
                                     'CRITICAL'],
                            help='logging level for optional file logging')
        parser.add_argument('-p', '--print', action='store_true',
                            help='print data and status to sys.stdout')
        parser.add_argument('-P', '--print_level', default='INFO',
                            choices=['DEBUG', 'INFO', 'WARNING', 'ERROR',
                                     'CRITICAL'],
                            help='logging level for optional printing')
        name = parser.prog.replace('.py', '')
        args = parser.parse_args()

        log = getLogger(name)
        log.setLevel(DEBUG)

        if args.print:
            print_handler = StreamHandler()
            print_handler.setLevel(args.print_level)
            print_formatter = Formatter('%(name)s %(levelname)s %(message)s')
            print_handler.setFormatter(print_formatter)
            log.addHandler(print_handler)

        if args.log:
            log_name = name.lower()
            path = Path('/var/log/%s' % log_name)
            if path.exists():
                path = path / Path('%s.log' % log_name)
                log_handler = TimedRotatingFileHandler(str(path),
                                                       when='midnight')
                log_handler.setLevel(args.log_level)
                log_formatter = Formatter(
                    '%(asctime)s %(name)s %(levelname)s %(message)s')
                log_handler.setFormatter(log_formatter)
                log.addHandler(log_handler)
            else:
                log.warning('directory %s not found; logging option ignored'
                            % path)

        return name, args, log

    @classmethod
    def get_message(cls):
        try:
            message = cls._queue.get(timeout=1)
        except Empty:
            message = b''
        return message.decode().strip()

    @classmethod
    def queue_message(cls, message_id, *args):
        data = '%s %7s:' % (str(datetime.now()), message_id)
        for arg in args:
            data = data + ' ' + str(arg)
        cls._queue.put(data.ljust(MESSAGE_LENGTH).encode()[:MESSAGE_LENGTH])
        level = (eval(message_id.upper())
                 if message_id in ('debug', 'warning', 'error') else INFO)
        LOG.log(level, data[27:])

    @classmethod
    def process_request(cls, request):
        cls.queue_message('request', request)

        # A channel request has the form: "channel_name request_id argument".
        # Split the request into its three components and check for errors.

        request_split = request.lower().split(None, maxsplit=2)
        nsplit = len(request_split)

        if nsplit < 2:
            warning = 'invalid syntax; request ignored'
            cls.queue_message('warning', warning)
            return

        channel_name = request_split[0]
        if channel_name in cls.channels:
            channel = cls.channels[channel_name]
        else:
            warning = 'channel "%s" not found; request ignored' % channel_name
            cls.queue_message('warning', warning)
            return

        request_id = request_split[1]
        if request_id == 'r':
            request_id = 'rea'
        for req_id in REQUESTS:
            if req_id.startswith(request_id):
                request_id = req_id
                break
        else:
            warning = 'invalid request id "%s"; request ignored' % request_id
            cls.queue_message('warning', warning)
            return

        argument = request_split[2] if nsplit == 3 else 'DEFAULT'
        if REQUESTS[request_id]:
            for arg in REQUESTS[request_id]:
                if arg.startswith(argument):
                    argument = REQUESTS[request_id][arg]
                    break
                elif arg == '*' and argument != 'DEFAULT':
                    break
                elif arg == '#'and argument.replace('.', '', 1).isdecimal():
                    if argument.isdecimal():
                        argument = int(argument)
                    else:
                        argument = float(argument)
                    break
            else:
                warning = 'invalid argument "%s"; request ignored' % argument
                cls.queue_message('warning', warning)
                return

        # Valid request; invoke channel methods to process the request.

        debug = 'equivalent method call is %s.%s(%s)'\
                % (channel.name, request_id, argument)
        LOG.debug(debug)
        method = getattr(channel, request_id, None)
        if hasattr(method, '__call__'):
            method(argument)
        else:
            warning = ('request id "%s" is not valid for channel %s; '
                       'request ignored' % (request_id, channel.name))
            cls.queue_message('warning', warning)

# Define the global variables NAME, ARGS, and LOG when the iomgr module is
# loaded.  This enables the global variables to be used both in iomgr.py and
# in other modules.  Use in other modules requires the following imoprt
# statement in the using module:

# from iomgr import NAME, ARGS, LOG

NAME, ARGS, LOG = IOMGR.init()

# IOMGR main program:

if __name__ == '__main__':
    IOMGR.start()
    if ARGS.interactive:
        LOG.info('Begin interactive session')
        LOG.info('Enter requests: channel_name request_id argument or quit')
    while True:
        try:
            if ARGS.interactive:
                req = input()
                if req and 'quit'.startswith(req.lower()):
                    break
                IOMGR.process_request(req)
        except KeyboardInterrupt:
            break
    IOMGR.stop()
    if ARGS.interactive:
        LOG.info('End interactive session')