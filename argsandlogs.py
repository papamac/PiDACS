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

"""
__author__ = 'papamac'
__version__ = '1.0.0'
__date__ = 'December 12, 2019'


from argparse import ArgumentParser
from logging import addLevelName, Formatter, getLogger, StreamHandler
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

from colortext import *


# Global constants:

LOG = getLogger('Plugin')

THREAD_DEBUG = 5
DATA = 15

DYNAMIC_PORT_RANGE = range(49152, 65535)  # Range of valid dynamic ports.
DEFAULT_PORT_NUMBER = 50000   # Arbitrary selection from DYNAMIC_PORT_RANGE.


class AL:
    """
    """
    parser = ArgumentParser()
    name = parser.prog.replace('.py', '')
    args = None

    @classmethod
    def start(cls):
        # Parse command line arguments, initialize printing/logging and log
        # main program starting message.

        # printing (-p) defaults to 'DATA' and logging (-l) defaults to None
        # if they are omitted from the command line.  If the daemon option (-d)
        # is set, printing is set to None regardless of its command line
        # setting and logging is set to 'DATA' if it was omitted from the
        # command line.

        cls.parser.add_argument('-p', '--print', choices=['THREAD_DEBUG',
                    'DEBUG', 'DATA', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                    help='optional printing to sys.stdout and printing level',
                    default='DATA')
        cls.parser.add_argument('-l', '--log', choices=['THREAD_DEBUG',
                    'DEBUG', 'DATA', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
                    help='optional file logging and logging level')
        cls.parser.add_argument('-L', '--log_directory',
                    help='top-level log directory (full pathname or relative)',
                    default='/var/local/log')
        cls.args = cls.parser.parse_args()

        addLevelName(THREAD_DEBUG, 'THREAD_DEBUG')
        addLevelName(DATA, 'DATA')
        LOG.setLevel(THREAD_DEBUG)

        if hasattr(cls.args, 'daemon') and cls.args.daemon:
            cls.args.print = None
            if cls.args.log is None:
                cls.args.log = 'DATA'

        if cls.args.print:
            print_handler = StreamHandler()
            print_handler.setLevel(cls.args.print)
            print_formatter = Formatter('%(message)s')
            print_handler.setFormatter(print_formatter)
            LOG.addHandler(print_handler)

        log_name = cls.name.lower()
        if hasattr(cls.args, 'port_number'):
            port_number = DEFAULT_PORT_NUMBER
            if cls.args.port_number:
                port = 0
                if cls.args.port_number.isnumeric():
                    port = int(cls.args.port_number)
                if port in DYNAMIC_PORT_RANGE:
                    port_number = port
                else:
                    warning = ('invalid port number "%s"; default used'
                               % cls.args.port_number)
                    LOG.warning(ct(BYELLOW, warning))
            cls.args.port_number = port_number
            log_name += str(port_number)

        if cls.args.log:
            dir_path = Path(cls.args.log_directory) / Path(cls.name.lower())
            log_path = dir_path / Path(log_name + '.log')
            try:
                dir_path.mkdir(parents=True, exist_ok=True)
                log_handler = TimedRotatingFileHandler(log_path,
                                                       when='midnight')
            except OSError as oserr:
                warning = ('open error %s "%s" %s; log option ignored'
                           % (oserr.errno, log_path, oserr.strerror))
                LOG.warning(ct(BYELLOW, warning))
                cls.args.log = None
            else:
                log_handler.setLevel(cls.args.log)
                log_formatter = Formatter(
                    '%(asctime)s %(levelname)s %(message)s')
                log_handler.setFormatter(log_formatter)
                LOG.addHandler(log_handler)

        args = str(cls.args).split('(')[1][:-1]
        LOG.info(ct(BBLUE, 'starting %s with the following arguments/'
                           'defaults:\n%s' % (cls.name, args)))

    @classmethod
    def stop(cls):

        # Log main program stopping message.

        LOG.info(ct(BBLUE, 'stopping %s' % cls.name))
