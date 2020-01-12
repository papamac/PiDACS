#!/usr/bin/python3
"""
 PACKAGE:  Raspberry Pi Data Acquisition and Control System (PiDACS)
  MODULE:  pidacs-s.py
   TITLE:  PiDACS server main program (pidacs-s)
FUNCTION:  pidacs-s is a versatile server program for the PiDACS input/output
           manager (iomgr).  It serves data from the iomgr to remote PiDACS
           clients (e.g., pidacs-c or the indigo PiDACS-Bridge plugin) and
           accepts requests from the clients and passes them to the iomgr.  It
           can also accept iomgr requests interactively from the command line
           and display iomgr messages/data to the user.
   USAGE:  pidacs-s is executed from the command line with options specified in
           the argsandlogs module augmented by the code below.  It can also
           be executed as a daemon by using the pidacsd shell script.  It is
           compatible with Python 2.7.16 and all versions of Python 3.x.
  AUTHOR:  papamac
 VERSION:  1.0.2
    DATE:  January 3, 2020


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

DEPENDENCIES/LIMITATIONS:

****************************** needs work *************************************

"""
__author__ = 'papamac'
__version__ = '1.0.2'
__date__ = 'January 3, 2020'


from signal import signal, SIGTERM

from iomgr import IOMGR
from papamaclib.argsandlogs import AL
from papamaclib.colortext import getLogger
from papamaclib.messagesocket import MessageServer
from papamaclib.nbi import NBI


# Global constant:

LOG = getLogger('Plugin')


# pidacs-s function:

def terminate(*args):
    server.running = False


# pidacs-s main program:

AL.parser.add_argument('port_names', nargs='+',
                       help='string of IO port names separated by spaces')
AL.parser.add_argument('-P', '--port_number',
                       help='%s port number' % AL.name)
AL.parser.add_argument('-d', '--daemon', action='store_true',
                       help='daemon server: interactive requests and printing '
                       'disabled; file logging enabled')
AL.start()
IOMGR.start()
if IOMGR.running:
    server = MessageServer(AL.args.port_number,
                           get_message=IOMGR.get_message,
                           process_request=IOMGR.process_request)
    server.start()
    signal(SIGTERM, terminate)

    if not AL.args.daemon:
        NBI.start()
        LOG.blue('starting %s interactive session\nenter requests: '
                 '[channel_name request_id argument] or [quit]' % AL.name)
    try:
        while server.running:
            if not AL.args.daemon:
                user_request = NBI.get_input()
                if user_request:
                    if 'quit'.startswith(user_request.lower()):
                        server.running = False
                    else:
                        IOMGR.process_request('local pidacs-s', user_request)
    except KeyboardInterrupt:
        pass

    server.stop()
    IOMGR.stop()
AL.stop()
