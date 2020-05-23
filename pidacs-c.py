#!/usr/bin/env python3
"""
 PACKAGE:  Raspberry Pi Data Acquisition and Control System (PiDACS)
  MODULE:  pidacs-c.py
   TITLE:  PiDACS interactive client main program
FUNCTION:  pidacs-c is a remote, interactive client program for the PiDACS
           input/output manager (iomgr).  It connects to a PiDACS server
           (pidacs) over a network to receive iomgr messages/data and send
           user requests to the iomgr.
   USAGE:  pidacs-c is executed from the command line with options specified in
           the argsandlogs module augmented by the code below.  It is
           compatible with Python 2.7.16 and all versions of Python 3.x.
  AUTHOR:  papamac
 VERSION:  1.0.7
    DATE:  May 23, 2020


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
__version__ = '1.0.7'
__date__ = 'May 23, 2020'


from socket import gethostname

from papamaclib.argsandlogs import AL
from papamaclib.colortext import getLogger
from papamaclib.messagesocket import MessageSocket, STATUS_INTERVAL
from papamaclib.nbi import NBI


# Global constant:

LOG = getLogger('Plugin')


# pidacs-c function:

def log_message(reference_name, message):
    message_split = message.split(maxsplit=1)
    level = int(message_split[0])
    LOG.log(level, message_split[1])


# pidacs-c main program:

AL.parser.add_argument('server', nargs='?', default=gethostname(),
                       help='server FQDN or IPv4 address')
AL.parser.add_argument('-P', '--port_number', type=int,
                       choices=range(50000, 60000, 1000), default=50000,
                       help='server port number')
AL.start(__version__)

server = MessageSocket(AL.name, process_message=log_message,
                       recv_timeout=STATUS_INTERVAL + 10.0)
server.connect_to_server(AL.args.server, AL.args.port_number)
if server.connected:
    server.start()
    NBI.start()
    LOG.blue('starting %s interactive session\nenter requests: '
             '[channel_name request_id argument] or [quit]' % AL.name)
    try:
        while server.running:
            user_request = NBI.get_input()
            if user_request:
                if 'quit'.startswith(user_request.lower()):
                    server.running = False
                else:
                    server.send(user_request)
    except KeyboardInterrupt:
        server.running = False
    server.stop()
AL.stop()
