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

"""
__author__ = 'papamac'
__version__ = '1.0.0'
__date__ = 'December 12, 2019'


from logging import getLogger
from socket import create_connection, gethostname

from argsandlogs import AL
from colortext import *
from messagesocket import MessageSocket, SOCKET_TIMEOUT
from nbi import NBI


# Globals:

LOG = getLogger('Plugin.pidacs-c')

STATUS_INTERVAL = 600.0                     # iomgr status reporting interval.
SERVER_TIMEOUT = STATUS_INTERVAL + 10.0     # Timeout must be longer than the
#                                             status reporting interval to
#                                             avoid timeouts from an idle
#                                             server.


def log_message(message):
    message_split = message.split(maxsplit=1)
    level = int(message_split[0])
    LOG.log(level, message_split[1])


# pidacs-c main program:

AL.parser.add_argument('server', nargs='?', default=gethostname(),
                       help='FQDN or IPv4 address of the PiDACS server')
AL.parser.add_argument('-P', '--port_number',
                       help='%s port number' % AL.name)
AL.start()
address_tuple = AL.args.server, AL.args.port_number
try:
    server_socket = create_connection(address_tuple, SOCKET_TIMEOUT)
except OSError as oserr:
    LOG.error(ct(BRED, 'connection error %s "%s:%s" %s'
                 % (oserr.errno, AL.args.server, AL.args.port_number,
                    oserr.strerror)))
else:
    server = MessageSocket(AL.args.server, server_socket,
                           process_message=log_message,
                           recv_timeout=SERVER_TIMEOUT)
    LOG.info(ct(BGREEN, 'connected "%s"' % server.name))
    server.send(gethostname())
    server.start()

    NBI.start()
    LOG.info(ct(BBLUE, 'starting %s interactive session'
                       '\nenter requests: [channel_name request_id '
                       'argument] or [quit]' % AL.name))
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
