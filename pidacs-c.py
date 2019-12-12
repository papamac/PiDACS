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


from socket import create_connection, gethostname

from argsandlogs import AL
from colortext import *
from messagesocket import MessageSocket, SOCKET_TIMEOUT
from nbi import NBI


SERVER_TIMEOUT = 300.0


def log_messages(message):
    level = int(message[:2])
    AL.log.log(level, message[2:])


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
    AL.log.info(ct(BRED, 'connection error %s "%s:%s" %s'
                % (oserr.errno, AL.args.server, AL.args.port_number,
                   oserr.strerror)))
else:
    server = MessageSocket(AL.args.server, server_socket, log_messages,
                           SERVER_TIMEOUT)
    AL.log.info(ct(BGREEN, 'connected "%s"' % server.name))
    server.send(gethostname())
    server.start()

    NBI.start()
    AL.log.info(ct(BBLUE, '\nstarting %s interactive session'
                          '\nenter requests: [channel_name request_id '
                          'argument] or [quit]' % AL.name))
    try:
        while server.running:
            user_request = NBI.get_input()
            if user_request is None:
                continue
            if user_request and 'quit'.startswith(user_request.lower()):
                server.running = False
                continue
            server.send(user_request)
    except KeyboardInterrupt:
        server.running = False
    server.stop()
AL.log.info(ct(BBLUE, 'ending %s' % AL.name))
