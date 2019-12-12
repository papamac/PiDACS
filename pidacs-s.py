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
__version__ = '0.9.7'
__date__ = 'December 12, 2019'


from signal import signal, SIGTERM
from socket import *
from threading import Thread

from argsandlogs import AL
from colortext import *
from iomgr import IOMGR
from messagesocket import MessageSocket, SOCKET_TIMEOUT
from nbi import NBI


class Server:
    """
    """
    running = False
    _socket = None
    _accept = None
    _serve = None
    _clients = []

    @classmethod
    def start(cls):
        IOMGR.start()
        if IOMGR.running:

            # Start server operations.

            cls._accept = Thread(name='accept_client_connections',
                                 target=cls._accept_client_connections)
            cls._serve = Thread(name='serve_clients',
                                target=cls._serve_clients)
            cls.running = True
            cls._accept.start()
            cls._serve.start()
            signal(SIGTERM, cls.terminate)

    @classmethod
    def stop(cls):
        if IOMGR.running:
            for client in cls._clients:
                client.stop()
            cls._accept.join()
            cls._serve.join()
            IOMGR.stop()

    @classmethod
    def terminate(cls, *args):
        cls.running = False

    @classmethod
    def _accept_client_connections(cls):
        cls._socket = socket(AF_INET, SOCK_STREAM)
        cls._socket.settimeout(SOCKET_TIMEOUT)
        cls._socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        cls._socket.bind(('', AL.args.port_number))
        cls._socket.listen(5)
        ipv4, port = cls._socket.getsockname()
        name = '%s[%s:%s]' % (gethostname(), ipv4, port)
        AL.log.info(ct(BGREEN, 'accepting client connections "%s"' % name))
        while cls.running:
            try:
                client_socket, client_address_tuple = cls._socket.accept()
            except timeout:
                continue
            client = MessageSocket('', client_socket, IOMGR.process_request)
            client.name = client.recv() + client.name
            AL.log.info(ct(BGREEN, 'connected "%s"' % client.name))
            client.start()
            cls._clients.append(client)

    @classmethod
    def _serve_clients(cls):
        while cls.running:
            message = IOMGR.get_message()
            if message:
                for client in cls._clients:
                    if client.running:
                        client.send(message)


# pidacs-s main program:

AL.parser.add_argument('port_names', nargs='+',
                       help='string of IO port names separated by spaces')
AL.parser.add_argument('-P', '--port_number',
                       help='%s port number' % AL.name)
AL.parser.add_argument('-d', '--daemon', action='store_true',
                       help='daemon server: interactive requests and printing '
                       'disabled; file logging enabled')
AL.start()
Server.start()
if Server.running:
    if not AL.args.daemon:
        NBI.start()
        AL.log.info(ct(BBLUE, '\nstarting %s interactive session'
                              '\nenter requests: [channel_name request_id '
                              'argument] or [quit]' % AL.name))
    try:
        while Server.running:
            if not AL.args.daemon:
                user_request = NBI.get_input()
                if user_request is None:
                    continue
                if user_request and 'quit'.startswith(user_request.lower()):
                    Server.running = False
                    continue
                IOMGR.process_request(user_request)
    except KeyboardInterrupt:
        pass
    Server.stop()
AL.log.info(ct(BBLUE, 'ending %s' % AL.name))
