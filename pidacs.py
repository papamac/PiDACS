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
__date__ = 'August 14, 2019'

from signal import signal, SIGTERM
from threading import Thread

from iomgr import IOMGR
from logging import DEBUG, ERROR, WARNING
from nbi import NBI
from pidacs_global import *


class Client(Thread):
    """
    """
    clients = []

    def __init__(self, sock, socket_id):
        self.socket = sock
        self.socket_id = socket_id
        self.running = False
        Thread.__init__(self, name='Client ' + socket_id)
        self.clients.append(self)

    def start(self):
        self.running = True
        Thread.start(self)

    def stop(self):
        self.running = False
        self.join()
        self.socket.close()

    def run(self):
        while self.running:
            try:
                request = recv_msg(self, self.socket)
                if not self.running:
                    continue
            except OSError as err:
                err_msg = 'recv error "%s"; client stopped' % self.socket_id
                IOMGR.queue_message(ERROR, err_msg)
                IOMGR.queue_message(ERROR, '%s' % err)
                break
            except BrokenPipe:
                warning = 'client disconnected "%s"' % self.socket_id
                IOMGR.queue_message(WARNING, warning)
                break
            dt_recvd = datetime.now()
            requestdt = request[:DATETIME_LENGTH]
            try:
                dt_sent = datetime.strptime(requestdt, '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                warning = ('request has invalid datetime "%s" %s'
                           % (self.socket_id, requestdt))
                IOMGR.queue_message(WARNING, warning)
                continue
            latency = (dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY:
                warning = ('late request "%s"; latency = %3.1f sec'
                           % (self.socket_id, latency))
                IOMGR.queue_message(WARNING, warning)
            IOMGR.process_request(request[DATETIME_LENGTH + 1:])
        else:
            return
        self.running = False


class PiDACS:
    """
    """
    running = False
    _socket = None
    _accept = None
    _serve = None

    @classmethod
    def start(cls):
        IOMGR.init()

        # Open server socket to listen for client connections.

        cls._socket = socket(AF_INET, SOCK_STREAM)
        cls._socket.settimeout(SOCKET_TIMEOUT)
        cls._socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        if IOMGR.args.IP_port_number.isnumeric():
            port_number = int(IOMGR.args.IP_port_number)
        else:
            port_number = 0
        if port_number not in DYNAMIC_PORT_RANGE:
            IOMGR.log.warning('invalid IP port number "%s"; server terminated'
                              % port_number)
            return
        address_tuple = '', port_number
        try:
            cls._socket.bind(address_tuple)
            cls._socket.listen(5)
        except OSError as err:
            IOMGR.log.error('socket error; %s server terminated' % IOMGR.name)
            IOMGR.log.error(err)
            return

        # Start it all up.

        IOMGR.start()
        if IOMGR.running:
            cls._accept = Thread(name='PiDACS_accept_connections',
                                 target=cls._accept_connections)
            cls._serve = Thread(name='PiDACS_serve_clients',
                                target=cls._serve_clients)
            cls.running = True
            cls._accept.start()
            cls._serve.start()
            signal(SIGTERM, cls.terminate)

    @classmethod
    def stop(cls):
        if IOMGR.running:
            for client in Client.clients:
                client.stop()
            cls._accept.join()
            cls._serve.join()
            IOMGR.stop()

    @classmethod
    def terminate(cls, *args):
        cls.running = False

    @classmethod
    def _accept_connections(cls):
        while cls.running:
            try:
                client_socket, client_address_tuple = cls._socket.accept()
            except timeout:
                continue
            client_socket_id = '%s:%i' % client_address_tuple
            status = 'client connected "%s"' % client_socket_id
            IOMGR.queue_message(DEBUG, status)
            client_socket.settimeout(SOCKET_TIMEOUT)
            client = Client(client_socket, client_socket_id)
            client.start()

    @classmethod
    def _serve_clients(cls):
        while cls.running:
            message = IOMGR.get_message()
            if message:
                for client in Client.clients:
                    if client.running:
                        try:
                            send_msg(client.socket, message)
                        except OSError as err:
                            err_msg = ('send error "%s"; client stopped'
                                       % client.socket_id)
                            IOMGR.queue_message(ERROR, err_msg)
                            IOMGR.queue_message(ERROR, '%s' % err)
                        except BrokenPipe:
                            err_msg = ('broken pipe "%s"; client stopped'
                                       % client.socket_id)
                            IOMGR.queue_message(ERROR, err_msg)
                        else:
                            continue
                        client.running = False


# PiDACS main program:

if __name__ == '__main__':
    PiDACS.start()
    interactive = IOMGR.args.interactive and PiDACS.running
    if interactive:
        NBI.start()
        IOMGR.log.info('Begin Interactive Session')
        IOMGR.log.info('Enter requests: channel_name request_id argument '
                       'or quit')
    try:
        while PiDACS.running:
            if interactive:
                user_request = NBI.get_input()
                if user_request is None:
                    continue
                if user_request and 'quit'.startswith(user_request.lower()):
                    PiDACS.running = False
                    continue
                IOMGR.process_request(user_request)
            else:
                continue
    except KeyboardInterrupt:
        PiDACS.running = False
    PiDACS.stop()
    if interactive:
        IOMGR.log.info('End Interactive Session')
