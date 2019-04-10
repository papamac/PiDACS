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
__version__ = '0.9.0'
__date__ = 'April 12, 2018'

from threading import Thread

from iomgr import IOMGR, ARGS, LOG
from pidacs_global import *


class Client(Thread):
    """
    """
    def __init__(self, sock, sock_id):
        self.sock = sock
        self.sock_id = sock_id
        self._running = False
        Thread.__init__(self, name='Client ' + sock_id,
                        target=self._process_client_requests)

    def _process_client_requests(self):
        while self._running:
            req = ''
            try:
                req = recv_msg(self, self.sock)
                if not self._running:
                    break
            except OSError as err_msg:
                error = 'recv error "%s"; client stopped' % self.sock_id
                IOMGR.queue_message('error', error)
                IOMGR.queue_message('error', '%s' % err_msg)
                self._running = False
                break
            except BrokenPipe:
                status = 'client disconnected "%s"' % self.sock_id
                IOMGR.queue_message('status', status)
                self._running = False
                break
            dt_recvd = datetime.now()
            try:
                dt_sent = datetime.strptime(req[:DATETIME_LENGTH],
                                            '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                warning = 'invalid datetime "%s" %s' % (self.sock_id, req)
                IOMGR.queue_message('warning', warning)
                continue
            latency = (dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY_LIMIT:
                warning = ('late request "%s" %s %5.3f'
                           % (self.sock_id, req, latency))
                IOMGR.queue_message('warning', warning)
            IOMGR.process_request(req[DATETIME_LENGTH + 1:])

    def start(self):
        self._running = True
        Thread.start(self)

    def stop(self):
        self._running = False
        self.join()
        self.sock.close()


class PiDACS:
    """
    """
    _clients = []
    _running = False
    _accept = None
    _serve = None

    @classmethod
    def _accept_connections(cls):
        srv_sock = socket(AF_INET, SOCK_STREAM)
        srv_sock.settimeout(SOCKET_TIMEOUT)
        srv_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        address_tuple = '', int(ARGS.IP_port)
        srv_sock.bind(address_tuple)
        srv_sock.listen(5)
        while cls._running:
            try:
                client_sock, client_address_tuple = srv_sock.accept()
            except timeout:
                continue
            client_sock_id = '%s:%i' % client_address_tuple
            status = 'client connected "%s"' % client_sock_id
            IOMGR.queue_message('status', status)
            client_sock.settimeout(SOCKET_TIMEOUT)
            client = Client(client_sock, client_sock_id)
            cls._clients.append(client)
            client.start()

    @classmethod
    def _serve_clients(cls):
        while cls._running:
            message = IOMGR.get_message()
            if message:
                for client in cls._clients:
                    if client._running:
                        try:
                            send_msg(client.sock, message)
                        except OSError as err_msg:
                            error = ('send error "%s"; client stopped'
                                    % client.sock_id)
                            IOMGR.queue_message('error', error)
                            IOMGR.queue_message('error', '%s' % err_msg)
                        except BrokenPipe:
                            error = ('broken pipe "%s"; client stopped'
                                     % client.sock_id)
                            IOMGR.queue_message('error', error)
                        else:
                            continue
                        IOMGR.queue_message('status', 'disconnected "%s"'
                                            % client.sock_id)
                        client.stop()
                        cls._clients.remove(client)

    @classmethod
    def start(cls):
        IOMGR.start()
        cls._running = True
        cls._accept = Thread(name='PiDACS_accept_connections',
                             target=cls._accept_connections)
        cls._accept.start()
        cls._serve = Thread(name='PiDACS_serve_clients',
                            target=cls._serve_clients)
        cls._serve.start()

    @classmethod
    def stop(cls):
        cls._running = False
        for client in cls._clients:
            client.stop()
        cls._accept.join()
        cls._serve.join()
        IOMGR.stop()

# PiDACS main program:

if __name__ == '__main__':
    PiDACS.start()
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
    PiDACS.stop()
    if ARGS.interactive:
        LOG.info('End interactive session')
