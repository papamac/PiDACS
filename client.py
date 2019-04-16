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

from os import _exit
from threading import Thread

from pidacs_global import *


class Client:
    _print = None
    _srv_addr = None
    _srv_sock = None
    _dt_recvd = None
    running = False

    @classmethod
    def _connect_to_server(cls):
        print('Enter server address:port number, return for defaults, '
              'or quit to exit')
        cls._srv_addr = DEFAULT_SRV_ADDR
        port_num = DEFAULT_PORT_NUMBER
        response = input()
        if response:
            if 'quit'.startswith(response.lower()):
                print('End interactive client')
                exit()
            if ':' in response:
                cls._srv_addr, port_num = response.split(':')
                if not port_num.isdigit():
                    print('Port number not an unsigned integer')
                    cls._connect_to_server()
                    return
                port_num = int(port_num)
                if port_num not in DYNAMIC_PORT_RANGE:
                    print('Port number is not in dynamic port range')
                    cls._connect_to_server()
                    return
            else:
                cls._srv_addr = response
        try:
            ipv4_addr = gethostbyname(cls._srv_addr)
        except OSError as err_msg:
            print('Address resolution error "%s" %s'
                  % (cls._srv_addr, err_msg))
            cls._connect_to_server()
            return
        cls._srv_sock = socket(AF_INET, SOCK_STREAM)
        cls._srv_sock.settimeout(SOCKET_TIMEOUT)
        try:
            cls._srv_sock.connect((ipv4_addr, port_num))
        except OSError as err_msg:
            print('Connection error "%s" %s' % (cls._srv_addr, err_msg))
            cls._connect_to_server()
            return
        srv_sock_id = '%s:%i' % cls._srv_sock.getsockname()
        print('Connected to server "%s" via socket "%s"'
              % (cls._srv_addr, srv_sock_id))

    @classmethod
    def _print_server_messages(cls):
        err_msg = None
        while cls.running:
            message = ''
            try:
                message = recv_msg(cls, cls._srv_sock, cls._dt_recvd)
                if not cls.running:
                    continue
            except OSError as err_msg:
                err_msg = 'Recv error "%s" %s' % (cls._srv_addr, err_msg)
                break
            except BrokenPipe:
                err_msg = 'Server disconnected "%s"' % cls._srv_addr
                break
            except ServerTimeout:
                err_msg = 'Server timeout "%s"' % cls._srv_addr
                break
            cls._dt_recvd = datetime.now()
            print(message[11:19] + message[DATETIME_LENGTH:])
            try:
                dt_sent = datetime.strptime(message[:26],
                                            '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                print('******** invalid datetime "%s" ********' % message[:26])
                continue
            latency = (cls._dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY_LIMIT:
                print('******** late message %5.3f ********' % latency)
        else:
            return
        cls.running = False
        print(err_msg)
        _exit(0)

    @classmethod
    def send_request(cls, request):
        req = '%s %s' % (datetime.now(), request)
        try:
            send_msg(cls._srv_sock, req)
        except OSError as err_msg:
            err_msg = 'Send error "%s" %s' % (cls._srv_addr, err_msg)
        except BrokenPipe:
            err_msg = 'Broken pipe to server "%s"' % cls._srv_addr
        else:
            return True
        print(err_msg)
        return False

    @classmethod
    def start(cls):
        cls._connect_to_server()
        cls._dt_recvd = datetime.now()
        cls.running = True
        cls._print = Thread(name='print_server_messages',
                            target=cls._print_server_messages)
        cls._print.start()

    @classmethod
    def stop(cls):
        cls.running = False
        cls._print.join()
        cls._srv_sock.close()


# Interactive client main program:

print('\nBegin interactive client')
Client.start()
while True:
    try:
        req = input()
        if req and'quit'.startswith(req.lower()):
            break
        if not Client.send_request(req):
            break
    except KeyboardInterrupt:
        break
Client.stop()
print('End interactive client')
