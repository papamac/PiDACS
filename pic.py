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
__version__ = '0.9.5'
__date__ = 'August 9, 2019'

from threading import Thread

from nbi import NBI
from pidacs_global import *


class PIC:
    running = False
    _print = None
    _address = None
    _socket = None
    _dt_recvd = None

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
        cls._socket.close()

    @classmethod
    def _connect_to_server(cls):
        print('Enter address:port number, return for defaults, '
              'or quit to exit')
        cls._address = DEFAULT_ADDRESS
        port_number = DEFAULT_PORT_NUMBER
        response = input()
        if response:
            if 'quit'.startswith(response.lower()):
                print('End interactive client')
                exit()
            if ':' in response:
                cls._address, port_number = response.split(':')
                if not port_number.isdigit():
                    print('Port number not an unsigned integer')
                    cls._connect_to_server()
                    return
                port_number = int(port_number)
                if port_number not in DYNAMIC_PORT_RANGE:
                    print('Port number is not in dynamic port range')
                    cls._connect_to_server()
                    return
            else:
                cls._address = response
        try:
            ipv4_address = gethostbyname(cls._address)
        except OSError as err:
            print('Address resolution error "%s" %s' % (cls._address, err))
            cls._connect_to_server()
            return
        cls._socket = socket(AF_INET, SOCK_STREAM)
        cls._socket.settimeout(SOCKET_TIMEOUT)
        try:
            cls._socket.connect((ipv4_address, port_number))
        except OSError as err:
            print('Connection error "%s" %s' % (cls._address, err))
            cls._connect_to_server()
            return
        socket_id = '%s:%i' % cls._socket.getsockname()
        print('Connected to server "%s" via socket "%s"'
              % (cls._address, socket_id))

    @classmethod
    def _print_server_messages(cls):
        while cls.running:
            try:
                message = recv_msg(cls, cls._socket, cls._dt_recvd)
                if not cls.running:
                    continue
            except OSError as err:
                err_msg = 'Recv error "%s" %s' % (cls._address, err)
                break
            except BrokenPipe:
                err_msg = 'Server disconnected "%s"' % cls._address
                break
            except ServerTimeout:
                err_msg = 'Server timeout "%s"' % cls._address
                break
            cls._dt_recvd = datetime.now()
            dt_message = message[:DATETIME_LENGTH]
            try:
                dt_sent = datetime.strptime(dt_message, '%Y-%m-%d %H:%M:%S.%f')
            except ValueError:
                print('******** invalid datetime "%s" ********' % dt_message)
                continue
            latency = (cls._dt_recvd - dt_sent).total_seconds()
            if latency > LATENCY:
                print('******** late message; latency = %3.1f sec********'
                      % latency)
            print(message[DATETIME_LENGTH + 1:])
        else:
            return

        print(err_msg)
        cls.running = False

    @classmethod
    def send_request(cls, request):
        if cls.running:
            request = '%s %s' % (datetime.now(), request)
            try:
                send_msg(cls._socket, request)
            except OSError as err:
                err_msg = 'Send error "%s" %s' % (cls._address, err)
            except BrokenPipe:
                err_msg = 'Broken pipe to server "%s"' % cls._address
            else:
                return

            print(err_msg)


# Interactive client main program:

print('\nBegin PiDACS Interactive Client (PIC)')
PIC.start()
NBI.start()
try:
    while PIC.running:
        user_request = NBI.get_input()
        if user_request is None:
            continue
        if user_request and 'quit'.startswith(user_request.lower()):
            PIC.running = False
            continue
        PIC.send_request(user_request)
except KeyboardInterrupt:
    PIC.running = False
PIC.stop()
print('End PiDACS Interactive Client')
