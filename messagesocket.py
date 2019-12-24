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
__version__ = '1.0.2'
__date__ = 'December 23, 2019'


from binascii import crc32
from datetime import datetime
from logging import getLogger
from math import sqrt
from socket import *
from threading import Thread, Lock

from colortext import *


# Globals:

LOG = getLogger('Plugin.messagesocket')

CRC_LEN = 8                      # CRC length (bytes).
SEQ_LEN = 8                      # Sequence number length (bytes).
HEX_LEN = CRC_LEN + SEQ_LEN      # Hex segment length (bytes).
DT_LEN = 26                      # Datetime length (bytes).
HDR_LEN = HEX_LEN + DT_LEN       # Total header length (bytes).
DATA_LEN = 120                   # Data segment length (bytes).
MSG_LEN = HDR_LEN + DATA_LEN     # Total fixed message length (bytes).
SOCKET_TIMEOUT = 10.0            # Timeout limit for socket connection, recv,
#                                  and send methods (sec).
DEFAULT_STATUS_INTERVAL = 600.0  # Default status reporting interval (sec).


def next_seq(seq):
    return seq + 1 if seq < 0xffffffff else 0


class SocketStatus:

    # Private methods:

    def __init__(self, name, status_interval):
        self._name = name
        self._status_interval = status_interval
        self._lock = Lock()
        self._min = None
        self._max = None
        self._recv_seq = None
        self._init()

    def _init(self):
        self._shorts = self._crc_errs = self._dt_errs = self._seq_errs = 0
        self._send_errs = self._send_timeouts = 0
        self._recvd = self._sent = 0
        self._min = 1000000.0
        self._max = self._sum = self._sum2 = 0.0
        self._status_dt = datetime.now()

    def _report(self):

        # Report data if the current status interval has expired.

        with self._lock:
            interval = (datetime.now() - self._status_dt).total_seconds()
            if interval >= self._status_interval:
                min_ = 0 if self._min == 1000000.0 else self._min
                avg = self._sum / self._recvd if self._recvd else 0.0
                std = (sqrt(self._sum2 / self._recvd - avg * avg)
                       if self._recvd else 0.0)
                recv_rate = self._recvd / interval
                recv_status = ('recv[%i %i %i %i|%i %i %i %i|%i %i]'
                               % (self._shorts, self._crc_errs, self._dt_errs,
                                  self._seq_errs, min_, self._max, avg, std,
                                  self._recvd, recv_rate))
                if ((self._shorts + self._crc_errs + self._dt_errs
                        + self._seq_errs)
                        or self._max > 1000.0 * SOCKET_TIMEOUT):
                    recv_status = ct(REVERSE, recv_status)

                send_rate = self._sent / interval
                send_status = ('send[%i %i|%i %i]'
                               % (self._send_errs, self._send_timeouts,
                                  self._sent, send_rate))
                if self._send_errs + self._send_timeouts:
                    send_status = ct(REVERSE, send_status)

                LOG.debug('status "%s" %s %s'
                          % (self._name, recv_status, send_status))
                self._init()  # Initialize status data for the next interval.

    # Public methods.

    def update_name(self, name):
        self._name = name

    def recv(self, message, recvd_dt):

        # Check for message header for short messages, crc errors, datetime
        # errors and sequence errors.

        if len(message) < HDR_LEN:  # Check for short message.
            self._shorts += 1
            self._report()
            return ''  # Return null message for soft error.
        crc_msg = int(message[:CRC_LEN], 16)  # Check for CRC error.
        crc_calc = crc32(message.encode()[CRC_LEN:]) & 0xffffffff  # 2.7 & 3.x
        if crc_msg != crc_calc:
            self._crc_errs += 1
            self._report()
            return ''  # Return null message for soft error.
        try:  # Check for datetime error.
            msg_dt = datetime.strptime(message[HEX_LEN:HDR_LEN],
                                       '%Y-%m-%d|%H:%M:%S.%f')
        except ValueError:
            self._dt_errs += 1
            self._report()
            return ''  # Return null message for soft error.
        msg_seq = int(message[CRC_LEN:HEX_LEN], 16)  # Check for sequence error
        if self._recv_seq is not None:
            if msg_seq != self._recv_seq:
                self._seq_errs += 1
        self._recv_seq = msg_seq

        # Update message count, sequence number, datetime, and latency data.

        self._recvd += 1
        self._recv_seq = next_seq(self._recv_seq)
        latency = 1000.0 * (recvd_dt - msg_dt).total_seconds()
        self._min = min(latency, self._min)
        self._max = max(latency, self._max)
        self._sum += latency
        self._sum2 += latency * latency
        self._report()
        return message[HDR_LEN:]  # Good message; return it without header.

    def send(self, err):
        if err is None:
            self._sent += 1
        else:
            self._send_errs += 1
        self._report()

    def send_timeout(self):
        self._send_timeouts += 1
        self._report()


class MessageSocket(Thread):

    # Private methods.

    def __init__(self, address, sock,
                 reference_name=None,
                 process_message=None,
                 process_error=None,
                 recv_timeout=0.0,
                 status_interval=DEFAULT_STATUS_INTERVAL):
        sock.settimeout(SOCKET_TIMEOUT)
        ipv4, port = sock.getpeername()
        name = '%s[%s:%s]' % (address, ipv4, port)
        Thread.__init__(self, name=name)
        self._socket = sock
        self._reference_name = reference_name
        self._process_message = process_message
        self._process_error = process_error
        self._recv_timeout = recv_timeout
        self._status = SocketStatus(name, status_interval)
        self._recvd_dt = datetime.now()
        self._send_seq = 0
        self._running = True              # Socket is open and running.
        self.running = False              # Read thread is running.

    # Public methods.

    def update_name(self, name):
        self.name = name
        self._status.update_name(name)

    def start(self):
        self.running = True
        Thread.start(self)

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join()
        if self._running:
            self._socket.shutdown(SHUT_RDWR)
            self._socket.close()

    def run(self):
        while self.running:
            message = self.recv()
            if message:
                if self._reference_name:
                    self.process_message(self._reference_name, message)
                else:
                    self.process_message(message)

    def process_message(self, *args):
        if self._process_message:
            self._process_message(*args)

    def recv(self):

        # Receive a fixed-length message in multiple segments.
        # recv has three possible returns:

        # Normal:     recv returns the message if a valid message was received.
        # Soft error: recv returns a null string if a message was received, but
        #             it contains fatal errors and cannot be processed; the
        #             socket is still running.
        # Hard error: recv returns Null if no message was received and the
        #             socket is not running.  This category includes socket
        #             exceptions and normal socket shutdown by the peer.

        if not self._running:
            return  # Return None for hard error.
        byte_msg = b''
        bytes_received = 0
        while bytes_received < MSG_LEN:

            # Try receiving a message segment and handle exceptions.

            try:
                segment = self._socket.recv(MSG_LEN - bytes_received)
            except timeout:
                if not self._running:
                    return  # Return None for hard error.
                if not self._recv_timeout:
                    continue
                interval = (datetime.now() - self._recvd_dt).total_seconds()
                if interval < self._recv_timeout:
                    continue
                self._socket.shutdown(SHUT_RDWR)
                self._socket.close()
                ex_msg = 'recv timeout "%s"' % self.name
            except OSError as oserr:
                ex_msg = ('recv error %s "%s" %s'
                          % (oserr.errno, self.name, oserr.strerror))

            # No exceptions; add non-null segment to message.

            else:
                if segment:
                    byte_msg += segment  # Valid segment.
                    bytes_received = len(byte_msg)
                    continue
                ex_msg = 'disconnected "%s"' % self.name

            # Exception return.

            if self._running:
                self._running = False
                self.running = False
                LOG.error(ct(BRED, ex_msg))
                self.process_error(self._reference_name)
            else:
                LOG.debug(ct(BBLUE, ex_msg))
            return  # Return None for hard error.

        # Full-length byte_msg received; check for errors, update status data,
        # report status, and return.

        message = byte_msg.decode().strip()
        self._recvd_dt = datetime.now()
        message = self._status.recv(message, self._recvd_dt)
        return message  # Return the message without the header or a  null
#                         string as determined by _status.recv.

    def send(self, message):
        if not self._running:
            return

        # Remove blanks and truncate if necessary.

        message = message.strip()
        if len(message) > DATA_LEN:
            LOG.warning(ct(BYELLOW, 'message truncated "%s"' % message))
            message = message[:DATA_LEN]

        # Add the crc, sequence number, and datetime to create a fixed-length
        # byte message.

        now_dt = datetime.now()
        iso_dt = now_dt.isoformat('|')
        if not now_dt.microsecond:
            iso_dt += '.000000'
        message = '%08x%s%s' % (self._send_seq, iso_dt, message)
        crc = crc32(message.encode()) & 0xffffffff  # Works with 2.7, 3.x
        message = '%08x%s' % (crc, message)
        byte_msg = message.ljust(MSG_LEN).encode()

        bytes_sent = 0
        send_err = None
        while bytes_sent < MSG_LEN:

            # Try sending a message segment and handle exceptions.

            try:
                segment_bytes_sent = self._socket.send(byte_msg[bytes_sent:])
            except timeout:
                self._status.send_timeout()
                return
            except OSError as oserr:
                ex_msg = ('send error %s "%s" %s'
                          % (oserr.errno, self.name, oserr.strerror))
            else:
                if segment_bytes_sent:
                    bytes_sent += segment_bytes_sent
                    continue
                send_err = True
                break

            # Exception return.

            if self._running:
                self._running = False
                self.running = False
                LOG.error(ct(BRED, ex_msg))
                self.process_error(self._reference_name)
            else:
                LOG.debug(ct(BBLUE, ex_msg))
            return

        # Full-length message sent or send_err; update status and return.

        self._status.send(send_err)
        self._send_seq = next_seq(self._send_seq)

    def process_error(self, *args):
        if self._process_error:
            self._process_error(*args)
