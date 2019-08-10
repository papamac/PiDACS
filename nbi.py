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

Non-blocking console input (nbi)

"""
__author__ = 'papamac'
__version__ = '0.9.5'
__date__ = 'August 9, 2019'

from queue import *
from threading import Thread


class NBI(Thread):

    _nbi = None
    _queue = Queue()

    @classmethod
    def start(cls):
        cls._nbi = Thread(name='nbi', target=cls._queue_input, daemon=True)
        cls._running = True
        cls._nbi.start()

    @classmethod
    def _queue_input(cls):
        while True:
            cls._queue.put(input())

    @classmethod
    def get_input(cls):
        try:
            data = cls._queue.get(timeout=1)
        except Empty:
            data = None
        return data