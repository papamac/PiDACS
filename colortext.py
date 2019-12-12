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


# ASCII escape sequences for text attributes and colors:

NORMAL = '\033[0m'
BRIGHT = '\033[1m'
REVERSE = '\033[7m'

RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'

BRED = BRIGHT + RED
BGREEN = BRIGHT + GREEN
BYELLOW = BRIGHT + YELLOW
BBLUE = BRIGHT + BLUE


# colortext function:

def ct(attribute, text):
    return attribute + text + NORMAL
