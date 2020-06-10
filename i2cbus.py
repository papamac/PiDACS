"""
 PACKAGE:  Raspberry Pi Data Acquisition and Control System (PiDACS)
  MODULE:  i2cbus.py
   TITLE:  Inter-Integrated Circuit Bus (i2cbus.py)
FUNCTION:  i2cbus.py instantiates an SMBus object for the particular Raspberry
           Pi revision being used
   USAGE:  i2cbus.py is imported by main programs.  After import, the global
           variable I2CBUS contains the correct SMBus instance.  This is
           normally done using the statement:
           from i2cbus import I2Cbus
  AUTHOR:  papamac
 VERSION:  1.0.2
    DATE:  June 10, 2020


MIT LICENSE:

Copyright (c) 2018-2020 David A. Krause, aka papamac

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


DESCRIPTION:

****************************** needs work *************************************

"""
__author__ = 'papamac'
__version__ = '1.0.2'
__date__ = 'June 10, 2020'

from smbus import SMBus


def _get_raspberry_pi_revision():
    with open('/proc/cpuinfo') as lines:
        for line in lines:
            if line.startswith('Revision'):
                return line[line.index(':') + 1:].strip()
    raise RuntimeError('No Raspberry Pi revision found.')


def _get_i2c_revision():
    return 0 if _get_raspberry_pi_revision() in ('0002', '0003') else 1


I2CBUS = SMBus(_get_i2c_revision())
