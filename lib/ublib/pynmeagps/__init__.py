#  GNU License (C) 2025 LaSa, DIMES, Univerity of Calabria

# This program is developed for LaSa, DIMES, Univerity of Calabria. 
# Its copy, use, redistribution or modification is prohibited, or requires
# you to ask for permission. All authorized modifications made to 
# the software are subject to the same conditions as the original software.
# This program is provided as is: WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# For a copy of the complete license please contact elqemmah.ay@dimes.unical.it.

# Minimal notes added; program logic unchanged.

"""
Created on 4 Mar 2021

:author: semuadmin
:copyright: SEMU Consulting © 2020
:license: BSD 3-Clause
"""
# pylint: disable=wrong-import-position, invalid-name

from pynmeagps._version import __version__
from pynmeagps.exceptions import (
    NMEAMessageError,
    NMEAParseError,
    NMEATypeError,
    NMEAStreamError,
)
from pynmeagps.nmeamessage import NMEAMessage
from pynmeagps.nmeareader import NMEAReader
from pynmeagps.socket_stream import SocketStream
from pynmeagps.nmeatypes_core import *
from pynmeagps.nmeatypes_get import *
from pynmeagps.nmeahelpers import *

version = __version__
