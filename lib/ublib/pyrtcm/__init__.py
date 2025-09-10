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
Created on 14 Feb 2022

:author: semuadmin
:copyright: SEMU Consulting © 2022
:license: BSD 3-Clause
"""

from pyrtcm._version import __version__

from pyrtcm.exceptions import (
    RTCMMessageError,
    RTCMParseError,
    RTCMTypeError,
    RTCMStreamError,
    ParameterError,
)
from pyrtcm.rtcmmessage import RTCMMessage
from pyrtcm.rtcmreader import RTCMReader
from pyrtcm.socket_stream import SocketStream
from pyrtcm.rtcmtypes_core import *
from pyrtcm.rtcmtypes_get import *
from pyrtcm.rtcmhelpers import *

version = __version__  # pylint: disable=invalid-name
