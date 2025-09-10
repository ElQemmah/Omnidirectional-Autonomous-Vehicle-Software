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
Created on 27 Sep 2020

:author: semuadmin
:copyright: SEMU Consulting © 2020
:license: BSD 3-Clause
"""

from pyubx2._version import __version__

from pyubx2.exceptions import (
    UBXMessageError,
    UBXParseError,
    UBXTypeError,
    UBXStreamError,
    ParameterError,
    GNSSStreamError,
)
from pyubx2.ubxmessage import UBXMessage
from pyubx2.ubxreader import UBXReader
from pyubx2.socket_stream import SocketStream
from pyubx2.ubxtypes_core import *
from pyubx2.ubxtypes_get import *
from pyubx2.ubxtypes_poll import *
from pyubx2.ubxtypes_set import *
from pyubx2.ubxhelpers import *
from pyubx2.ubxtypes_configdb import *

version = __version__  # pylint: disable=invalid-name
