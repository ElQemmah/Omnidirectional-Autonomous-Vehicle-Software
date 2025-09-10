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
NMEA Custom Exception Types

Created on 04 Mar 2021

:author: semuadmin
:copyright: SEMU Consulting © 2020
:license: BSD 3-Clause
"""


class NMEAParseError(Exception):
    """
    NMEA Parsing error.
    """


class NMEAStreamError(Exception):
    """
    NMEA Streaming error.
    """


class NMEAMessageError(Exception):
    """
    NMEA Undefined message class/id.
    Essentially a prompt to add missing payload types to UBX_PAYLOADS.
    """


class NMEATypeError(Exception):
    """
    NMEA Undefined payload attribute type.
    Essentially a prompt to fix incorrect payload definitions to UBX_PAYLOADS.
    """
