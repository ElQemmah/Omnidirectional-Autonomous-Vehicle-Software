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
RTCM Custom Exception Types

Created on 14 Feb 2022

:author: semuadmin
:copyright: SEMU Consulting © 2022
:license: BSD 3-Clause
"""


class ParameterError(Exception):
    """Parameter Error Class."""


class RTCMParseError(Exception):
    """
    RTCM Parsing error.
    """


class RTCMStreamError(Exception):
    """
    RTCM Streaming error.
    """


class RTCMMessageError(Exception):
    """
    RTCM Undefined message class/id.
    Essentially a prompt to add missing payload types to rtcm_PAYLOADS.
    """


class RTCMTypeError(Exception):
    """
    RTCM Undefined payload attribute type.
    Essentially a prompt to fix incorrect payload definitions to rtcm_PAYLOADS.
    """
