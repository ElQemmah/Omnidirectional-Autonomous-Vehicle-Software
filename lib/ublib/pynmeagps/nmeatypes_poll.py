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
NMEA Protocol Poll payload definitions

THESE ARE THE PAYLOAD DEFINITIONS FOR _POLL_ MESSAGES _TO_ THE RECEIVER
(e.g. Message Poll requests).

NB: Attribute names must be unique within each message id.
NB: Avoid reserved names 'msgID', 'talker', 'payload', 'checksum'.

Created on 4 Mar Sep 2021

While the NMEA 0183 © protocol is proprietary, the information here
has been collated from public domain sources.

:author: semuadmin
"""

from pynmeagps.nmeatypes_core import ST

NMEA_PAYLOADS_POLL = {
    # *********************************************
    # STANDARD MESSAGES
    # *********************************************
    "GAQ": {  # Galileo
        "msgId": ST,
    },
    "GBQ": {  # BeiDou
        "msgId": ST,
    },
    "GLQ": {  # GLONASS
        "msgId": ST,
    },
    "GNQ": {  # Any GNSS
        "msgId": ST,
    },
    "GPQ": {  # GPS, SBAS
        "msgId": ST,
    },
    "GQQ": {  # QZSS
        "msgId": ST,
    },
    # *********************************************
    # PROPRIETARY MESSAGES
    # *********************************************
    "UBX00": {
        "msgId": ST,  # '00'
    },
    "UBX03": {
        "msgId": ST,  # '03'
    },
    "UBX04": {
        "msgId": ST,  # '04'
    },
}
