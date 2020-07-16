"""ADDA Module.

This module implements ADS1256 and DAC8532 modules.
It also implements class ADDA for 2 modules each.

Extended from waveshare library for RaspberryPi at
https://github.com/waveshare/High-Precision-AD-DA-Board
"""

__all__ = ['ADDA', 'ADS1256', 'DAC8532']
__version__ = '0.1'
__author__ = 'Vaibhav Gupta'


# Exports
from .ADDA import ADDA
from .ADS1256 import ADS1256
from .DAC8532 import DAC8532