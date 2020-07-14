import spidev
import mraa
import time

from .ADS1256 import ADS1256
from .DAC8532 import DAC8532

"""    0 | 1
   >> 12 | 18 << RST
   >> 15 | 29 << CS_ADC
   >> 16 | 32 << CS_DAC
   >> 11 |  7 << DRDY
"""

class ADDA:
    HIGH = 1
    LOW = 0

    def __init__(self):
        # SPI device, bus = 1, device = 0
        self.SPI = spidev.SpiDev(1, 0)

        # Pins
        self.RST_PIN_0 = mraa.Gpio(12)
        self.CS_PIN_0 = mraa.Gpio(15)
        self.DRDY_PIN_0 = mraa.Gpio(11)
        self.CS_DAC_PIN_0 = mraa.Gpio(16)

        self.RST_PIN_1 = mraa.Gpio(18)
        self.CS_PIN_1 = mraa.Gpio(29)
        self.DRDY_PIN_1 = mraa.Gpio(7)
        self.CS_DAC_PIN_1 = mraa.Gpio(32)

        self.ADS1256_0 = ADS1256(self, self.RST_PIN_0, self.CS_PIN_0, self.DRDY_PIN_0)
        self.ADS1256_1 = ADS1256(self, self.RST_PIN_1, self.CS_PIN_1, self.DRDY_PIN_1)

        self.DAC8532_0 = DAC8532(self, self.CS_DAC_PIN_0)
        self.DAC8532_1 = DAC8532(self, self.CS_DAC_PIN_1)

        self.module_init()
        self.ADS1256_0.ADS1256_init()
        self.ADS1256_1.ADS1256_init()

    def ReadChannel(self, channel):
        if channel < 8:
            data = self.ADS1256_0.ADS1256_GetChannalValue(channel)
        else:
            data = self.ADS1256_1.ADS1256_GetChannalValue(channel - 8)
        return (data / 1677.72)
        
    def SetChannel(self, channel, data):
        if channel < 2:
            self.DAC8532_0.DAC8532_Out_Voltage(channel, data)
        else:
            self.DAC8532_1.DAC8532_Out_Voltage(channel - 2, data)
        pass

    def digital_write(self, pin, value):
        pin.write(value)

    def digital_read(self, pin):
        return pin.read()

    def delay_ms(self, delaytime):
        time.sleep(delaytime // 1000.0)

    def spi_writebyte(self, data):
        self.SPI.writebytes(data)

    def spi_readbytes(self, reg):
        return self.SPI.readbytes(reg)

    def module_init(self):
        self.RST_PIN_0.dir(mraa.DIR_OUT)
        self.CS_PIN_0.dir(mraa.DIR_OUT)
        self.CS_DAC_PIN_0.dir(mraa.DIR_OUT)
        self.DRDY_PIN_0.dir(mraa.DIR_IN)
        self.DRDY_PIN_0.mode(mraa.MODE_PULLUP)

        self.CS_PIN_1.dir(mraa.DIR_OUT)
        self.RST_PIN_1.dir(mraa.DIR_OUT)
        self.CS_DAC_PIN_1.dir(mraa.DIR_OUT)
        self.DRDY_PIN_1.dir(mraa.DIR_IN)
        self.DRDY_PIN_1.mode(mraa.MODE_PULLUP) 

        self.SPI.max_speed_hz = 20000
        self.SPI.mode = 0b01

        return 0