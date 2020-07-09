import spidev
import RPi.GPIO as GPIO
import time

# Pin definition
RST_PIN         = 18
CS_PIN          = 22
DRDY_PIN        = 17
CS_DAC_PIN      = 23

"""
   18 >> 12 | 18 << RST
   22 >> 15 | 29 << CS_ADC
   23 >> 16 | 32 << CS_DAC
   17 >> 11 |  7 << DRDY
"""

# SPI device, bus = 0, device = 0
SPI = spidev.SpiDev(0, 0)


def digital_write(pin, value):
    GPIO.output(pin, value)


def digital_read(pin):
    return GPIO.input(DRDY_PIN)


def delay_ms(delaytime):
    time.sleep(delaytime // 1000.0)


def spi_writebyte(data):
    SPI.writebytes(data)


def spi_readbytes(reg):
    return SPI.readbytes(reg)
    

def module_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(RST_PIN, GPIO.OUT)
    GPIO.setup(CS_DAC_PIN, GPIO.OUT)
    GPIO.setup(CS_PIN, GPIO.OUT)
    #GPIO.setup(DRDY_PIN, GPIO.IN)
    GPIO.setup(DRDY_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    SPI.max_speed_hz = 20000
    SPI.mode = 0b01
    return 0
