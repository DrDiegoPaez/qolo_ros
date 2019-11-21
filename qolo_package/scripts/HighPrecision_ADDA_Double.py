import time
import spidev
import mraa
from enum import IntEnum

class AD_DA:
    class data_format(IntEnum):
        voltage = 0
        bits_8 = 1
        bits_16 = 2
    def __init__(self):
        self._spi = spidev.SpiDev()
        self._spi.open(1, 0)
        # self._spi.open(1, 1)
        self._spi.mode = 0b01  # important
        self._spi.bits_per_word = 8
        self._spi.max_speed_hz = 30000
        # GPIO.setmode(GPIO.BOARD)
        # GPIO.setwarnings(False)
        # GPIO.setup(15, GPIO.OUT)  # CS AD
        # GPIO.setup(16, GPIO.OUT)  # CS DA
        # GPIO.setup(29, GPIO.OUT)  # CS AD
        # GPIO.setup(32, GPIO.OUT)  # CS DA
        # GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # DRDY
        # GPIO.setup(07, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # DRDY
        # GPIO.setup(12, GPIO.OUT)  # RST
        # GPIO.setup(18, GPIO.OUT)  # RST
        self.gpio_1 = mraa.Gpio(15) #15 22
        self.gpio_2 = mraa.Gpio(16) #16 23 
        self.gpio_3 = mraa.Gpio(29) #12 18
        self.gpio_4 = mraa.Gpio(32) #11 17
        self.gpio_5 = mraa.Gpio(11) #15 22
        self.gpio_6 = mraa.Gpio(07) #16 23 
        self.gpio_7 = mraa.Gpio(12) #12 18
        self.gpio_8 = mraa.Gpio(18) #11 17
        self.gpio_1.dir(mraa.DIR_OUT)
        self.gpio_2.dir(mraa.DIR_OUT)
        self.gpio_3.dir(mraa.DIR_OUT)
        self.gpio_4.dir(mraa.DIR_OUT)
        self.gpio_7.dir(mraa.DIR_OUT)
        self.gpio_8.dir(mraa.DIR_OUT)
        self.gpio_5.dir(mraa.DIR_IN) # DRDY
        self.gpio_5.mode(mraa.MODE_HIZ) 
        self.gpio_6.dir(mraa.DIR_IN) # DRDY
        self.gpio_6.mode(mraa.MODE_HIZ) 
        self._RST_1()
        self._RST1_1()
        self._start_adc()
        self._start_adc1()

 ####### ADS1256 ############################ADC

    def _start_adc(self):
        id = self._ReadChipID()
        if (id != 3):
            print "Error, ASD1256 Chip ID =", id
        else:
            print "Ok, ASD1256 Chip ID =", id
        self._CfgADC(self._GAIN.GAIN_1, self._DRATE_E.SPS_100)  # drate 15

    def _start_adc1(self):
        id = self._ReadChipID1()
        if (id != 3):
            print "Error, ASD1256 Chip ID =", id
        else:
            print "Ok, ASD1256 Chip ID =", id
        self._CfgADC1(self._GAIN.GAIN_1, self._DRATE_E.SPS_100)  # drate 15

    class _GAIN(IntEnum):
        GAIN_1 = 0
        GAIN_2 = 1
        GAIN_4 = 2
        GAIN_8 = 3
        GAIN_16 = 4
        GAIN_32 = 5
        GAIN_64 = 6

    class _CMD(IntEnum):
        WAKEUP = 0x00
        RDATA = 0x01
        RDATAC = 0x03
        SDATAC = 0x0F
        RREG = 0x10
        WREG = 0x50
        SELFCAL = 0xF0
        SELFOCAL = 0xF1
        SELFGCAL = 0xF2
        SYSOCAL = 0xF3
        SYSGCAL = 0xF4
        SYNC = 0xFC
        STANDBY = 0xFD
        RESET = 0xFE

    class _REG(IntEnum):
        STATUS = 0
        MUX = 1
        ADCON = 2
        DRATE = 3
        IO = 4
        OFC0 = 5
        OFC1 = 6
        OFC2 = 7
        FSC0 = 8
        FSC1 = 9
        FSC2 = 10

    class _DRATE_E(IntEnum):
        SPS_30000 = 0xF0
        SPS_15000 = 0xE0
        SPS_7500 = 0xD0
        SPS_3750 = 0xC0
        SPS_2000 = 0xB0
        SPS_1000 = 0xA1
        SPS_500 = 0x92
        SPS_100 = 0x82
        SPS_60 = 0x72
        SPS_50 = 0x63
        SPS_30 = 0x53
        SPS_25 = 0x43
        SPS_15 = 0x33
        SPS_10 = 0x20
        SPS_5 = 0x13
        SPS_2d5 = 0x03

    def _WriteReg(self,RegID, RegValue):
        self._CS_ADC_0()
        self._Send8Bit(self._CMD.WREG | RegID)
        self._Send8Bit(0x00)
        self._Send8Bit(RegValue)
        self._CS_ADC_1()
    
    def _WriteReg1(self,RegID, RegValue):
        self._CS_ADC1_0()
        self._Send8Bit(self._CMD.WREG | RegID)
        self._Send8Bit(0x00)
        self._Send8Bit(RegValue)
        self._CS_ADC1_1()

    def _CS_DAC_1(self):    #DAC
        self.gpio_2.write(1)

    def _CS_DAC_0(self):
        self.gpio_2.write(0)
    
    def _CS_DAC1_1(self):    #DAC
        self.gpio_4.write(1)
    
    def _CS_DAC1_0(self):
        self.gpio_4.write(0)

    def _CS_ADC_1(self):    #ADC
        self.gpio_1.write(1)

    def _CS_ADC_0(self):
        self.gpio_1.write(0)
    
    def _CS_ADC1_1(self):   #ADC
        self.gpio_3.write(1)

    def _CS_ADC1_0(self):
        self.gpio_3.write(0)

    def _RST_0(self):
        self.gpio_7.write(0)

    def _RST_1(self):
        self.gpio_7.write(1)

    def _RST1_0(self):
        self.gpio_8.write(0)

    def _RST1_1(self):
        self.gpio_8.write(1)

    def _WriteCmd(self,cmd):
        self._CS_ADC_0()
        self._Send8Bit(cmd)
        self._CS_ADC_1()
    
    def _WriteCmd1(self,cmd):
        self._CS_ADC1_0()
        self._Send8Bit(cmd)
        self._CS_ADC1_1()

    def _ReadChipID(self):
        self._WaitDRDY()
        id = self._ReadReg(self._REG.STATUS)
        return (id[0] >> 4)

    def _ReadChipID1(self):
        self._WaitDRDY1()
        # time.sleep(0.005)  # temp
        id = self._ReadReg1(self._REG.STATUS)
        return (id[0] >> 4)

    def _DelayDATA(self):
        time.sleep(0.00001)  # default(10us)	/* The minimum time delay 6.5us */

    def _ReadReg(self,RegID):
        self._CS_ADC_0()
        self._Send8Bit(0x10 | RegID)  # RegID
        self._Send8Bit(0x00)
        self._DelayDATA()
        read = self._Receive8Bit(1)
        self._CS_ADC_1()
        return read
    
    def _ReadReg1(self,RegID):
        self._CS_ADC1_0()
        time.sleep(0.005)  # temp
        self._Send8Bit(0x10 | RegID)  # RegID
        self._Send8Bit(0x00)
        self._DelayDATA()
        read = self._Receive8Bit(1)
        self._CS_ADC1_1()
        return read

    def _Send8Bit(self,data):
        time.sleep(0.0001)
        self._spi.xfer2([data])

    def _Receive8Bit(self,bytes):
        read = self._spi.readbytes(bytes)
        return read

    def _ReadData(self):
        self._CS_ADC_0()
        self._Send8Bit(self._CMD.RDATA)
        self._DelayDATA()
        buf = self._Receive8Bit(3)
        read = (buf[0] << 16) & 0x00FF0000
        read |= (buf[1] << 8)
        read |= buf[2]
        self._CS_ADC_1()
        if (read & 0x800000):
            read |= 0xFF000000
        return read #/ 1670

    def _ReadData1(self):
        self._CS_ADC1_0()
        self._Send8Bit(self._CMD.RDATA)
        self._DelayDATA()
        buf = self._Receive8Bit(3)
        read = (buf[0] << 16) & 0x00FF0000
        read |= (buf[1] << 8)
        read |= buf[2]
        self._CS_ADC1_1()
        if (read & 0x800000):
            read |= 0xFF000000
        return read #/ 1670
            
    def _SetChannel(self,ch):
        if (ch > 7):
            print "Max channel up to 7"
            return
        self._WriteReg(self._REG.MUX, (ch << 4) | (1 << 3))
        time.sleep(0.005)

    def _SetChannel_1(self,ch):
        if (ch > 7):
            print "Max channel up to 15"
            return
        self._WriteReg1(self._REG.MUX, (ch << 4) | (1 << 3))
        time.sleep(0.005)


    def ReadChannel(self,ch,format):
        if ch<8:
            D = self._WaitDRDY()
            if D == 0:
                Data = self._LoadChannel(ch)
        if ch>=8:
            D1 = self._WaitDRDY1()
            if D1 == 0:
                Data = self._LoadChannel_board2(ch-8)
        if format ==0: #voltage
            Data = int(Data / 1677.72)
        if format ==1: # 8bits
            Data = int(Data / 32767.5)
        if format ==2: #16bits
            Data = int(Data / 127)
        return Data
        #return 0

    def _LoadChannel(self,ch):
        self._SetChannel(ch)
        time.sleep(0.00005)  # bsp_DelayUS(5)
        self._WriteCmd(self._CMD.SYNC)
        time.sleep(0.00005)  # bsp_DelayUS(5)
        self._WriteCmd(self._CMD.WAKEUP)
        time.sleep(0.00005)  # bsp_DelayUS(25)
        return self._ReadData()

    def _LoadChannel_board2(self,ch):
        self._SetChannel_1(ch)
        time.sleep(0.00005)  # bsp_DelayUS(5)
        self._WriteCmd1(self._CMD.SYNC)
        time.sleep(0.00005)  # bsp_DelayUS(5)
        self._WriteCmd1(self._CMD.WAKEUP)
        time.sleep(0.00005)  # bsp_DelayUS(25)
        return self._ReadData1()

    def _WaitDRDY(self):
        for i in range(40000):
            if (self.gpio_5.read() == 0):
                return 0
            if (i >= 39999):
                print("ADS1256_WaitDRDY() Time Out ...")
                self._RST_0()
                time.sleep(0.01)
                self._RST_1()
                return 1


    def _WaitDRDY1(self):
        for i in range(40000):
            if (self.gpio_6.read() == 0):
                return 0
            if (i >= 39999):
                print("ADS1256_WaitDRDY1() Time Out ...")
                self._RST1_0()
                time.sleep(0.01)
                self._RST1_1()
                return 1


    def _CfgADC(self,gain, drate):
        if (self._WaitDRDY()):
            buf0 = (0 << 3) | (1 << 2) | (0 << 1)
            buf1 = 0x08
            buf2 = (0 << 5) | (0 << 3) | (gain << 0)
            buf3 = drate  # DRATE_10SPS
            self._CS_ADC_0()
            self._Send8Bit(0x50 | 0)
            self._Send8Bit(0x03)
            self._Send8Bit(buf0)
            self._Send8Bit(buf1)
            self._Send8Bit(buf2)
            self._Send8Bit(buf3)
            self._CS_ADC_1()
            time.sleep(0.05)

    def _CfgADC1(self,gain, drate):
        if (self._WaitDRDY1()):
            buf0 = (0 << 3) | (1 << 2) | (0 << 1)
            buf1 = 0x08
            buf2 = (0 << 5) | (0 << 3) | (gain << 0)
            buf3 = drate  # DRATE_10SPS
            self._CS_ADC1_0()
            self._Send8Bit(0x50 | 0)
            self._Send8Bit(0x03)
            self._Send8Bit(buf0)
            self._Send8Bit(buf1)
            self._Send8Bit(buf2)
            self._Send8Bit(buf3)
            self._CS_ADC1_1()
            time.sleep(0.05)

###################################DAC8552 (DAC)##############################

    def SET_DAC0(self,Data,format):
        if format==0:
            Data = int(Data * 13.1)
        if format==1:
            Data = Data * 257
        MSB = (Data >> 8)
        LSB = (Data & 0xff)
        self._CS_DAC_0()
        self._spi.writebytes([0x10]) #0x10
        self._spi.writebytes([MSB])
        self._spi.writebytes([LSB])
        self._CS_DAC_1()
        time.sleep(0.001)

    def SET_DAC1(self,Data,format):
        if format==0:
            Data = int(Data * 13.1)
        if format==1:
            Data = Data * 257
        MSB = (Data >> 8)
        LSB = (Data & 0xff)
        self._CS_DAC_0()
        self._spi.xfer([0x24])    #0x24
        self._spi.xfer([MSB])
        self._spi.xfer([LSB])
        self._CS_DAC_1()
        time.sleep(0.001)

    def SET_DAC2(self,Data,format):
        if format==0:
            Data = int(Data * 13.1)
        if format==1:
            Data = Data * 257
        MSB = (Data >> 8)
        LSB = (Data & 0xff)
        self._CS_DAC1_0()
        self._spi.writebytes([0x10]) #0x10
        self._spi.writebytes([MSB])
        self._spi.writebytes([LSB])
        self._CS_DAC1_1()
        time.sleep(0.001)

    def SET_DAC3(self,Data,format):
        if format==0:
            Data = int(Data * 13.1)
        if format==1:
            Data = Data * 257
        MSB = (Data >> 8)
        LSB = (Data & 0xff)
        self._CS_DAC1_0()
        self._spi.xfer([0x24])    #0x24
        self._spi.xfer([MSB])
        self._spi.xfer([LSB])
        self._CS_DAC1_1()
        time.sleep(0.001)

