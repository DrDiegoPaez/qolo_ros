DAC_Value_MAX = 65535
DAC_VREF = 5000.0   # 3.3


class DAC8532:
    channel_A = 0x30
    channel_B = 0x34

    def __init__(self, adda, cs_pin):
        self.adda = adda
        self.cs_pin = cs_pin
        
    def DAC8532_Write_Data(self, Channel, Data):
        self.adda.digital_write(self.cs_pin, self.adda.LOW)
        self.adda.spi_writebyte([Channel, Data >> 8, Data & 0xff])
        self.adda.digital_write(self.cs_pin, self.adda.HIGH)
        
    def DAC8532_Out_Voltage(self, Channel, Voltage):
        channels = {
            0: self.channel_A
            1: self.channel_B
        }
        if (Voltage <= DAC_VREF) and (Voltage >= 0):
            temp = int(Voltage * DAC_Value_MAX / DAC_VREF)
            self.DAC8532_Write_Data(channels[Channel], temp)
