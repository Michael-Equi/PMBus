from smbus import SMBus
import math

class pmbus:

    #constants initialized on object creation
    VOUT_MODE = 0b00000
    VOUT_N = 0b00000

    #class instance variables
    voltageIn = 0.0
    voltageOut = 0.0
    current = 0.0
    tempurature = 0.0
    powerOut = 0.0

    def __init__(self, addr):
        self.address = addr
        bus = SMBus(1)
        bus.pec = True #untested
        self.VOUT_MODE = bus.read_byte_data(self.address, 0x20)
        bus.close()
        voutN = self.VOUT_MODE & 0b00011111
        self.VOUT_N = self.twos_comp(voutN, 5)
        print("DRQ1250 succesfully connected to PMBus... \n")

    #Decode/encode Linear data format => X=Y*2^N
    def decodePMBus(self, message):
        messageN = message >> 11
        messageY = message & 0b0000011111111111
        message = messageY*(2.0**(self.twos_comp(messageN, 5))) #calculate real values (everything but VOUT works)
        return message

    def encodePMBus(self, message):
        YMAX = 1023.0
        Nval =  int(math.log2(message/YMAX))
        #print(bin(Nval))
        Yval = round(message*(2.0**(-Nval)))
        #print(bin(Yval))
        message = (Nval << 11) | Yval
        #print(bin(message))
        return message

    def setUVLimit(self, uvLimit):

        if(uvLimit > 32):
            uvWarnLimit = float(uvLimit) + 2
            uvFaultLimit = float(uvLimit)
        else:
            uvWarnLimit = 34.0
            uvFaultLimit = 32.0

        bus = SMBus(1)
        #bus.write_byte_data(self.address, 0x10, int(0b00000000))
        bus.write_word_data(self.address, 0x59, int(uvFaultLimit)) #This causes an error
        bus.write_word_data(self.address, 0x58, int(uvWarnLimit)) #The error shows up here or if this is commented out it shows up on under the getTempurature method
        bus.close()

    def getVoltageIn(self):
        bus = SMBus(1)
        self.voltageIn = self.decodePMBus(bus.read_word_data(self.address, 0x88))
        bus.close()
        return self.voltageIn

    def getVoltageOut(self):
        bus = SMBus(1)
        voltageOutMessage = bus.read_word_data(self.address, 0x8B)
        bus.close()
        self.voltageOut = voltageOutMessage*(2.0**self.VOUT_N)
        return self.voltageOut

    def getCurrent(self):
        bus = SMBus(1)
        self.current = self.decodePMBus(bus.read_word_data(self.address, 0x8C))
        bus.close()
        return self.current

    def getPowerOut(self, fromDRQ):
        if(fromDRQ == True):
            bus = SMBus(1)
            self.powerOut = self.decodePMBus(bus.read_word_data(self.address, 0x96))
            bus.close()
        else:
            self.powerOut = self.voltageOut * self.current
        return self.powerOut

    def getTempurature(self):
        bus = SMBus(1)
        self.tempurature = self.decodePMBus(bus.read_word_data(self.address, 0x8D))
        bus.close()
        return self.tempurature

    #method for computing twos complement
    def twos_comp(self, val, bits):
        #compute the 2's complement of int value val
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val
