from smbus import SMBus
import math

class PMBus:

    #constants initialized on object creation
    VOUT_MODE = 0b00000
    VOUT_N = 0b00000

    def __init__(self, addr, id=1):
        self.busID = id
        self.address = addr
        self.VOUT_MODE = self._readBytePMBus(0x20)
        voutN = self.VOUT_MODE & 0b00011111
        self.VOUT_N = self.twos_comp(voutN, 5)
        print("DRQ1250 succesfully connected to PMBus... \n")

    #Decode/encode Linear data format => X=Y*2^N
    def _decodePMBus(self, message):
        messageN = message >> 11
        messageY = message & 0b0000011111111111
        message = messageY*(2.0**(self.twos_comp(messageN, 5))) #calculate real values (everything but VOUT works)
        return message

    def _encodePMBus(self, message):
        YMAX = 1023.0
        #print(message)
        Nval = int(math.log(message/YMAX,2))
        #print("NVal: " + str(Nval))
        Yval = int(message*(2**-Nval))
        #print("YVal: " + str(Yval))
        message = ((Nval & 0b00011111)<<11) | Yval
        #print(bin(message))
        return message

    #wrapper functions for reading/writing a word/byte to an address with pec
    def _writeWordPMBus(self, cmd, word, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        bus.write_word_data(self.address, cmd, word)
        bus.close()

    def _readWordPMBus(self, cmd, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        data = bus.read_word_data(self.address, cmd)
        bus.close()
        return data

    def _writeBytePMBus(self, cmd, byte, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        bus.write_byte_data(self.address, cmd, byte)
        bus.close()

    def _readBytePMBus(self, cmd, pecByte=True):
        bus = SMBus(self.busID)
        bus.pec = pecByte
        data = bus.read_byte_data(self.address, cmd)
        bus.close()
        return data

    ################################### Functions for setting PMBus values
    def setVinUVLimit(self, uvLimit, minUnderVolt=32.0):
        """The VIN_UV_WARN_LIMIT command sets the value of the input voltage that causes an
        input voltage low warning. This value is typically greater than the input undervoltage
        fault threshold, VIN_UV_FAULT_LIMIT (Section 15.27). The VIN_UV_FAULT_LIMIT
        command sets the value of the input voltage that causes an input undervoltage fault."""
        #min = 32, max = 75 on DRQ1250
        if(uvLimit > minUnderVolt):
            uvWarnLimit  = float(uvLimit) + 2
            uvFaultLimit = float(uvLimit)
        else:
            uvWarnLimit  = minUnderVolt + 2
            uvFaultLimit = minUnderVolt

        #print("Old VIN UV Limit: " + str(self.getVinUVLimit()))
        self._writeWordPMBus(0x59, self._encodePMBus(uvFaultLimit))
        self._writeWordPMBus(0x58, self._encodePMBus(uvWarnLimit))
        #print("New VIN UV Limit: " + str(self.getVinUVLimit()))

    def setVinOVLimit(self, ovLimit, maxOverVolt=110.0):
        """The VIN_OV_WARN_LIMIT command sets the value of the input voltage that causes an
        input voltage high warning. This value is typically less than the input overvoltage fault
        threshold. The VIN_OV_FAULT_LIMIT command sets the value of the input voltage that causes an
        input overvoltage fault."""
        #min = 32, max = 110 on DRQ1250
        if(ovLimit < maxOverVolt):
            ovWarnLimit  = float(ovLimit) - 2
            ovFaultLimit = float(ovLimit)
        else:
            ovWarnLimit  = maxOverVolt - 2
            ovFaultLimit = maxOverVolt

        #print("Old VIN OV Limit: " + str(self.getVinOVLimit()))
        self._writeWordPMBus(0x55, self._encodePMBus(ovFaultLimit))
        self._writeWordPMBus(0x57, self._encodePMBus(ovWarnLimit))
        #print("New VIN OV Limit: " + str(self.getVinOVLimit()))

    def setVoutOVLimit(self, ovLimit, maxOverVolt=15.6):
        """The VOUT_OV_WARN_LIMIT command sets the value of the output voltage at the
        sense or output pins that causes an output voltage high warning. This value is typically
        less than the output overvoltage threshold. The VOUT_OV_FAULT_LIMIT command sets the value
        of the output voltage measured at the sense or output pins that causes an output
        overvoltage fault."""
        #min = 8.1, max=15.6 on DRQ1250
        if(ovLimit < maxOverVolt):
            ovWarnLimit  = float(ovLimit) - 1
            ovFaultLimit = float(ovLimit)
        else:
            ovWarnLimit  = maxOverVolt - 1
            ovFaultLimit = maxOverVolt

        ovWarnLimit  = int(ovWarnLimit*(2**-self.VOUT_N))
        ovFaultLimit = int(ovFaultLimit*(2**-self.VOUT_N))

        #print("Old VOUT OV Limit: " + str(self.getVoutOVLimit()))
        self._writeWordPMBus(0x40, ovFaultLimit)
        self._writeWordPMBus(0x42, ovWarnLimit)
        #print("New VOUT OV Limit: " + str(self.getVoutOVLimit()))

    def setIoutOCLimit(self, ocLimit, maxOverCurrent=65.0):
        """The IOUT_OV_WARN_LIMIT command sets the value of the output current that causes
        an output overcurrent warning. The IOUT_OC_FAULT_LIMIT command sets the value of the output current, in
        amperes, that causes the overcurrent detector to indicate an overcurrent fault condition."""
        #min = 59, max = 65 for DRQ1250
        if(ocLimit < maxOverCurrent):
            ocWarnLimit  = float(ocLimit) - 3
            ocFaultLimit = float(ocLimit)
        else:
            ocWarnLimit  = maxOverCurrent - 3
            ocFaultLimit = maxOverCurrent

        #print("Old IOUT OC Limit: " + str(self.getIoutOCLimit()))
        self._writeWordPMBus(0x46, self._encodePMBus(ocFaultLimit))
        self._writeWordPMBus(0x4A, self._encodePMBus(ocWarnLimit))
        #print("New IOUT OC Limit: " + str(self.getIoutOCLimit()))

    def setIoutFaultResponse(self, byte):
        #see page 37-40 on PMBus spec for info on response bytes
        #print("Old IOUT Fault Response: " + bin(self.getIoutFaultResponse()))
        self._writeBytePMBus(0x47, byte)
        #print("New IOUT Fault Response: " + bin(self.getIoutFaultResponse()))

    def setOTLimit(self, otLimit, maxOverTemp=145.0):
        """The OT_WARN_LIMIT command set the temperature, in degrees Celsius, of the unit at
        which it should indicate an Overtemperature Warning alarm. The OT_FAULT_LIMIT command
        set the temperature, in degrees Celsius, of the unit at which it should indicate an Overtemperature Fault."""
        #min = 30, max = 145 for DRQ1250
        if(otLimit < maxOverTemp):
            otWarnLimit  = float(otLimit) - 3
            otFaultLimit = float(otLimit)
        else:
            otWarnLimit  = maxOverTemp - 3
            otFaultLimit = maxOverTemp

        #print("Old OT Limit: " + str(self.getOTLimit()))
        self._writeWordPMBus(0x4F, self._encodePMBus(otFaultLimit))
        self._writeWordPMBus(0x51, self._encodePMBus(otWarnLimit))
        #print("New OT Limit: " + str(self.getOTLimit()))

    def setFaultResponse(self, register, byte):
        #see page 37-40 on PMBus spec for info on response bytes
        """
        DRQ1250 registers:
        VIN UV  = 0x5A
        VIN OV  = 0x56
        VOUT OV = 0x41
        OT      = 0x50
        """
        print("Old Fault Response: " + bin(self.getFaultResponse(register)))
        return self._writeBytePMBus(register, byte)
        print("New Fault Response: " + bin(self.getFaultResponse(register)))

    def setTonDelay(self, delay):
        """The TON_DELAY sets the time, in milliseconds, from when a start condition
        is received (as programmed by the ON_OFF_CONFIG command) until the output
        voltage starts to rise."""
        #max delay is 500ms min is 1ms for DRQ1250
        self._writeWordPMBus(0x60, self._encodePMBus(delay))

    def setTonRise(self, time):
        """The TON_RISE sets the time, in milliseconds, from when the output starts to rise until
        the voltage has entered the regulation band."""
        #max time is 100ms, min is 10ms for DRQ1250
        self._writeWordPMBus(0x61, self._encodePMBus(time))


    def setToffDelay(self, delay):
        """The TOFF_DELAY sets the time, in milliseconds, from a stop condition
        is received (as programmed by the ON_OFF_CONFIG command) until the unit
        stops transferring energy to the output."""
        #max delay is 500ms, min is 0ms for DRQ1250
        self._writeWordPMBus(0x64, self._encodePMBus(delay))

    def setToffFall(self, time):
        """The TOFF_FALL sets the time, in milliseconds, from the end of the turn-off delay time
        (Section 16.5) until the voltage is commanded to zero. Note that this command can only be used
        with a device whose output can sink enough current to cause the output voltage
        to decrease at a controlled rate."""
        #max time is 100ms, min is 10ms for DRQ1250
        self._writeWordPMBus(0x65, self._encodePMBus(time))


    def storeUserAll(self):
        """The STORE_USER_ALL command instructs the PMBus device to copy the entire
        contents of the Operating Memory to the matching locations in the non-volatile User
        Store memory. Any items in Operating Memory that do not have matching locations in
        the User Store are ignored."""
        self._writeBytePMBus(0x15,0x00)

    def restoreUserAll(self):
        """The RESTORE_USER_ALL command instructs the PMBus device to copy the entire
        contents of the non-volatile User Store memory to the matching locations in the
        Operating Memory. The values in the Operating Memory are overwritten by the value
        retrieved from the User Store. Any items in User Store that do not have matching
        locations in the Operating Memory are ignored."""
        self._writeBytePMBus(0x16,0x00)

    def restoreDefaultAll(self):
        """The RESTORE_DEFAULT_ALL command instructs the PMBus device to copy the entire
        contents of the non-volatile Default Store memory to the matching locations in the
        Operating Memory. The values in the Operating Memory are overwritten by the value
        retrieved from the Default Store. Any items in Default Store that do not have matching
        locations in the Operating Memory are ignored."""
        self._writeBytePMBus(0x12,0x00)

    def clearFaults(self):
        """The CLEAR_FAULTS command is used to clear any fault bits that have been set. This
        command clears all bits in all status registers simultaneously. At the same time, the
        device negates (clears, releases) its SMBALERT# signal output if the device is asserting
        the SMBALERT# signal.The CLEAR_FAULTS does not cause a unit that has latched off for a
        fault condition to restart. Units that have shut down for a fault condition are restarted
        as described in Section 10.7."""
        self._writeBytePMBus(0x03,0x00)

    #See PMBus spec page 53-54 for information on the on/off functionality
    def regOff(self, hard=False):
        if hard:
            self._writeBytePMBus(0x01,0x00) #Hard off
        else:
            self._writeBytePMBus(0x01,0x40) #Soft off

    def regOn(self):
        self._writeBytePMBus(0x01,0x80)

    ################################### Functions for getting PMBus values
    def getVoltageIn(self):
        self.voltageIn = self._decodePMBus(self._readWordPMBus(0x88))
        return self.voltageIn

    def getVoltageOut(self):
        voltageOutMessage = self._readWordPMBus(0x8B)
        self.voltageOut = voltageOutMessage*(2.0**self.VOUT_N)
        return self.voltageOut

    def getCurrent(self):
        bus = SMBus(1)
        self.current = self._decodePMBus(self._readWordPMBus(0x8C))
        bus.close()
        return self.current

    def getPowerOut(self, fromDRQ):
        if(fromDRQ == True):
            self.powerOut = self._decodePMBus(self._readWordPMBus(0x96))
        else:
            self.powerOut = self.voltageOut * self.current
        return self.powerOut

    def getTempurature(self):
        self.tempurature = self._decodePMBus(self._readWordPMBus(0x8D))
        return self.tempurature

    def getVinUVLimit(self):
        #returns fault, warn
        return self._decodePMBus(self._readWordPMBus(0x59)), self._decodePMBus(self._readWordPMBus(0x58))

    def getVinOVLimit(self):
        #returns fault, warn
        return self._decodePMBus(self._readWordPMBus(0x55)), self._decodePMBus(self._readWordPMBus(0x57))

    def getVoutOVLimit(self):
        #returns fault, warn
        return self._readWordPMBus(0x40)*(2.0**self.VOUT_N), self._readWordPMBus(0x42)*(2.0**self.VOUT_N)

    def getIoutOCLimit(self):
        #returns fault, warn
        return self._decodePMBus(self._readWordPMBus(0x46)), self._decodePMBus(self._readWordPMBus(0x4A))

    def getOTLimit(self):
        #returns fault, warn
        return self._decodePMBus(self._readWordPMBus(0x4F)), self._decodePMBus(self._readWordPMBus(0x51))

    def getTonDelay(self):
        return self._decodePMBus(self._readWordPMBus(0x60))

    def getTonRise(self):
        return self._decodePMBus(self._readWordPMBus(0x61))

    def getToffDelay(self):
        return self._decodePMBus(self._readWordPMBus(0x64))

    def getToffFall(self):
        return self._decodePMBus(self._readWordPMBus(0x65))

    def getSwitchingFreq(self):
        #returns value in kHz
        return self._decodePMBus(self._readWordPMBus(0x95))

    def getDutyCycle(self):
        #returns value in %
        return self._decodePMBus(self._readWordPMBus(0x94))

    def getIoutFaultResponse(self):
        #see page 37-40 on PMBus spec for info on response bytes
        return self._readBytePMBus(0x47)

    def getFaultResponse(self, register):
        #see page 37-40 on PMBus spec for info on response bytes
        """
        DRQ1250 registers:
        VIN UV  = 0x5A
        VIN OV  = 0x56
        VOUT OV = 0x41
        OT      = 0x50
        """
        return self._readBytePMBus(register)

    #members for getting the status of the DRQ device
    #see PMBUS spec part two pages 77-79
    def getStatusSummary(self):
        """The STATUS_WORD command returns two bytes of information with a summary of the
        unit's fault condition. Based on the information in these bytes, the host can get more
        information by reading the appropriate status registers. The low byte of the STATUS_WORD
        is the same register as the STATUS_BYTE command."""
        # BUSY | OFF | VOUT_OV_Fault | IOUT_OC_FAULT | VIN_UV_FAULT | TEMPURATURE | CML (command memory logic) | None
        # VOUT Fault | IOUT Fault | POUT  Fault | INPUT Fault | MFR_Specific | PWR_GD | Fans | Other | Unknown
        # Note: if PWR_GD is set then pwr is not good (negative logic)
        self.statusSummary = self._readWordPMBus(0x79)
        status = {
            "busy" :          bool(self.statusSummary & (0b1<<7)),
            "off" :           bool(self.statusSummary & (0b1<<6)),
            "vout_ov_fault" : bool(self.statusSummary & (0b1<<5)),
            "iout_oc_fault" : bool(self.statusSummary & (0b1<<4)),
            "vin_uv_fault" :  bool(self.statusSummary & (0b1<<3)),
            "temp_fault" :    bool(self.statusSummary & (0b1<<2)),
            "cml_fault" :     bool(self.statusSummary & (0b1<<1)),
            "vout_fault" :    bool(self.statusSummary & (0b1<<15)),
            "iout_fault" :    bool(self.statusSummary & (0b1<<14)),
            "input_fault" :   bool(self.statusSummary & (0b1<<13)),
            "pwr_gd" :        not bool(self.statusSummary & (0b1<<11)),
            "fan_fault" :     bool(self.statusSummary & (0b1<<10)),
            "other" :         bool(self.statusSummary & (0b1<<9)),
            "unknown" :       bool(self.statusSummary & (0b1<<8)),
        }
        return status, self.statusSummary

    #method for computing twos complement
    def twos_comp(self, val, bits):
        #compute the 2's complement of int value val
        if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
            val = val - (1 << bits)        # compute negative value
        return val
