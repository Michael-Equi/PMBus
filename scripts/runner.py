#!/usr/bin/env python

import time
from PMBus import pmbus

print("Initializing PMBUS... \n")

DRQ = pmbus(0x12) #New pmbus object with device address 0x12

time.sleep(1)

DRQ.setUVLimit(36.0)

time.sleep(1)

while True:
    print("Tempurature: " + str(DRQ.getTempurature()))
    print("Input Voltage: " + str(DRQ.getVoltageIn()))
    print("Output Voltage: " + str(DRQ.getVoltageOut()))
    print("Output Current: " + str(DRQ.getCurrent()))
    print("Output Power: " + str(DRQ.getPowerOut(False)) + "\n\n") #False is caclulated from given values of current and voltage while True gets values from DRQ1250

    #DRQ.encodePMBus(34.0)

    time.sleep(5)
