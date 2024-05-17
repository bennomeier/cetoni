""" cetoniSerial.py

A python module to control Cetoni Nemesys syringe pumps.

- Setup your syringe node numbers with Cetoni Elements
- Install pyserial from PyPI

Now you can use this module to control the syringe via serial interface.

Copyright (2024) B. Meier
Licence: MIT
"""


import serial
import struct
import time
import telnetlib
import socket

UNSIGNED32 = "L"
INTEGER32 = "l"
UNSIGNED16 = "H"
INTEGER8 = "b"

dTypeLength = {"L" : 32, "l" : 32, "H" : 16, "b" : 8}


def dump(x):
    return ''.join([type(x).__name__, "('",*['\\x'+'{:02x}'.format(i) for i in x], "')"])


class Syringe(object):
    """This class is an interface to a Cetoni syringe pump via the serial port."""

    def __init__(self, port, baudrate = 115200, timeout = 0.2, volume = 250, steps = 6000, ser = None, node = 2):
        """Initialize the interface to the motion controller

        port : Serial port, i.e. "COM4"
        baudrate : Optional keyword argument, preset to 9600
        timeout : Optional keyword argument, preset to 1
        volume : syringe volume in uL
        steps : syringe steps
        
        if a second syringe is used, configure the nodeID of the second syringe using Cetoni Elements,
        and pass the serial interface from the first syringe (stored as the .ser attribute)
        and the nodeID. To initialize two syringes, the code will look 
        
        S1 = cetoniSer.Syringe("COM5", volume = 1000)
        S2 = cetoniSer.Syringe("", volume = 250, ser = S1.ser, node = 3)
        """
        if ser is None:
            self.ser = serial.Serial(port, baudrate, timeout = timeout)
            self.isMaster = True
        else:
            self.ser = ser
            self.isMaster = False
            
        self.nodeID = node
            
        self.volume = volume

        self.currentPosition = 0 # in increments
        self.positionIncrement = 0 # stores travel distance of relative movements
        
        # characeters for the first two frames for synchronization
        self.DLE = b'\x90'
        self.STX = b'\x02'

        self.physicalOutputs = 0
        
        # 1 read encoder resolution, gear factor and velocity SI
        self.encoderResolution = self.read(0x3000, 5, 2) # number of increments per motor revolution
        self.gearNumerator = self.read(0x3003, 1, 2)
        self.gearDenominator = self.read(0x3003, 2, 2)
        self.gearFactor = self.gearNumerator / self.gearDenominator
        self.siVelocity = self.read(0x60a9, 0, 2) # equals fdb44700 for 0.001 rpm
        
        # the following factor has the unit inc / mm
        self.positionConversionFactor = self.encoderResolution*self.gearFactor
        
        #print(hex(self.siVelocity)) 
        
        #print("RES: ", self.encoderResolution)
        
        # 2 initialize syringe parameters
        # syringes have 60 mm stroke # all units in mm and uL
        self.volume = volume
        self.crossSection = self.volume / 60
        self.volumeToIncrements = 1/self.crossSection*self.positionConversionFactor

        self.increment = 0
        self.incrementVolume = 0
        
        
        # 3 read the position limits to calculate travel range
        self.minPosLimit = self.read(0x607D, 1, 2, dtype = INTEGER32)
        self.safetyMargin = self.read(0x607D, 2, 2, dtype = INTEGER32)
        
        self.maxPos = 0
        self.minPos = self.minPosLimit + self.safetyMargin

        print("Max Pos:", self.maxPos)
        print("Min Pos: ", self.minPos) 
        
        
        # 4 read the maximum flow rate
        self.maxProfileVelocity = self.read(0x607F, 0, 2) # velocity in device units
        
        # assume siVelocity corresponds to 0.001 rpm
        maxRPM = self.maxProfileVelocity*0.001
        
        maxMMperS = maxRPM/60/ self.gearFactor
        self.maxVolperS = maxMMperS * self.crossSection # max Volume per S in uL
        
        # 5 read the product type
        self.productType = (self.read(0x210c, 3, 2) >> 10) & 0x7f #6 for Nemesys M and 7 for Nemesys S (we have 7/S)
        
        
        # 6 read mode of operation (listed as INTEGER8, but we parse it as UINT32 anyway)
        self.modeOfOperation = self.read(0x6061, 0, 2)
        print("Mode of Operation: ", self.modeOfOperation)
        
        # 7 ensure force monitoring is enabled, i.e. bit 0 is set to 0
        digitalInputPolarity = self.read(0x3141, 2, 2)
        #assert digitalInputPolarity & 0x01 == 0

        # clear bit if required
        if digitalInputPolarity & 0x01:
            mask = ~(1 << 0)
            digitalInputPolarityCleared = digitalInputPolarity & mask
            self.write(0x3141, 2, 4, digitalInputPolarityCleared)
        
        #print(bin(digitalInputPolarity))
        
        # 8 check safety stop input is off
        # this is bit 28 of digital inputs
        digitalInputs = self.read(0x60fd, 0, 2)
        assert digitalInputs & (1 << 28) == 0
        #print(digitalInputs)
        
        # 9 check if device is in fault state and clear if required
        # state register is 0x6041, fault bit is bit 3
        stateRegister = self.read(0x6041, 0, 2)
        assert stateRegister & (1 << 3) == 0
        print("STATE REGISTER: ", stateRegister) # this reads 64 = 0x40 switch on disabled
        # if we need to clear the fault, cf page 73 of the manual
        
        # 10 set pump into enabled state cf page 74
        # step 1 shut down by sending control word 0x06, control register is 0x6040 (cf page 55)
        self.write(0x6040, 0, 4, 0x06)
        print("State register after shutdown: ", self.read(0x6041, 0, 2))
        
        # step 2 send controlword 0x0f "switch on and enable operation"
        self.write(0x6040, 0, 4,0x0f)

        #time.sleep(2)
        print("State register after switch on and enable operation: ", self.read(0x6041, 0, 2))
        
        #self.write(0x1017, 0x00, 4, 400)
            
        # start positioning
        # self.write(0x6040, 0, 4, 0x0f+0x20+0x10)

        # clear new setpoint bit
        # self.write(0x6040, 0, 4, 0x0f) # clear the new setpoint bit (0x10)
        
        # examples
        # read device type
        #print("Device Type: ", hex(self.read(0x1000, 0x00, 2)))
        #print("Heartbeat Time: ", self.read(0x1017, 0x00, 2))
        
    def calculateCRC(self, msg):
        # see page 40 of the C manual
        crc = 0x0000
        shifter = 0x8000

        for word in msg:

            # swap byte order in word, not required if data packed in little endian
            # word = ((word & 0xff00) >> 8) + ((word & 0x00ff) << 8)
            shifter = 0x8000
            
            while True: # implementation of the c do while loop
                carry = crc & 0x8000
                crc = (crc << 1) & 0xffff
                
                if (word & shifter):
                    crc = (crc + 1) & 0xffff
                
                if carry:
                    crc = crc ^ 0x1021
                    
                shifter = shifter >> 1
                
                if not (shifter):
                    break
        return crc               
            
    def writeFrame(self, command):
        command = command
        commandLen = int(len(command)/2)
        
        # here we pack the data in little endian
        commandAsIntegerArray = struct.unpack("<{:d}H".format(commandLen + 1), command + b'\x00' + b'\x00')

        #print(commandAsIntegerArray)
        crc = self.calculateCRC(commandAsIntegerArray)          
        commandWithCRC =  command + int.to_bytes(crc, 2, "little")
        
        # byte stuffing
        commandStuffed = commandWithCRC.replace(self.DLE, self.DLE + self.DLE)
        #print("Stuffed: ", commandStuffed)

        frame = self.DLE + self.STX + commandStuffed        
        self.ser.write(frame)
        
    def readFrame(self):
        ans1 = self.ser.read(4)
        
        try:
            length = ans1[3]*2
        except:
            print("Error: Ans", ans1)
            
        ans2 = self.ser.read(length+2)
        
        extraBytes = 0
        stuffedByte = self.DLE + self.DLE
        for stuffedByte in ans2:
            extraBytes += 1
            
        ans2 = ans2 + self.ser.read(extraBytes)
        
        ans = ans1[2:] + ans2        
        #print(ans)
        #print(length)
        
        #remove byte stuffing
        #print("Before: ", ans)
        ans = ans.replace(self.DLE + self.DLE, self.DLE)
        ansLen = int(len(ans)/2)
        ansAsIntegerArray = struct.unpack("<{:d}H".format(ansLen), ans)
        #print(ansAsIntegerArray)
        crc =  self.calculateCRC(ansAsIntegerArray)
        
        assert crc == 0                  
        return ans
        
    def read(self, index, subindex, numberOfWords, dtype = "L"):
        # see page 47 of Nemesys V4 firmware specification
        opCommand = b'\x60'
        length = struct.pack("B", numberOfWords)
        nodeId = struct.pack("B", self.nodeID)

        index = int.to_bytes(index, 2, 'little')
        subIndex = struct.pack("B", subindex)
        
        command = opCommand + length + nodeId + index + subIndex
        #print(dump(command))

        self.writeFrame(command)
        
        frame = self.readFrame()
        frameAsIntegerArray = struct.unpack("<{:d}H".format(int(len(frame)/2)), frame)
        
        length = (frameAsIntegerArray[0] & 0xff00) >> 8

        # check communication error is 0
        assert frameAsIntegerArray[1] == 0
        assert frameAsIntegerArray[2] == 0
        
        answer = frame[6:6 + length]
                
        answer = struct.unpack("<{:d}".format(int(length/4)) + dtype, answer)
        
        if len(answer) == 1:
            answer = answer[0]
                
        return answer   
        
        
    def write(self, index, subindex, numberOfWords, value, dtype="L", read = True):
        # see page 47 of Nemesys V4 firmware specification
        opCommand = b'\x68'
        length = struct.pack("B", numberOfWords)
        nodeId = struct.pack("B", self.nodeID)

        index = int.to_bytes(index, 2, 'little')
        subIndex = struct.pack("B", subindex)
        
        #value = int.to_bytes(value, 4, 'little')
        value = struct.pack("<" + dtype, value)
        
        command = opCommand + length + nodeId + index + subIndex + value
        
        #print("Write command: ", dump(command))

        self.writeFrame(command)

        if read:
            return self.readFrame()
        else:
            return None

    def checkPos(self, pos):
        return self.minPos <= pos <= self.maxPos

    def readStatus(self):
        return self.read(0x6041, 0, 2)
    
    def switchValves(self, v1, v2):
        """
        v1: inlet valve, 1 for open, bit 16
        v2: inlet valve, 2 for open, bit 17
        """
        
        # switch valv
        # physicalOutputs = self.read(0x60fe, 1, 2)

        # clear bits 16 and 17
        self.physicalOutputs = self.physicalOutputs & ~((1<<16) + (1 << 17)) 
        
        # set only the reuqired bits
        self.physicalOutputs = self.physicalOutputs | ((v1 << 16) + (v2 << 17))
        
        #print("Physical Outputs: ", physicalOutputs)
        # set port 1 open
        answer = self.write(0x60fe, 1, 4, self.physicalOutputs, dtype="L")
        #print(answer)
        return None

    def toInput(self):
        self.switchValves(1,0)

    def toOutput(self):
        self.switchValves(0, 1)

    def readVelocity(self):
        # check velocity
        self.velocity = self.read(0x6081, 0x00, 2)
        return self.velocity

    def setVelocity(self, velocity):
        assert 0 < velocity <= self.maxProfileVelocity
        self.velocity = velocity
        return self.write(0x6081, 0, 4, velocity)
    
    def readCurrentPosition(self):
        return self.read(0x6064, 0, 2, dtype="l")

    def readTargetPosition(self):
        return self.read(0x607a, 0, 2, dtype="l")

    def writeTargetPosition(self, pos):        
        return self.write(0x607a, 0, 4, pos, dtype="l")
    
    def positionAbsolute(self, targetPosition):
        # note you have to reread the current position after calling this 
        # function.
        
        assert self.minPos < targetPosition <= self.maxPos
        self.write(0x607a, 0, 4, targetPosition, dtype="l")

        # for the control word, cf. page 77
        # we need to also set bit 4, 0x10
        #         
        self.write(0x6040, 0, 4, 0x0f+0x20+0x10)

        # clear the new setpoint bit, otherwise the next command will have no effect
        self.write(0x6040, 0, 4, 0x0f) # clear the new setpoint bit (0x10)


    def configurePositionRelativeByVolume(self, volume):
        # volume is in uL
        # negative values for aspirating, positive for dispensing.
        if volume != self.incrementVolume:                         
            self.configurePositionRelative(self.vol2increments(volume))
            self.incrementVolume = volume
        print(self.incrementVolume)
    
    # relative positioning starts with a first call to relative positioning
    def configurePositionRelative(self, increment):
        # write increment to target posistion register
        # store current position
        # mmove on to run repeat relative f

        # negative values of incremment are (presumably) for aspirating
        self.write(0x607a, 0, 4, increment, dtype="l")
        self.positionIncrement = increment

        # read current position 
        self.currentPosition = self.readCurrentPosition()

    
    def executePositionRelative(self):
        # if fast == 1, dosing will start immediately
        newPosition = self.currentPosition + self.positionIncrement
        
        if self.checkPos(newPosition):
            # move - bit 6 is 2^7 = 128
            # change immediately so bit 5 = 64
            # if the option fast is set, we abort the current dosing and start the next step

            # bit 5 (immediate dispensing has to be set), not reading the response does not work, 
            self.write(0x6040, 0, 4, 0x0f+0x20+0x10 + 128 + (1 << 5), read = True )
            
            # clear new setpoint bit
            self.write(0x6040, 0, 4, 0x0f) # clear the new setpoint bit (0x10)

            self.currentPosition = newPosition
        else:
            raise ValueError("Final position outside allowed range")
    
    def waitUntilReady(self):
        while True:
            if self.read(0x6041, 0, 2) == 1079:
                break
            time.sleep(0.1)
        return None

    
    def aspirate(self, volume):
        self.configurePositionRelativeByVolume(-volume)
        time.sleep(0.05) # wait for valves to finish switching
        self.executePositionRelative()
        self.waitUntilReady()
        return None
        
    def dispense(self, volume):
        self.configurePositionRelativeByVolume(volume)
        time.sleep(0.05)
        self.executePositionRelative()
        self.waitUntilReady()
        return None
    
    def vol2increments(self, vol):
        # the position conversion factor has dimension increments / mm
        travelLength = vol / self.crossSection

        increments = round(travelLength * self.positionConversionFactor)
        return increments

    def setFlowRate(self, flowRate):
        """ set the flow rate in uL / s."""
        assert 0 <= flowRate <= self.maxVolperS

        MMperS = flowRate /self.crossSection

        rpm = MMperS*60*self.gearFactor
        profileVelocity = rpm*1000

        self.setVelocity(int(profileVelocity))
    
    
    def close(self):
        if self.isMaster:
            self.ser.close()

                
class FakeSyringe(Syringe):
    def __init__(self):
        print("Fake Syringe Initialized - No Hardware connection")
        self.volume = 0
        self.steps = 0

    def write(self, command, syringe = 1):
        print("No Hardware Connection")
        return self.read()


    def read(self):
        return "No Hardware connection"        


if __name__ == "__main__":
    S1 = cs.Syringe("/dev/ttyUSB0", volume =1000)
    S2 = cs.Syringe("", volume = 1000, ser=S1.ser, node=3)
    
    S1.toInput()
    S1.positionAbsolute(0)
    
    # use the commands below for fast repeated dispensing or aspirating of the same volume
    # and repeat only executePositionRelative() for every dispense / aspirate action
    S1.setFlowRate(20)
    S1.currentPosition = S1.readCurrentPosition()
    S1.configurePositionRelativeByVolume(-100)
    S1.executePositionRelative()
    
    # for aspirate / dispense action
    S2.toInput()
    S2.aspirate(100)

    S2.toOutput()
    S2.dispense(50)
    
