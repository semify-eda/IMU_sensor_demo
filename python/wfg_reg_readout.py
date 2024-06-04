from SmartWaveAPI import SmartWave
from SmartWaveAPI.definitions import I2CWrite, I2CRead, TriggerMode
import time
from i2ct_debug import *
from sys import argv, exit

try:
    com = argv[1]
except:
    com = None



def set_register(sw, address, data):
    print("set register: addr: %x, data: %x" % (address, data))
    sw.writeFPGARegister(address, data)


def set_register_8bit(sw, address, data):
    print("set register: addr: %x, data: %x" % (address, data))
    sw.writeFPGARegister(address, data)


def read_register(sw, address):
    data = sw.readFPGARegister(address)
    # print("read register: addr: %x, data: %x" % (address, data))
    return data


with SmartWave().connect(reset=False, port_name=com) as sw:

    # read_register(sw, 0x46000)
    # read_register(sw, 0x46004)
    # print("SoC CTRL")
    # read_register(sw, 0x48010)
    
    sw.debugCallback = lambda x : print(x)

    print("Pinmux Output Sel 0 (0x46000): \n0x%x\n" % read_register(sw, 0x46000))

    print("Pinmux Output Sel 1 (0x46004): \n0x%x\n" % read_register(sw, 0x46004))

    # print("Pinmux Output Sel 2 (0x46008): \n0x%x\n" % read_register(sw, 0x46008))

    print("Pinmux Output Sel 3 (0x4600c): \n0x%x\n" % read_register(sw, 0x4600c))

    print("Pinmux Input Sel 0 (0x46020): \n0x%x\n" % read_register(sw, 0x46020))

    print("Pinmux Input Sel 1 (0x46024): \n0x%x\n" % read_register(sw, 0x46024))

    # print("Pinmux Input Sel 2 (0x46028): \n0x%x\n" % read_register(sw, 0x46028))
    
    print("Pinmux Pullup Sel 0 (0x46014): \n0x%x\n" % read_register(sw, 0x46010))
    
    print("Pinmux Pullup Sel 1 (0x46014): \n0x%x\n" % read_register(sw, 0x46014))
    
    # print("Pinmux Pullup Sel 2 (0x46018): \n0x%x\n" % read_register(sw, 0x46018))

    # print("Button Value (0xC2FFF): \n0x%x\n" % read_register(sw, 0xC2FFF))

    # print("Timer Value (0xE0008): \n0x%x\n" % (int(read_register(sw, 0xE0008)) >> 8))

    # print("SoC Core Enable (0x48010): \n0x%x\n" % (int(read_register(sw, 0x48010)) >> 1))
   
    # print(f"SoC Status Value (0xC0FFF): \n{read_register(sw, 0xC0FFF)}\n")
    
    # x = read_register(sw, 0x880F0)

    # debug_reg_printout(x)
    # print("I2CT debug entry (0x88024): \n0x%x\n" % )
    # read_register(sw, 0x88000)

    # read_register(sw, 0x88004)


    # print("IMU Data register @ time 0")
    # set_register(sw, 0x88014, 0x2c)
    # read_register(sw, 0x88020)
    # time.sleep(0.1)
    # print("IMU Data register @ time 0.1")
    # set_register(sw, 0x88014, 0x2c)
    # read_register(sw, 0x88020)
    # time.sleep(0.1)
    # print("IMU Data register @ time 0.2")
    # set_register(sw, 0x88014, 0x2c)
    # read_register(sw, 0x88020)
