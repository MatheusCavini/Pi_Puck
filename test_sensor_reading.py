from smbus2 import SMBus, i2c_msg
import sys
import time

I2C_CHANNEL = 12
LEGACY_I2C_CHANNEL = 4
ROB_ADDR = 0x1F
SENSORS_SIZE = (46+1) # Data + checksum.

sensors_data = bytearray([0] * SENSORS_SIZE)
prox = [0 for x in range(8)]

bus = None

def init_comm():
    global bus
    try:
        bus = SMBus(I2C_CHANNEL)
    except:
        try:
            bus = SMBus(LEGACY_I2C_CHANNEL)
        except:
            print("Cannot open I2C device")
            sys.exit(1)

def read_sensors():
    global sensors_data
    try:
        read = i2c_msg.read(ROB_ADDR, SENSORS_SIZE)
        bus.i2c_rdwr(read)
        sensors_data = list(read)
    except:
            sys.exit(1)

init_comm()

while True:
    read_sensors()
    checksum = 0
    for i in range(SENSORS_SIZE-1):
            checksum ^= sensors_data[i]

    if(checksum == sensors_data[SENSORS_SIZE-1]):
        for i in range(8):
            prox[i] = sensors_data[i*2+1]*256+sensors_data[i*2]
        print("prox: {0:4d}, {1:4d}, {2:4d}, {3:4d}, {4:4d}, {5:4d}, {6:4d}, {7:4d}\r\n".format(prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7]))
    else:
        print("Checksum error")
        sys.exit(1)
    
    time.sleep(1)