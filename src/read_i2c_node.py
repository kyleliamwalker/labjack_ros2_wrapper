#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from random import randrange
import time

class ReadI2C( Node ):

    def __init__(self):

        super().__init__('read_i2c')
        # Open first found LabJack
        self.lj_handle = ljm.openS("T7", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.lj_handle)

        self.get_logger().info("Opened a LabJack with Device type: %i, Connection type: %i,\n"
              "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
              (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))
        
        i2cAddress = 0x28
        numBytes = 6
        i2c_data = [i2cAddress << 1, 0xAA, 0x01, 0x0F]

        ljm.eWriteName(self.lj_handle, "I2C_SDA_DIONUM", 4)  # SDA pin number = 1 (FIO1)
        ljm.eWriteName(self.lj_handle, "I2C_SCL_DIONUM", 5)  # SCL pin number = 0 (FIO0)

        # ljm.eWriteName(self.lj_handle, "I2C_SPEED_THROTTLE", 1000)

        ljm.eWriteName(self.lj_handle, "I2C_OPTIONS", 0)

        ljm.eWriteName(self.lj_handle, "I2C_SLAVE_ADDRESS", i2cAddress)

        # # Open I2C communication
        # ljm.eWriteNameByteArray(self.lj_handle, "I2C_DATA_TX", 2, [0x24, 0x00])
        # ljm.eWriteName(self.lj_handle, "I2C_GO", 1)  # Do the I2C communications.

        # # Wait for I2C transaction to complete
        # time.sleep(0.1)

        # # Do a read only transaction to obtain the readings
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_TX", 0)  # Set the number of bytes to transmit
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_RX", numBytes)  # Set the number of bytes to receive
        # ljm.eWriteName(self.lj_handle, "I2C_GO", 1)  # Do the I2C communications.

        ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_TX", len(i2c_data))
        ljm.eWriteNameArray(self.lj_handle, "I2C_DATA_TX", len(i2c_data), i2c_data)
        ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_RX", numBytes)
        ljm.eWriteName(self.lj_handle, "I2C_GO", 1)

        try:
            while True:
                # Parse BNO055 orientation data
                aBytes = [0]*numBytes
                aBytes = ljm.eReadNameByteArray(self.lj_handle, "I2C_DATA_RX", numBytes)
                self.get_logger().info("Raw Data: %s" % aBytes)
                heading = int(aBytes[1] << 8) | int(aBytes[0])
                roll = int(aBytes[3] << 8) | int(aBytes[2])
                pitch = int(aBytes[5] << 8) | int(aBytes[4])
                heading_deg = heading/16.0
                roll_deg = roll/16.0
                pitch_deg = pitch/16.0
                self.get_logger().info("Heading: %s, Roll: %s, Pitch:, %s" % (heading, roll, pitch))
        except KeyboardInterrupt:
            print('interrupted!')

        # # Speed throttle is inversely proportional to clock frequency. 0 = max.
        # ljm.eWriteName(self.lj_handle, "I2C_SPEED_THROTTLE", 65516)  # Speed throttle = 65516 (~100 kHz)

        # # Options bits:
        # #     bit0: Reset the I2C bus.
        # #     bit1: Restart w/o stop
        # #     bit2: Disable clock stretching.
        # ljm.eWriteName(self.lj_handle, "I2C_OPTIONS", 0)  # Options = 0

        # ljm.eWriteName(self.lj_handle, "I2C_SLAVE_ADDRESS", 80)  # Slave Address of the I2C chip = 80 (0x50)

        # # Initial read of EEPROM bytes 0-3 in the user memory area. We need a single I2C
        # # transmission that writes the chip's memory pointer and then reads the data.
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_TX", 1)  # Set the number of bytes to transmit
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_RX", 4)  # Set the number of bytes to receive

        # # Set the TX bytes. We are sending 1 byte for the address.
        # numBytes = 1
        # aBytes = [0]  # Byte 0: Memory pointer = 0
        # ljm.eWriteNameByteArray(self.lj_handle, "I2C_DATA_TX", numBytes, aBytes)
        # time.sleep(0.5)
        # ljm.eWriteName(self.lj_handle, "I2C_GO", 1)  # Do the I2C communications.

        # # Read the RX bytes.
        # numBytes = 4
        # # aBytes[0] to aBytes[3] will contain the data
        # aBytes = [0]*4
        # aBytes = ljm.eReadNameByteArray(self.lj_handle, "I2C_DATA_RX", numBytes)

        # self.get_logger().info("\nRead User Memory [0-3] = %s" %
        #     " ".join([("%.0f" % val) for val in aBytes]))

        # # Write EEPROM bytes 0-3 in the user memory area, using the page write
        # # technique.  Note that page writes are limited to 16 bytes max, and time.sleep(0.5)
        # self.get_logger().info("Write User Memory [0-3] = %s" %
        #     " ".join([("%.0f" % val) for val in aBytes[1:]]))

        # # Final read of EEPROM bytes 0-3 in the user memory area. We need a single I2C
        # # transmission that writes the address and then reads the data.
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_TX", 1)  # Set the number of bytes to transmit
        # ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_RX", 4)  # Set the number of bytes to receive

        # # Set the TX bytes. We are sending 1 byte for the address.
        # numBytes = 1
        # aBytes = [0]  # Byte 0: Memory pointer = 0
        # ljm.eWriteNameByteArray(self.lj_handle, "I2C_DATA_TX", numBytes, aBytes)

        # ljm.eWriteName(self.lj_handle, "I2C_GO", 1)  # Do the I2C communications.

        # # Read the RX bytes.
        # numBytes = 4
        # # aBytes[0] to aBytes[3] will contain the data
        # aBytes = [0]*4
        # aBytes = ljm.eReadNameByteArray(self.lj_handle, "I2C_DATA_RX", numBytes)

        # self.get_logger().info("Read User Memory [0-3] = %s" %
        #     " ".join([("%.0f" % val) for val in aBytes]))

def main():
    rclpy.init()
    labjack = ReadI2C()
    try:
        rclpy.spin(labjack)
    except KeyboardInterrupt:
        print("Shutting down labjack node...")
    finally:
        ljm.close(labjack.lj_handle)
        labjack.destroy_node()

if __name__ == '__main__':
    main()