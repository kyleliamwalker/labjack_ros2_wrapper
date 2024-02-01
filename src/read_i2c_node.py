#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
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
        self.numBytes = 6

        ljm.eWriteName(self.lj_handle, "I2C_SDA_DIONUM", 4)  # SDA pin number 
        ljm.eWriteName(self.lj_handle, "I2C_SCL_DIONUM", 5)  # SCL pin number 

        # ljm.eWriteName(self.lj_handle, "I2C_SPEED_THROTTLE", 10000)

        ljm.eWriteName(self.lj_handle, "I2C_OPTIONS", 0)

        ljm.eWriteName(self.lj_handle, "I2C_SLAVE_ADDRESS", i2cAddress)

        # # Open I2C communication
        # ljm.eWriteNameByteArray(self.lj_handle, "I2C_DATA_TX", 2, [0x24, 0x00])p
        # time.sleep(0.1)

        # # Do a read only transaction to obtain the readings
        ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_TX", 0)  # Set the number of bytes to transmit
        ljm.eWriteName(self.lj_handle, "I2C_NUM_BYTES_RX", self.numBytes)  # Set the number of bytes to receive
        ljm.eWriteName(self.lj_handle, "I2C_GO", 1)  # Do the I2C communications.

        bno055_operational_mode = ljm.eReadName(self.lj_handle, "I2C_DATA_RX")
        print("BNO055 Operational Mode:", bno055_operational_mode)

        self.create_timer(0.01, self.labjack_callback)


    def labjack_callback(self):
        # # Parse BNO055 orientation data
        # aBytes = [0]*self.numBytes
        # aBytes = ljm.eReadNameArray(self.lj_handle, "I2C_DATA_RX", self.numBytes)
        # time.sleep(0.05)
        # self.get_logger().info("Raw Data: %s" % aBytes)
        # # Read I2C_ACKS
        # i2c_acks = ljm.eReadNameArray(self.lj_handle, "I2C_ACKS", 1)
        # print("I2C ACK Status:", i2c_acks[0])
        try:
            # Read I2C_ACKS before reading data
            i2c_acks = ljm.eReadNameArray(self.lj_handle, "I2C_ACKS", 1)
            print("I2C ACK Status:", i2c_acks[0])

            # Parse BNO055 orientation data
            a_bytes = ljm.eReadNameArray(self.lj_handle, "I2C_DATA_RX", self.numBytes)
            self.get_logger().info("Raw Data: %s" % a_bytes)

            # Process the data as needed

        except ljm.LJMError as e:
            self.get_logger().error(f"LabJack error: {e}")

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