#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class ReadDigital( Node ):

    def __init__(self):

        super().__init__('read_digital')
        # Open first found LabJack
        self.lj_handle = ljm.openS("T7", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.lj_handle)

        self.get_logger().info("Opened a LabJack with Device type: %i, Connection type: %i,\n"
              "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
              (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))

        # init timer to publish encoder position
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.labjack_callback)
        self.digital_pub = self.create_publisher(Float32MultiArray, '/labjack_digital', 10)

    def labjack_callback(self):

        data = ljm.eReadNames(self.lj_handle, 2, ["FIO0", "FIO1"])
        # self.get_logger().info("AIN0 : %f V, AIN1 : %f V" % (data[0], data[1]))

        # pub msg
        digital_msg = Float32MultiArray()
        digital_msg.data = data
        digital_msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        digital_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # dim[0] is AIN0
        digital_msg.layout.dim[0].label = "FIO0"
        digital_msg.layout.dim[0].size = 1
        digital_msg.layout.dim[0].stride = 1
        # dim[1] is AIN1
        digital_msg.layout.dim[1].label = "FIO1"
        digital_msg.layout.dim[1].size = 1
        digital_msg.layout.dim[1].stride = 1

        self.digital_pub.publish(digital_msg)


def main():
    rclpy.init()
    labjack = ReadDigital()
    try:
        rclpy.spin(labjack)
    except KeyboardInterrupt:
        print("Shutting down labjack node...")
    finally:
        ljm.close(labjack.lj_handle)
        labjack.destroy_node()

if __name__ == '__main__':
    main()