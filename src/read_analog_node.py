#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class ReadAnalog( Node ):

    def __init__(self):

        super().__init__('read_analog')
        # Open first found LabJack
        self.lj_handle = ljm.openS("T7", "ANY", "ANY")
        self.info = ljm.getHandleInfo(self.lj_handle)

        self.get_logger().info("Opened a LabJack with Device type: %i, Connection type: %i,\n"
              "Serial number: %i, IP address: %s, Port: %i,\nMax bytes per MB: %i" %
              (self.info[0], self.info[1], self.info[2], ljm.numberToIP(self.info[3]), self.info[4], self.info[5]))
        
        # AIN0 and AIN1:
        #   Negative channel = single ended (199)
        #   Range: +/-10.0 V (10.0)
        #   Resolution index = Default (0)
        #   Settling, in microseconds = Auto (0)
        self.names = ["AIN0_NEGATIVE_CH", "AIN0_RANGE", "AIN0_RESOLUTION_INDEX", "AIN0_SETTLING_US",
                      "AIN1_NEGATIVE_CH", "AIN1_RANGE", "AIN1_RESOLUTION_INDEX", "AIN1_SETTLING_US"]
        self.aValues = [199, 10.0, 0, 0, 199, 10.0, 0, 0]
        ljm.eWriteNames(self.lj_handle, len(self.names), self.names, self.aValues)

        # init timer to publish encoder position
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.labjack_callback)
        self.analog_pub = self.create_publisher(Float32MultiArray, '/labjack', 10)

    def labjack_callback(self):

        data = ljm.eReadNames(self.lj_handle, 2, ["AIN0", "AIN1"])
        # self.get_logger().info("AIN0 : %f V, AIN1 : %f V" % (data[0], data[1]))

        # pub msg
        analog_msg = Float32MultiArray()
        analog_msg.data = data
        analog_msg.layout.data_offset = 0 

        # create two dimensions in the dim array
        analog_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # dim[0] is AIN0
        analog_msg.layout.dim[0].label = "AIN0"
        analog_msg.layout.dim[0].size = 1
        analog_msg.layout.dim[0].stride = 1
        # dim[1] is AIN1
        analog_msg.layout.dim[1].label = "AIN1"
        analog_msg.layout.dim[1].size = 1
        analog_msg.layout.dim[1].stride = 1

        self.analog_pub.publish(analog_msg)


def main():
    rclpy.init()
    labjack = ReadAnalog()
    try:
        rclpy.spin(labjack)
    except KeyboardInterrupt:
        print("Shutting down labjack node...")
    finally:
        ljm.close(labjack.lj_handle)
        labjack.destroy_node()

if __name__ == '__main__':
    main()