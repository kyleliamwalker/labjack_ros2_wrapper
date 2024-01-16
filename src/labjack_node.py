#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from labjack import ljm
from std_msgs.msg import Float32MultiArray

class LabJack( Node ):

    def __init__(self):

        super().__init__('labjack')
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
        self.analog_pub.publish(analog_msg)


def main():
    rclpy.init()
    labjack = LabJack()
    try:
        rclpy.spin(labjack)
    except KeyboardInterrupt:
        print("Shutting down labjack node...")
    finally:
        labjack.destroy_node()

if __name__ == '__main__':
    main()